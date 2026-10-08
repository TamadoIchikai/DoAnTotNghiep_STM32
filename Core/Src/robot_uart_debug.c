#include "robot_uart_debug.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define SOF_0                       0xAAU
#define SOF_1                       0x55U
#define PROTOCOL_VERSION            1U

#define MSG_PING                    0x01U
#define MSG_PICK_PLACE              0x10U
#define MSG_STATUS                  0x80U
#define MSG_PONG                    0x81U

#define STATUS_READY                0U
#define STATUS_ACK                  1U
#define STATUS_STARTED              2U
#define STATUS_PICKED               3U
#define STATUS_PLACED               4U
#define STATUS_DONE                 5U
#define STATUS_BUSY                 6U
#define STATUS_REJECTED             7U

#define ERROR_NONE                  0U
#define ERROR_CRC                   1U
#define ERROR_BAD_PAYLOAD           3U
#define ERROR_BUSY                  4U

#define HEADER_SIZE                 6U
#define MAX_PAYLOAD                 64U
#define PICK_PLACE_PAYLOAD_SIZE     20U
#define STATUS_PAYLOAD_SIZE         4U
#define MAX_TX_EVENTS               16U

typedef enum
{
    PARSER_WAIT_SOF_0 = 0,
    PARSER_WAIT_SOF_1,
    PARSER_READ_HEADER,
    PARSER_READ_PAYLOAD,
    PARSER_READ_CRC_0,
    PARSER_READ_CRC_1
} ParserState;

typedef enum
{
    TX_EVENT_PONG = 0,
    TX_EVENT_STATUS
} TxEventType;

typedef struct
{
    TxEventType type;
    uint16_t command_id;
    uint8_t status;
    uint8_t error;
} TxEvent;

typedef struct
{
    uint16_t command_id;
    int32_t source_x_centi_mm;
    int32_t source_y_centi_mm;
    int32_t destination_x_centi_mm;
    int32_t destination_y_centi_mm;
    uint8_t source_action;
    uint8_t destination_action;
    uint8_t motion_profile;
    uint8_t reserved;
} PickPlaceCommand;

typedef enum
{
    EXEC_IDLE = 0,
    EXEC_WAIT_STARTED,
    EXEC_WAIT_PICKED,
    EXEC_WAIT_PLACED,
    EXEC_WAIT_DONE
} ExecState;

static UART_HandleTypeDef *s_pi_uart = NULL;
static uint8_t s_rx_byte = 0U;

static ParserState s_parser_state = PARSER_WAIT_SOF_0;
static uint8_t s_header[HEADER_SIZE];
static uint8_t s_payload[MAX_PAYLOAD];
static uint8_t s_header_index = 0U;
static uint16_t s_payload_index = 0U;
static uint16_t s_payload_length = 0U;
static uint16_t s_received_crc = 0U;

static volatile uint8_t s_tx_head = 0U;
static volatile uint8_t s_tx_tail = 0U;
static TxEvent s_tx_events[MAX_TX_EVENTS];

static volatile bool s_command_ready = false;
static PickPlaceCommand s_pending_command;

static ExecState s_exec_state = EXEC_IDLE;
static PickPlaceCommand s_active_command;
static uint32_t s_exec_deadline_ms = 0U;
static uint8_t s_current_status = STATUS_READY;
static uint16_t s_current_command_id = 0xFFFFU;

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int32_t read_i32_le(const uint8_t *p)
{
    uint32_t value =
        ((uint32_t)p[0]) |
        ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24);
    return (int32_t)value;
}

static void write_u16_le(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;

        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static void parser_reset(void)
{
    s_parser_state = PARSER_WAIT_SOF_0;
    s_header_index = 0U;
    s_payload_index = 0U;
    s_payload_length = 0U;
    s_received_crc = 0U;
}

static bool tx_queue_push_isr(
    TxEventType type,
    uint16_t command_id,
    uint8_t status,
    uint8_t error
)
{
    const uint8_t next =
        (uint8_t)((s_tx_head + 1U) % MAX_TX_EVENTS);

    if (next == s_tx_tail)
    {
        return false;
    }

    s_tx_events[s_tx_head].type = type;
    s_tx_events[s_tx_head].command_id = command_id;
    s_tx_events[s_tx_head].status = status;
    s_tx_events[s_tx_head].error = error;
    s_tx_head = next;
    return true;
}

static bool tx_queue_push_main(
    TxEventType type,
    uint16_t command_id,
    uint8_t status,
    uint8_t error
)
{
    bool result;
    __disable_irq();
    result = tx_queue_push_isr(
        type,
        command_id,
        status,
        error
    );
    __enable_irq();
    return result;
}

static bool tx_queue_pop(TxEvent *event)
{
    bool available = false;

    __disable_irq();
    if (s_tx_tail != s_tx_head)
    {
        *event = s_tx_events[s_tx_tail];
        s_tx_tail =
            (uint8_t)((s_tx_tail + 1U) % MAX_TX_EVENTS);
        available = true;
    }
    __enable_irq();

    return available;
}

static void send_frame(
    uint8_t message_type,
    uint16_t command_id,
    const uint8_t *payload,
    uint16_t payload_length
)
{
    uint8_t frame[2U + HEADER_SIZE + MAX_PAYLOAD + 2U];
    uint16_t index = 0U;

    frame[index++] = SOF_0;
    frame[index++] = SOF_1;

    frame[index++] = PROTOCOL_VERSION;
    frame[index++] = message_type;
    write_u16_le(&frame[index], payload_length);
    index += 2U;
    write_u16_le(&frame[index], command_id);
    index += 2U;

    if (payload_length > 0U && payload != NULL)
    {
        memcpy(&frame[index], payload, payload_length);
        index += payload_length;
    }

    const uint16_t crc =
        crc16_ccitt(
            &frame[2],
            (uint16_t)(HEADER_SIZE + payload_length)
        );

    write_u16_le(&frame[index], crc);
    index += 2U;

    // DEBUG OUTPUT: Print the transmitted binary frame
    printf("[UART1 TX] ");
    for (uint16_t i = 0; i < index; i++) {
        printf("%02X ", frame[i]);
    }
    printf("\r\n");

    (void)HAL_UART_Transmit(
        s_pi_uart,
        frame,
        index,
        20U
    );
}

static void process_one_tx_event(void)
{
    TxEvent event;
    if (!tx_queue_pop(&event))
    {
        return;
    }

    if (event.type == TX_EVENT_PONG)
    {
        send_frame(
            MSG_PONG,
            event.command_id,
            NULL,
            0U
        );
        return;
    }

    uint8_t payload[STATUS_PAYLOAD_SIZE];
    payload[0] = event.status;
    payload[1] = event.error;
    payload[2] = 0U;
    payload[3] = 0U;

    send_frame(
        MSG_STATUS,
        event.command_id,
        payload,
        STATUS_PAYLOAD_SIZE
    );
}

static void handle_complete_frame_isr(void)
{
    const uint8_t version = s_header[0];
    const uint8_t message_type = s_header[1];
    const uint16_t payload_length =
        read_u16_le(&s_header[2]);
    const uint16_t command_id =
        read_u16_le(&s_header[4]);

    uint8_t crc_buffer[HEADER_SIZE + MAX_PAYLOAD];
    memcpy(
        crc_buffer,
        s_header,
        HEADER_SIZE
    );
    memcpy(
        &crc_buffer[HEADER_SIZE],
        s_payload,
        payload_length
    );

    const uint16_t calculated_crc =
        crc16_ccitt(
            crc_buffer,
            (uint16_t)(HEADER_SIZE + payload_length)
        );

    // DEBUG OUTPUT: Print the received binary frame
    printf("[UART1 RX] ");
    for (int i = 0; i < HEADER_SIZE; i++) {
        printf("%02X ", s_header[i]);
    }
    for (int i = 0; i < payload_length; i++) {
        printf("%02X ", s_payload[i]);
    }
    printf("%02X %02X (Calc CRC: %04X)\r\n", s_received_crc & 0xFF, s_received_crc >> 8, calculated_crc);

    if (calculated_crc != s_received_crc)
    {
        (void)tx_queue_push_isr(
            TX_EVENT_STATUS,
            command_id,
            STATUS_REJECTED,
            ERROR_CRC
        );
        return;
    }

    if (version != PROTOCOL_VERSION)
    {
        printf("[UART1 RX] Error: version mismatch (got %u, expected %u)\r\n", version, PROTOCOL_VERSION);
        return;
    }

    if (message_type == MSG_PING && payload_length == 0U)
    {
        (void)tx_queue_push_isr(
            TX_EVENT_PONG,
            command_id,
            0U,
            0U
        );
        return;
    }

    if (
        message_type != MSG_PICK_PLACE ||
        payload_length != PICK_PLACE_PAYLOAD_SIZE
    )
    {
        (void)tx_queue_push_isr(
            TX_EVENT_STATUS,
            command_id,
            STATUS_REJECTED,
            ERROR_BAD_PAYLOAD
        );
        return;
    }

    if (
        command_id == s_current_command_id &&
        s_current_command_id != 0xFFFFU
    )
    {
        (void)tx_queue_push_isr(
            TX_EVENT_STATUS,
            command_id,
            s_current_status,
            ERROR_NONE
        );
        return;
    }

    if (
        s_command_ready ||
        s_exec_state != EXEC_IDLE
    )
    {
        (void)tx_queue_push_isr(
            TX_EVENT_STATUS,
            command_id,
            STATUS_BUSY,
            ERROR_BUSY
        );
        return;
    }

    PickPlaceCommand command;
    command.command_id = command_id;
    command.source_x_centi_mm =
        read_i32_le(&s_payload[0]);
    command.source_y_centi_mm =
        read_i32_le(&s_payload[4]);
    command.destination_x_centi_mm =
        read_i32_le(&s_payload[8]);
    command.destination_y_centi_mm =
        read_i32_le(&s_payload[12]);
    command.source_action = s_payload[16];
    command.destination_action = s_payload[17];
    command.motion_profile = s_payload[18];
    command.reserved = s_payload[19];

    s_pending_command = command;
    s_command_ready = true;

    s_current_command_id = command_id;
    s_current_status = STATUS_ACK;

    (void)tx_queue_push_isr(
        TX_EVENT_STATUS,
        command_id,
        STATUS_ACK,
        ERROR_NONE
    );
}

static void parser_feed_isr(uint8_t byte)
{
    switch (s_parser_state)
    {
        case PARSER_WAIT_SOF_0:
            if (byte == SOF_0)
            {
                s_parser_state =
                    PARSER_WAIT_SOF_1;
            }
            break;

        case PARSER_WAIT_SOF_1:
            if (byte == SOF_1)
            {
                s_header_index = 0U;
                s_parser_state =
                    PARSER_READ_HEADER;
            }
            else
            {
                s_parser_state =
                    (byte == SOF_0) ?
                    PARSER_WAIT_SOF_1 :
                    PARSER_WAIT_SOF_0;
            }
            break;

        case PARSER_READ_HEADER:
            s_header[s_header_index++] = byte;

            if (s_header_index >= HEADER_SIZE)
            {
                s_payload_length =
                    read_u16_le(&s_header[2]);

                if (
                    s_payload_length >
                    MAX_PAYLOAD
                )
                {
                    printf("[UART1 RX] Error: payload length %u > MAX (%u)\r\n", s_payload_length, MAX_PAYLOAD);
                    parser_reset();
                }
                else if (
                    s_payload_length == 0U
                )
                {
                    s_parser_state =
                        PARSER_READ_CRC_0;
                }
                else
                {
                    s_payload_index = 0U;
                    s_parser_state =
                        PARSER_READ_PAYLOAD;
                }
            }
            break;

        case PARSER_READ_PAYLOAD:
            s_payload[s_payload_index++] = byte;

            if (
                s_payload_index >=
                s_payload_length
            )
            {
                s_parser_state =
                    PARSER_READ_CRC_0;
            }
            break;

        case PARSER_READ_CRC_0:
            s_received_crc = byte;
            s_parser_state =
                PARSER_READ_CRC_1;
            break;

        case PARSER_READ_CRC_1:
            s_received_crc |=
                (uint16_t)byte << 8;
            handle_complete_frame_isr();
            parser_reset();
            break;

        default:
            parser_reset();
            break;
    }
}

static void print_centi_mm(
    const char *name,
    int32_t value
)
{
    const bool negative = value < 0;
    uint32_t magnitude = negative ?
        (uint32_t)(-value) :
        (uint32_t)value;

    printf(
        "%s: %s%lu.%02lu mm\r\n",
        name,
        negative ? "-" : "",
        (unsigned long)(magnitude / 100U),
        (unsigned long)(magnitude % 100U)
    );
}

static void start_pending_command(void)
{
    bool available = false;
    PickPlaceCommand command;

    __disable_irq();
    if (s_command_ready)
    {
        command = s_pending_command;
        s_command_ready = false;
        available = true;
    }
    __enable_irq();

    if (!available)
    {
        return;
    }

    s_active_command = command;

    printf(
        "\r\n========== UART2 RX PICK_PLACE ==========\r\n"
    );
    printf(
        "command_id: %u\r\n",
        command.command_id
    );
    print_centi_mm(
        "source_x",
        command.source_x_centi_mm
    );
    print_centi_mm(
        "source_y",
        command.source_y_centi_mm
    );
    printf(
        "source_action: MAGNET_%s (%u)\r\n",
        command.source_action == 1U ?
            "ON" : "OFF",
        command.source_action
    );
    print_centi_mm(
        "destination_x",
        command.destination_x_centi_mm
    );
    print_centi_mm(
        "destination_y",
        command.destination_y_centi_mm
    );
    printf(
        "destination_action: MAGNET_%s (%u)\r\n",
        command.destination_action == 0U ?
            "OFF" : "ON",
        command.destination_action
    );
    printf(
        "motion_profile: %u\r\n",
        command.motion_profile
    );
    printf(
        "=========================================\r\n"
    );

    s_current_status = STATUS_STARTED;
    (void)tx_queue_push_main(
        TX_EVENT_STATUS,
        command.command_id,
        STATUS_STARTED,
        ERROR_NONE
    );

    /* ======= MODIFICATION 1: Route to CLI parser ======= */
    /* Synthesize text command from binary payload values.  */
    /* The CLI parser handles _mm conversion automatically. */
    /* Binary payload is in centi-mm (0.01 mm), so divide by 100 to get mm */
    float pick_x  = (float)command.source_x_centi_mm / 100.0f;
    float pick_y  = (float)command.source_y_centi_mm / 100.0f;
    float place_x = (float)command.destination_x_centi_mm / 100.0f;
    float place_y = (float)command.destination_y_centi_mm / 100.0f;

    char synthetic_cmd[128];
    snprintf(synthetic_cmd, sizeof(synthetic_cmd),
             "auto_con_1_mag_on_0_%.3f_%.3f_mag_off_%.3f_%.3f_mm",
             pick_x, pick_y, place_x, place_y);

    extern void Process_Command(char *cmd);
    Process_Command(synthetic_cmd);

    /* Robot_Control.c state machine will call               */
    /* RobotUartDebug_ReportStatus() at each transition.     */
    s_exec_state = EXEC_WAIT_DONE;
    /* ====== END MODIFICATION 1 ====== */
}

static void process_executor(void)
{
    const uint32_t now = HAL_GetTick();

    if (s_exec_state == EXEC_IDLE)
    {
        start_pending_command();
        return;
    }

    if (
        (int32_t)(now - s_exec_deadline_ms) < 0
    )
    {
        return;
    }

    switch (s_exec_state)
    {
        case EXEC_WAIT_PICKED:
            HAL_GPIO_WritePin(
                GPIOA,
                GPIO_PIN_5,
                GPIO_PIN_SET
            );
            s_current_status = STATUS_PICKED;
            (void)tx_queue_push_main(
                TX_EVENT_STATUS,
                s_active_command.command_id,
                STATUS_PICKED,
                ERROR_NONE
            );
            printf(
                "MAGNET TEST LED PA5: ON\r\n"
            );
            s_exec_deadline_ms = now + 500U;
            s_exec_state = EXEC_WAIT_PLACED;
            break;

        case EXEC_WAIT_PLACED:
            HAL_GPIO_WritePin(
                GPIOA,
                GPIO_PIN_5,
                GPIO_PIN_RESET
            );
            s_current_status = STATUS_PLACED;
            (void)tx_queue_push_main(
                TX_EVENT_STATUS,
                s_active_command.command_id,
                STATUS_PLACED,
                ERROR_NONE
            );
            printf(
                "MAGNET TEST LED PA5: OFF\r\n"
            );
            s_exec_deadline_ms = now + 200U;
            s_exec_state = EXEC_WAIT_DONE;
            break;

        case EXEC_WAIT_DONE:
            s_current_status = STATUS_DONE;
            (void)tx_queue_push_main(
                TX_EVENT_STATUS,
                s_active_command.command_id,
                STATUS_DONE,
                ERROR_NONE
            );
            printf(
                "TX STATUS: DONE\r\n"
            );
            s_exec_state = EXEC_IDLE;
            break;

        default:
            s_exec_state = EXEC_IDLE;
            break;
    }
}

void RobotUartDebug_Init(
    UART_HandleTypeDef *pi_uart
)
{
    s_pi_uart = pi_uart;
    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_command_ready = false;
    s_exec_state = EXEC_IDLE;
    s_current_command_id = 0xFFFFU;
    s_current_status = STATUS_READY;
    parser_reset();

    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_5,
        GPIO_PIN_RESET
    );
}

void RobotUartDebug_StartReceive(void)
{
    if (s_pi_uart != NULL)
    {
        (void)HAL_UART_Receive_IT(
            s_pi_uart,
            &s_rx_byte,
            1U
        );
    }
}

void RobotUartDebug_UART_RxCpltCallback(
    UART_HandleTypeDef *huart
)
{
    if (huart != s_pi_uart)
    {
        return;
    }

    /* Feed byte to binary protocol parser */
    parser_feed_isr(s_rx_byte);

    (void)HAL_UART_Receive_IT(
        s_pi_uart,
        &s_rx_byte,
        1U
    );
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_pi_uart)
    {
        // If an overrun or framing error occurs, the HAL aborts reception.
        // We MUST restart it to keep the connection alive!
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        
        (void)HAL_UART_Receive_IT(
            s_pi_uart,
            &s_rx_byte,
            1U
        );
    }
}

void RobotUartDebug_Process(void)
{
    /*
     * Keep this function running quickly in while(1).
     * PONG heartbeat and status frames are sent here,
     * never from the UART receive interrupt.
     */
    process_one_tx_event();
    process_executor();
}

/* ======= MODIFICATION 2: Status reporting for Robot_Control.c ======= */
void RobotUartDebug_ReportStatus(uint8_t status)
{
    if (s_current_command_id == 0xFFFFU) return;

    s_current_status = status;
    (void)tx_queue_push_main(
        TX_EVENT_STATUS,
        s_current_command_id,
        status,
        ERROR_NONE
    );

    printf("TX STATUS: %u\r\n", status);

    if (status == STATUS_DONE) {
        s_exec_state = EXEC_IDLE;
        s_current_command_id = 0xFFFFU;
    }
}
/* ====== END MODIFICATION 2 ====== */
