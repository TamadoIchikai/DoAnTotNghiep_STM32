#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
  // p178 REF - defined 32 bit register
  // AFR[2] because 2 peripherals AFRH and AFRL (high/low)
};

struct rcc {
  // p170 REF
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR;
};

struct systick {
  // p249 DUI0553A
  volatile uint32_t CTRL, LOAD, VAL, CALIB;
};

struct uart {
  // p824 REF
  volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
};

//
// Enum MODER values are per datasheet: 0, 1, 2, 3 // p185 REF
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

// p58 REF - each GPIO seperate about 0x400 away from GPIOA to GPIOB
#define GPIO(bank) ((struct gpio *)(0x40020000 + 0x400 * (bank)))
#define SYSTICK ((struct systick *)0xe000e010) // p246 PM0214
#define RCC ((struct rcc *)0x40023800)         // p58 REF
#define USART1 ((struct uart *)0x40011000)
#define USART2 ((struct uart *)0x40004400)
#define USART3 ((struct uart *)0x40004800)
#define FREQ 16000000 // CPU frequency, 16 Mhz

#define BIT(x) (1UL << (x))
// move bank to upper byte and pin to lower byte
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)
#define STM32VectorTable_Entries 96 // p236-237 REF
static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  uint16_t n = PINNO(pin);
  gpio->MODER &= ~(3U << (n * 2));      // Clear existing setting
  gpio->MODER |= (mode & 3) << (n * 2); // Set new mode
  // `&3` for keeping only the last 2 bit (defensive code)
  // `pin*2` since p185 REF defined MODER has 2 bits per pin
  // pin 0 -> bits [1:0]
  // pin 1 -> bits [3:2]
  // pin 2 -> bits [5:4]
  // pin 3 -> bits [7:6]
  RCC->AHB1ENR |= BIT(PINBANK(pin)); // Enable GPIO clock
  // AHB1 control clock from GPIOA to GPIOK
}

static inline void gpio_set_af(uint16_t pin, uint8_t af_num) {
  struct gpio *gpio = GPIO(PINBANK(pin)); // GPIO bank
  int n = PINNO(pin);                     // Pin number
  // AFR[0] and AFR[1] are 2 register for Alternate function
  // AFR[0] pin 0...7; AFR[1] pin 8...15
  gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4)); // Clear 4 bits
  // >> 3 is shift 3 bits, this is similar to divde integer 8 -> Chose AFR[x]
  // n & 7 retain the last 3 bits -> seperate pin position in 8 pins
  // * 4 position of this pin in 32 bits
  // 15 UL << mean take 0x1111 shift by pin (pos on 32 bits)
  // &= ~ will clear those 4 bits (invert 0x1111 bits to 0x0000 then and )
  gpio->AFR[n >> 3] |= ((uint32_t)af_num) << ((n & 7) * 4);
  // Write 4 bit from af_num to previous cleared 4 bits
}
static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16); // p187 REF
}

static inline void spin(volatile uint32_t count) {
  while (count--)
    (void)0;
}

static inline void systick_init(uint32_t ticks) {
  if ((ticks - 1) > 0xffffff)
    return; // Systick timer is 24 bit
  SYSTICK->LOAD = ticks - 1;
  SYSTICK->VAL = 0;
  SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2); // Enable systick
}

static volatile uint32_t s_ticks; // volatile is important!!
void SysTick_Handler(void) { s_ticks++; }

void delay(unsigned ms) {        // This function waits "ms" milliseconds
  uint32_t until = s_ticks + ms; // Time in a future when we need to stop
  while (s_ticks < until)
    (void)0; // Loop until then
}
// t: expiration time, prd: period, now: current time. Return true if expired
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t)
    *t = 0; // Time wrapped? Reset timer
  if (*t == 0)
    *t = now + prd; // First poll? Set expiration
  if (*t > now)
    return false;                               // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd; // Next expiration time
  return true;                                  // Expired, return true
}

static inline void uart_init(struct uart *uart, unsigned long baud) {
  uint8_t af = 7;          // Alternate function
  uint16_t rx = 0, tx = 0; // pins

  if (uart == USART1)
    RCC->APB2ENR |= BIT(4); // p148 REF
  if (uart == USART2)
    RCC->APB1ENR |= BIT(17); // p145 REF
  if (uart == USART3)
    RCC->APB1ENR |= BIT(18); // p145 REF

  if (uart == USART1)
    tx = PIN('A', 9), rx = PIN('A', 10); // p58 PRO-DATA
  if (uart == USART2)
    tx = PIN('A', 2), rx = PIN('A', 3); // p58 PRO-DATA
  if (uart == USART3)
    tx = PIN('D', 8), rx = PIN('D', 9); // p61 PRO-DATA
  // you can chose other port and pin with the same purpose RX/TX
  gpio_set_mode(tx, GPIO_MODE_AF);
  gpio_set_af(tx, af);
  gpio_set_mode(rx, GPIO_MODE_AF);
  gpio_set_af(rx, af);
  uart->CR1 = 0;           // Disable this UART, Clear all CR1 bits
  uart->BRR = FREQ / baud; // FREQ is a UART bus frequency
  uart->CR1 |= BIT(13) | BIT(2) | BIT(3); // Set UE, RE, TE - p817 REF
}

static inline int uart_read_ready(struct uart *uart) {
  return uart->SR & BIT(5); // If RXNE bit is set, data is ready
  // p814 REF
}

static inline uint8_t uart_read_byte(struct uart *uart) {
  return (uint8_t)(uart->DR & 255);
  // p816 REF
}

static inline void uart_write_byte(struct uart *uart, uint8_t byte) {
  uart->DR = byte;
  while ((uart->SR & BIT(7)) == 0)
    spin(1);
}

static inline void uart_write_buf(struct uart *uart, char *buf, size_t len) {
  while (len-- > 0)
    uart_write_byte(uart, *(uint8_t *)buf++);
}
int main(void) {
  uint16_t led = PIN('A', 5);           // Blue LED
  RCC->AHB1ENR |= BIT(PINBANK(led));    // Enable GPIO clock for LED
  gpio_set_mode(led, GPIO_MODE_OUTPUT); // Set blue LED to output mode
  uint32_t timer, period = 500;         // Declare timer and 500ms period
  systick_init(16000000 / 1000);        // Tick every 1 ms
  uart_init(USART2, 115200);
  for (;;) {
    if (timer_expired(&timer, period, s_ticks)) {
      static bool on = true; // This block is executed
      gpio_write(led, on);   // Every `period` milliseconds
      on = !on;              // Toggle LED state
      uart_write_buf(USART2, "hi\r\n", 4);
    }
    // Here we could perform other activities!
  }
  return 0;
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
  // memset .bss to zero, and copy .data section to RAM region
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *dst = &_sbss; dst < &_ebss; dst++)
    *dst = 0;
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;)
    *dst++ = *src++;

  main(); // Call main()
  for (;;)
    (void)0; // Infinite loop in the case if main() returns
}

extern void _estack(void); // Defined in link.ld

// 16 standard and STM32-specific handlers
__attribute__((section(
    ".vectors"))) void (*const tab[16 + STM32VectorTable_Entries])(void) = {
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler};
// `.vectors` will make compiler store the code at the start of the output
// binary, make sure the vector table is loaded first
