BUILD_DIR := debug

SOURCES = src/main.c
LINKER_SCRIPT = lnk/link.ld

ELF = $(BUILD_DIR)/firmware.elf
BIN = $(BUILD_DIR)/firmware.bin

CFLAGS ?= -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
          -Wformat-truncation -fno-common -Wconversion \
          -g3 -Os -ffunction-sections -fdata-sections -I. -Isrc \
          -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
          $(EXTRA_CFLAGS)

LDFLAGS ?= -T$(LINKER_SCRIPT) -nostartfiles -nostdlib \
           --specs=nano.specs -lc -lgcc -Wl,--gc-sections

.PHONY: all build flash clean

all: build

build: $(ELF)

$(BUILD_DIR):
	mkdir -p $@

$(ELF): $(SOURCES) $(LINKER_SCRIPT) | $(BUILD_DIR)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -Wl,-Map=$@.map -o $@

$(BIN): $(ELF)
	arm-none-eabi-objcopy -O binary $< $@

flash: $(BIN)
	st-flash --reset write $< 0x8000000

clean:
	rm -f $(ELF) $(BIN) $(ELF).map
