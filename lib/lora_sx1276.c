#include "lora_sx1276.h"
#include <string.h>
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

// --- Pin mapping (ajuste conforme seu wiring) ---
#define PIN_SCK   18
#define PIN_MOSI  19
#define PIN_MISO  16
#define PIN_CS    17
#define PIN_RST   20
#define PIN_DIO0  22

// --- SX1276 register addresses (usados) ---
#define REG_FIFO                0x00
#define REG_OPMODE              0x01
#define REG_FRF_MSB             0x06
#define REG_FRF_MID             0x07
#define REG_FRF_LSB             0x08
#define REG_PA_CONFIG           0x09
#define REG_LNA                 0x0C
#define REG_FIFO_ADDR_PTR       0x0D
#define REG_FIFO_TX_BASE_ADDR   0x0E
#define REG_IRQ_FLAGS           0x12
#define REG_RX_NB_BYTES         0x13
#define REG_PKT_SNR_VALUE       0x19
#define REG_PKT_RSSI_VALUE      0x1A
#define REG_MODEM_CONFIG1       0x1D
#define REG_MODEM_CONFIG2       0x1E
#define REG_SYMB_TIMEOUT_LSB    0x1F
#define REG_PREAMBLE_MSB        0x20
#define REG_PREAMBLE_LSB        0x21
#define REG_PAYLOAD_LENGTH      0x22
#define REG_MODEM_CONFIG3       0x26
#define REG_DIO_MAPPING1        0x40
#define REG_VERSION             0x42
#define REG_PA_DAC              0x4D

// Useful masks / values
#define MAP_DIO0_LORA_TXDONE    0x40
#define IRQ_TX_DONE_MASK        0x08

// SPI instance
#define LORA_SPI spi0
#define SPI_BAUD 1000000  // 1 MHz to start; pode aumentar se quiser

// helper: CS control
static inline void cs_select(void){ gpio_put(PIN_CS, 0); }
static inline void cs_unselect(void){ gpio_put(PIN_CS, 1); }

void lora_write_reg(uint8_t addr, uint8_t val){
    uint8_t buf[2];
    buf[0] = addr | 0x80; // MSB=1 => write (datasheet)
    buf[1] = val;
    cs_select();
    spi_write_blocking(LORA_SPI, buf, 2);
    cs_unselect();
}

uint8_t lora_read_reg(uint8_t addr){
    uint8_t tx[2] = { addr & 0x7F, 0x00 }; // tx[0]: MSB=0 => read, tx[1]: exigência do protocolo spi
    uint8_t rx[2];
    cs_select();
    spi_write_read_blocking(LORA_SPI, tx, rx, 2);
    cs_unselect();
    return rx[1];
}

void lora_reset(void){
    gpio_put(PIN_RST, 0);
    sleep_ms(10);
    gpio_put(PIN_RST, 1);
    sleep_ms(10);
}

void lora_set_frequency(uint32_t freq){
    // frf = (freq << 19) / 32e6
    uint64_t frf = ((uint64_t)freq << 19) / 32000000ULL;
    lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void lora_init(void){
    // SPI pins
    spi_init(LORA_SPI, SPI_BAUD);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    // CS, RST, DIO0
    gpio_init(PIN_CS); gpio_set_dir(PIN_CS, GPIO_OUT); cs_unselect();
    gpio_init(PIN_RST); gpio_set_dir(PIN_RST, GPIO_OUT); gpio_put(PIN_RST, 1);
    gpio_init(PIN_DIO0); gpio_set_dir(PIN_DIO0, GPIO_IN);

    // Reset module
    lora_reset();

    // Put in LoRa sleep mode to allow configuration of long-range mode
    lora_write_reg(REG_OPMODE, 0x80); // LoRa mode + sleep

    // Check version
    uint8_t ver = lora_read_reg(REG_VERSION);
    // expected 0x12 for many SX1276 modules
    if (ver == 0x00 || ver == 0xFF) {
        // possible wiring issue
        printf("Erro: comunicação SPI não está funcionando (talvez um fio solto).\n");
    }

    // Frequency
    lora_set_frequency(915000000UL); // 915 MHz

    // Config modem registers:
    // Use common practical values:
    // REG_MODEM_CONFIG1 (0x1D) -> BW=125kHz, CR=4/5, explicit header
    // REG_MODEM_CONFIG2 (0x1E) -> SF=7, CRC on
    // REG_MODEM_CONFIG3 (0x26) -> LowDataRateOptimize off / AGC on
    lora_write_reg(REG_MODEM_CONFIG1, 0x72); // example value (BW125, CR4/5, Explicit)
    lora_write_reg(REG_MODEM_CONFIG2, 0x74); // SF7, CRC on
    lora_write_reg(REG_MODEM_CONFIG3, 0x04); // AGC auto on

    // Preamble length (8)
    lora_write_reg(REG_PREAMBLE_MSB, 0x00);
    lora_write_reg(REG_PREAMBLE_LSB, 0x08);

    // Set base addresses for FIFO
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    // Set output power (PA_BOOST)
    // For ~14 dBm: PA = 0x80 | (14 - 2) = 0x8C
    lora_write_reg(REG_PA_CONFIG, 0x8C);

    // Clear all IRQ flags
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);

    // Map DIO0 => TxDone (for interrupt usage if desired)
    lora_write_reg(REG_DIO_MAPPING1, MAP_DIO0_LORA_TXDONE);

    // Set to standby
    lora_write_reg(REG_OPMODE, 0x81); // LoRa + Standby
}

int lora_send_packet(uint8_t *buf, uint8_t len){
    if (len == 0 || len > 255) return -1;

    // Set FIFO ptr to TX base
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    // Write payload to FIFO
    for (uint8_t i = 0; i < len; ++i){
        lora_write_reg(REG_FIFO, buf[i]);
    }

    // Set payload length
    lora_write_reg(REG_PAYLOAD_LENGTH, len);

    // Clear IRQ flags
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);

    // Start transmission
    lora_write_reg(REG_OPMODE, 0x83); // LoRa + TX

    // Wait for TX done (poll)
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    while (1) {
        uint8_t irq = lora_read_reg(REG_IRQ_FLAGS);
        if (irq & IRQ_TX_DONE_MASK) break;
        // timeout 5s
        if (to_ms_since_boot(get_absolute_time()) - t0 > 5000) {
            // timeout
            return -2;
        }
        sleep_ms(10);
    }

    // Clear TxDone
    lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

    // Set back to standby
    lora_write_reg(REG_OPMODE, 0x81);

    return 0;
}