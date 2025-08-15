#ifndef LORA_SX1276_H
#define LORA_SX1276_H

#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"

void lora_init(void);
int  lora_send_packet(uint8_t *buf, uint8_t len);
uint8_t lora_read_reg(uint8_t addr);
void lora_write_reg(uint8_t addr, uint8_t val);
void lora_set_frequency(uint32_t freq);
void lora_reset(void);

#endif