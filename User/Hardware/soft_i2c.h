#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include "ch32v20x.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct
{
    GPIO_TypeDef *scl_port;
    uint16_t scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t sda_pin;
    uint16_t delay_cycles;
} SoftI2C;

void SoftI2C_Init(SoftI2C *bus,
                  GPIO_TypeDef *scl_port, uint16_t scl_pin,
                  GPIO_TypeDef *sda_port, uint16_t sda_pin,
                  uint16_t delay_cycles);
void SoftI2C_Start(SoftI2C *bus);
void SoftI2C_Stop(SoftI2C *bus);
bool SoftI2C_WriteByte(SoftI2C *bus, uint8_t data);
uint8_t SoftI2C_ReadByte(SoftI2C *bus, bool acknowledge);
bool SoftI2C_Write(SoftI2C *bus, uint8_t address_7bit,
                   const uint8_t *data, size_t length);
bool SoftI2C_Read(SoftI2C *bus, uint8_t address_7bit,
                  uint8_t *data, size_t length);
bool SoftI2C_IsDeviceReady(SoftI2C *bus, uint8_t address_7bit);

#endif
