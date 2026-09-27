#include "soft_i2c.h"
#include "bsp_gpio.h"

#define SOFT_I2C_CLOCK_TIMEOUT  1000U

static void i2c_delay(const SoftI2C *bus)
{
    volatile uint16_t count = bus->delay_cycles;
    while(count-- != 0U)
    {
        __asm volatile("nop");
    }
}

static void sda_output(SoftI2C *bus)
{
    BSP_GPIO_ConfigOutput(bus->sda_port, bus->sda_pin, GPIO_Mode_Out_OD);
}

static void sda_input(SoftI2C *bus)
{
    BSP_GPIO_ConfigInput(bus->sda_port, bus->sda_pin, BSP_GPIO_INPUT_PULL_UP);
}

static void scl_low(SoftI2C *bus)
{
    BSP_GPIO_Write(bus->scl_port, bus->scl_pin, false);
}

static bool scl_release(SoftI2C *bus)
{
    uint16_t timeout = SOFT_I2C_CLOCK_TIMEOUT;

    BSP_GPIO_Write(bus->scl_port, bus->scl_pin, true);
    while(!BSP_GPIO_Read(bus->scl_port, bus->scl_pin))
    {
        if(timeout-- == 0U)
        {
            return false;
        }
    }
    return true;
}

static void sda_write(SoftI2C *bus, bool high)
{
    BSP_GPIO_Write(bus->sda_port, bus->sda_pin, high);
}

void SoftI2C_Init(SoftI2C *bus,
                  GPIO_TypeDef *scl_port, uint16_t scl_pin,
                  GPIO_TypeDef *sda_port, uint16_t sda_pin,
                  uint16_t delay_cycles)
{
    uint8_t pulse;

    bus->scl_port = scl_port;
    bus->scl_pin = scl_pin;
    bus->sda_port = sda_port;
    bus->sda_pin = sda_pin;
    bus->delay_cycles = delay_cycles;

    BSP_GPIO_ConfigOutput(scl_port, scl_pin, GPIO_Mode_Out_OD);
    BSP_GPIO_ConfigOutput(sda_port, sda_pin, GPIO_Mode_Out_OD);
    sda_write(bus, true);
    (void)scl_release(bus);

    for(pulse = 0; pulse < 9U && !BSP_GPIO_Read(sda_port, sda_pin); ++pulse)
    {
        scl_low(bus);
        i2c_delay(bus);
        (void)scl_release(bus);
        i2c_delay(bus);
    }
    SoftI2C_Stop(bus);
}

void SoftI2C_Start(SoftI2C *bus)
{
    sda_output(bus);
    sda_write(bus, true);
    (void)scl_release(bus);
    i2c_delay(bus);
    sda_write(bus, false);
    i2c_delay(bus);
    scl_low(bus);
}

void SoftI2C_Stop(SoftI2C *bus)
{
    sda_output(bus);
    sda_write(bus, false);
    i2c_delay(bus);
    (void)scl_release(bus);
    i2c_delay(bus);
    sda_write(bus, true);
    i2c_delay(bus);
}

bool SoftI2C_WriteByte(SoftI2C *bus, uint8_t data)
{
    uint8_t bit;
    bool acknowledged;

    sda_output(bus);
    for(bit = 0; bit < 8U; ++bit)
    {
        sda_write(bus, (data & 0x80U) != 0U);
        data <<= 1;
        i2c_delay(bus);
        (void)scl_release(bus);
        i2c_delay(bus);
        scl_low(bus);
    }

    sda_write(bus, true);
    sda_input(bus);
    i2c_delay(bus);
    (void)scl_release(bus);
    i2c_delay(bus);
    acknowledged = !BSP_GPIO_Read(bus->sda_port, bus->sda_pin);
    scl_low(bus);
    sda_output(bus);
    sda_write(bus, true);
    return acknowledged;
}

uint8_t SoftI2C_ReadByte(SoftI2C *bus, bool acknowledge)
{
    uint8_t bit;
    uint8_t data = 0;

    sda_write(bus, true);
    sda_input(bus);
    for(bit = 0; bit < 8U; ++bit)
    {
        data <<= 1;
        (void)scl_release(bus);
        i2c_delay(bus);
        if(BSP_GPIO_Read(bus->sda_port, bus->sda_pin))
        {
            data |= 1U;
        }
        scl_low(bus);
        i2c_delay(bus);
    }

    sda_output(bus);
    sda_write(bus, !acknowledge);
    i2c_delay(bus);
    (void)scl_release(bus);
    i2c_delay(bus);
    scl_low(bus);
    sda_write(bus, true);
    return data;
}

bool SoftI2C_Write(SoftI2C *bus, uint8_t address_7bit,
                   const uint8_t *data, size_t length)
{
    size_t index;

    if((data == NULL && length != 0U) || address_7bit > 0x7FU)
    {
        return false;
    }

    SoftI2C_Start(bus);
    if(!SoftI2C_WriteByte(bus, (uint8_t)(address_7bit << 1)))
    {
        SoftI2C_Stop(bus);
        return false;
    }

    for(index = 0; index < length; ++index)
    {
        if(!SoftI2C_WriteByte(bus, data[index]))
        {
            SoftI2C_Stop(bus);
            return false;
        }
    }

    SoftI2C_Stop(bus);
    return true;
}

bool SoftI2C_Read(SoftI2C *bus, uint8_t address_7bit,
                  uint8_t *data, size_t length)
{
    size_t index;

    if((data == NULL && length != 0U) || address_7bit > 0x7FU)
    {
        return false;
    }

    SoftI2C_Start(bus);
    if(!SoftI2C_WriteByte(bus, (uint8_t)((address_7bit << 1) | 1U)))
    {
        SoftI2C_Stop(bus);
        return false;
    }

    for(index = 0; index < length; ++index)
    {
        data[index] = SoftI2C_ReadByte(bus, index + 1U < length);
    }
    SoftI2C_Stop(bus);
    return true;
}

bool SoftI2C_IsDeviceReady(SoftI2C *bus, uint8_t address_7bit)
{
    bool acknowledged;

    SoftI2C_Start(bus);
    acknowledged = SoftI2C_WriteByte(bus, (uint8_t)(address_7bit << 1));
    SoftI2C_Stop(bus);
    return acknowledged;
}
