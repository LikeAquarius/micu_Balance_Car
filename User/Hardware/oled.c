#include "oled.h"
#include "oledfont.h"
#include "soft_i2c.h"
#include "debug.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define OLED_I2C_ADDRESS    0x3CU
#define OLED_SCL_PORT       GPIOB
#define OLED_SCL_PIN        GPIO_Pin_0
#define OLED_SDA_PORT       GPIOB
#define OLED_SDA_PIN        GPIO_Pin_1
#define OLED_I2C_DELAY      48U
#define OLED_PAGE_COUNT     (OLED_HEIGHT / 8U)
#define OLED_CHAR_WIDTH     6U
#define OLED_PRINTF_BUFFER  (OLED_TEXT_COLUMNS + 1U)

static SoftI2C oled_bus;
static uint8_t oled_buffer[OLED_PAGE_COUNT][OLED_WIDTH];

static bool oled_write_commands(const uint8_t *commands, uint8_t count)
{
    uint8_t packet[32];

    if(count > (uint8_t)(sizeof(packet) - 1U))
    {
        return false;
    }
    packet[0] = 0x00U;
    memcpy(&packet[1], commands, count);
    return SoftI2C_Write(&oled_bus, OLED_I2C_ADDRESS, packet, (size_t)count + 1U);
}

static bool oled_set_page_column(uint8_t page, uint8_t x)
{
    uint8_t commands[3];

    commands[0] = (uint8_t)(0xB0U | page);
    commands[1] = (uint8_t)(x & 0x0FU);
    commands[2] = (uint8_t)(0x10U | (x >> 4));
    return oled_write_commands(commands, 3U);
}

bool OLED_Init(void)
{
    static const uint8_t init_commands[] =
    {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0x2E, 0xAF
    };

    SoftI2C_Init(&oled_bus,
                 OLED_SCL_PORT, OLED_SCL_PIN,
                 OLED_SDA_PORT, OLED_SDA_PIN,
                 OLED_I2C_DELAY);
    Delay_Ms(100);

    if(!SoftI2C_IsDeviceReady(&oled_bus, OLED_I2C_ADDRESS))
    {
        return false;
    }
    if(!oled_write_commands(init_commands, (uint8_t)sizeof(init_commands)))
    {
        return false;
    }

    OLED_Clear();
    return true;
}

void OLED_DisplayOn(void)
{
    const uint8_t commands[] = {0x8D, 0x14, 0xAF};
    (void)oled_write_commands(commands, (uint8_t)sizeof(commands));
}

void OLED_DisplayOff(void)
{
    const uint8_t commands[] = {0xAE, 0x8D, 0x10};
    (void)oled_write_commands(commands, (uint8_t)sizeof(commands));
}

void OLED_ClearBuffer(void)
{
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

void OLED_Clear(void)
{
    OLED_ClearBuffer();
    OLED_Refresh();
}

void OLED_Fill(bool on)
{
    memset(oled_buffer, on ? 0xFF : 0x00, sizeof(oled_buffer));
    OLED_Refresh();
}

void OLED_Refresh(void)
{
    uint8_t page;
    uint8_t packet[OLED_WIDTH + 1U];

    packet[0] = 0x40U;
    for(page = 0; page < OLED_PAGE_COUNT; ++page)
    {
        memcpy(&packet[1], oled_buffer[page], OLED_WIDTH);
        if(oled_set_page_column(page, 0U))
        {
            (void)SoftI2C_Write(&oled_bus, OLED_I2C_ADDRESS,
                                packet, sizeof(packet));
        }
    }
}

void OLED_SetPixel(uint8_t x, uint8_t y, bool on)
{
    uint8_t mask;

    if(x >= OLED_WIDTH || y >= OLED_HEIGHT)
    {
        return;
    }
    mask = (uint8_t)(1U << (y & 7U));
    if(on)
    {
        oled_buffer[y >> 3][x] |= mask;
    }
    else
    {
        oled_buffer[y >> 3][x] &= (uint8_t)~mask;
    }
}

void OLED_DrawChar(uint8_t row, uint8_t column, char character)
{
    uint8_t index;
    uint8_t x;

    if(row >= OLED_TEXT_ROWS || column >= OLED_TEXT_COLUMNS)
    {
        return;
    }
    if(character < ' ' || character > '~')
    {
        character = '?';
    }

    x = (uint8_t)(column * OLED_CHAR_WIDTH);
    for(index = 0; index < OLED_CHAR_WIDTH; ++index)
    {
        oled_buffer[row][x + index] = F6x8[(uint8_t)character - (uint8_t)' '][index];
    }
}

void OLED_Print(uint8_t row, uint8_t column, const char *text)
{
    if(text == NULL)
    {
        return;
    }

    while(*text != '\0' && row < OLED_TEXT_ROWS)
    {
        if(*text == '\n')
        {
            ++row;
            column = 0;
            ++text;
            continue;
        }
        if(column >= OLED_TEXT_COLUMNS)
        {
            ++row;
            column = 0;
            continue;
        }
        OLED_DrawChar(row, column, *text++);
        ++column;
    }
}

int oled_printf(uint8_t row, uint8_t column, const char *format, ...)
{
    char text[OLED_PRINTF_BUFFER];
    va_list arguments;
    int result;

    if(format == NULL || row >= OLED_TEXT_ROWS || column >= OLED_TEXT_COLUMNS)
    {
        return -1;
    }

    va_start(arguments, format);
    result = vsnprintf(text, sizeof(text), format, arguments);
    va_end(arguments);

    OLED_Print(row, column, text);
    OLED_Refresh();
    return result;
}
