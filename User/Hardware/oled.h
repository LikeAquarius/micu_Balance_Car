#ifndef __OLED_H
#define __OLED_H

#include <stdbool.h>
#include <stdint.h>

#define OLED_WIDTH          128U
#define OLED_HEIGHT         64U
#define OLED_TEXT_ROWS      8U
#define OLED_TEXT_COLUMNS   21U

bool OLED_Init(void);
void OLED_DisplayOn(void);
void OLED_DisplayOff(void);
void OLED_ClearBuffer(void);
void OLED_Clear(void);
void OLED_Fill(bool on);
void OLED_Refresh(void);
void OLED_SetPixel(uint8_t x, uint8_t y, bool on);
void OLED_DrawChar(uint8_t row, uint8_t column, char character);
void OLED_Print(uint8_t row, uint8_t column, const char *text);
int oled_printf(uint8_t row, uint8_t column, const char *format, ...);

#endif
