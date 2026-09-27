#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

void Encoder_Init(void);
void Encoder_Reset(void);
int16_t Encoder_ReadLeft(void);
int16_t Encoder_ReadRight(void);

#endif
