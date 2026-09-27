#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temperature;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} MPU6050_Sample;

bool MPU6050_Init(void);
bool MPU6050_ReadSample(MPU6050_Sample *sample);

#endif
