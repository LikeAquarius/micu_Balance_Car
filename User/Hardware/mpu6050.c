#include "mpu6050.h"
#include "soft_i2c.h"
#include "debug.h"

#define MPU6050_ADDRESS       0x68U
#define MPU6050_SCL_PORT      GPIOB
#define MPU6050_SCL_PIN       GPIO_Pin_6
#define MPU6050_SDA_PORT      GPIOB
#define MPU6050_SDA_PIN       GPIO_Pin_7
#define MPU6050_I2C_DELAY     48U

#define REG_SMPLRT_DIV        0x19U
#define REG_CONFIG            0x1AU
#define REG_GYRO_CONFIG       0x1BU
#define REG_ACCEL_CONFIG      0x1CU
#define REG_ACCEL_XOUT_H      0x3BU
#define REG_PWR_MGMT_1        0x6BU
#define REG_WHO_AM_I          0x75U

static SoftI2C mpu_bus;

static bool write_register(uint8_t reg, uint8_t value)
{
    uint8_t packet[2] = {reg, value};
    return SoftI2C_Write(&mpu_bus, MPU6050_ADDRESS, packet, sizeof(packet));
}

static bool read_registers(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t index;

    if(data == NULL || length == 0U)
    {
        return false;
    }

    SoftI2C_Start(&mpu_bus);
    if(!SoftI2C_WriteByte(&mpu_bus, (uint8_t)(MPU6050_ADDRESS << 1)) ||
       !SoftI2C_WriteByte(&mpu_bus, reg))
    {
        SoftI2C_Stop(&mpu_bus);
        return false;
    }

    SoftI2C_Start(&mpu_bus);
    if(!SoftI2C_WriteByte(&mpu_bus, (uint8_t)((MPU6050_ADDRESS << 1) | 1U)))
    {
        SoftI2C_Stop(&mpu_bus);
        return false;
    }

    for(index = 0U; index < length; ++index)
    {
        data[index] = SoftI2C_ReadByte(&mpu_bus, index + 1U < length);
    }
    SoftI2C_Stop(&mpu_bus);
    return true;
}

bool MPU6050_Init(void)
{
    uint8_t who_am_i = 0U;

    SoftI2C_Init(&mpu_bus,
                 MPU6050_SCL_PORT, MPU6050_SCL_PIN,
                 MPU6050_SDA_PORT, MPU6050_SDA_PIN,
                 MPU6050_I2C_DELAY);
    Delay_Ms(50);

    if(!read_registers(REG_WHO_AM_I, &who_am_i, 1U) || who_am_i != 0x68U)
    {
        return false;
    }

    if(!write_register(REG_PWR_MGMT_1, 0x80U)) return false;
    Delay_Ms(100);
    if(!write_register(REG_PWR_MGMT_1, 0x01U)) return false;
    if(!write_register(REG_CONFIG, 0x03U)) return false;
    if(!write_register(REG_SMPLRT_DIV, 0x04U)) return false;
    if(!write_register(REG_GYRO_CONFIG, 0x08U)) return false;
    if(!write_register(REG_ACCEL_CONFIG, 0x00U)) return false;
    Delay_Ms(20);
    return true;
}

bool MPU6050_ReadSample(MPU6050_Sample *sample)
{
    uint8_t data[14];

    if(sample == NULL || !read_registers(REG_ACCEL_XOUT_H, data, sizeof(data)))
    {
        return false;
    }

    sample->ax = (int16_t)((uint16_t)data[0] << 8 | data[1]);
    sample->ay = (int16_t)((uint16_t)data[2] << 8 | data[3]);
    sample->az = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    sample->temperature = (int16_t)((uint16_t)data[6] << 8 | data[7]);
    sample->gx = (int16_t)((uint16_t)data[8] << 8 | data[9]);
    sample->gy = (int16_t)((uint16_t)data[10] << 8 | data[11]);
    sample->gz = (int16_t)((uint16_t)data[12] << 8 | data[13]);
    return true;
}
