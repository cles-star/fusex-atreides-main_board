#include "stm32f4xx_hal.h"
#include "main.h"

#define MPU9250_CS_PIN MPU6500_CS_Pin
#define MPU9250_CS_PORT MPU6500_CS_GPIO_Port

// Déclaration des adresses des registres MPU9250
#define MPU9250_REG_PWR_MGMT_1   0x6B
#define MPU9250_REG_SMPLRT_DIV   0x19
#define MPU9250_REG_CONFIG       0x1A
#define MPU9250_REG_GYRO_CONFIG  0x1B
#define MPU9250_REG_ACCEL_CONFIG 0x1C

void MPU9250_CS_LOW();
void MPU9250_CS_HIGH();
void MPU9250_SPI_Write(uint8_t reg, uint8_t data, SPI_HandleTypeDef *hspi);
void MPU9250_SPI_Read(uint8_t reg, uint8_t* data, uint16_t size, SPI_HandleTypeDef *hspi);
void MPU9250_Init(SPI_HandleTypeDef hspi);
void MPU9250_ReadAccel(int16_t* accelData, SPI_HandleTypeDef *hspi);
void MPU9250_ReadGyro(int16_t* gyroData, SPI_HandleTypeDef *hspi);
