/*
 * mpu9250.c
 *
 *  Created on: Dec 28, 2023
 *      Author: chalu
 */
#include "mpu9250.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_ACCEL_XOUT_L 0x3C
#define MPU9250_ACCEL_YOUT_H 0x3D
#define MPU9250_ACCEL_YOUT_L 0x3E
#define MPU9250_ACCEL_ZOUT_H 0x3F
#define MPU9250_ACCEL_ZOUT_L 0x40

#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_GYRO_XOUT_L 0x44
#define MPU9250_GYRO_YOUT_H 0x45
#define MPU9250_GYRO_YOUT_L 0x46
#define MPU9250_GYRO_ZOUT_H 0x47
#define MPU9250_GYRO_ZOUT_L 0x48

void MPU9250_CS_LOW() {
    HAL_GPIO_WritePin(MPU9250_CS_PORT, MPU9250_CS_PIN, GPIO_PIN_RESET);
}

void MPU9250_CS_HIGH() {
    HAL_GPIO_WritePin(MPU9250_CS_PORT, MPU9250_CS_PIN, GPIO_PIN_SET);
}

void MPU9250_SPI_Write(uint8_t reg, uint8_t data, SPI_HandleTypeDef *hspi) {
    MPU9250_CS_LOW();
    HAL_SPI_Transmit(hspi, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(hspi, &data, 1, HAL_MAX_DELAY);
    MPU9250_CS_HIGH();
}

void MPU9250_SPI_Read(uint8_t reg, uint8_t* data, uint16_t size, SPI_HandleTypeDef *hspi) {
    reg |= 0x80; // Set MSB to 1 for read operation
    MPU9250_CS_LOW();
    HAL_SPI_Transmit(hspi, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, data, size, HAL_MAX_DELAY);
    MPU9250_CS_HIGH();
}

void MPU9250_ReadAccel(int16_t* accelData, SPI_HandleTypeDef *hspi) {
    uint8_t buffer[6];

    // Lecture des données d'accélération brute
    MPU9250_SPI_Read(MPU9250_ACCEL_XOUT_H, buffer, 6, hspi);

    // Concaténation des octets pour former les données brutes
    accelData[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    accelData[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
    accelData[2] = (int16_t)((buffer[4] << 8) | buffer[5]);
}

void MPU9250_ReadGyro(int16_t* gyroData, SPI_HandleTypeDef *hspi) {
    uint8_t buffer[6];

    // Lecture des données d'accélération brute
    MPU9250_SPI_Read(MPU9250_GYRO_XOUT_H, buffer, 6, hspi);

    // Concaténation des octets pour former les données brutes
    gyroData[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    gyroData[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
    gyroData[2] = (int16_t)((buffer[4] << 8) | buffer[5]);
}

void MPU9250_Init(SPI_HandleTypeDef hspi){
	// Désactiver le module de gestion de l'alimentation pour permettre la configuration
	MPU9250_SPI_Write(MPU9250_REG_PWR_MGMT_1, 0x00, &hspi);

	// Configurer d'autres registres selon vos besoins
	// ...

	// Par exemple, configurer la fréquence d'échantillonnage
	MPU9250_SPI_Write(MPU9250_REG_SMPLRT_DIV, 0x07, &hspi); // 1kHz sample rate

	// Configurer le mode de mesure gyroscopique et accélérative
	MPU9250_SPI_Write(MPU9250_REG_CONFIG, 0x00, &hspi); // DLPF disabled, gyro 250Hz, accel 250Hz
	MPU9250_SPI_Write(MPU9250_REG_GYRO_CONFIG, 0x10, &hspi); // +/- 500dps
	MPU9250_SPI_Write(MPU9250_REG_ACCEL_CONFIG, 0x10, &hspi); // +/- 8g

	// Réactiver le module de gestion de l'alimentation
	MPU9250_SPI_Write(MPU9250_REG_PWR_MGMT_1, 0x01, &hspi);
}
