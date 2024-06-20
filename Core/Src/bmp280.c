#include "main.h"
#include "bmp280.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

void BMP280_CS_LOW(){
	HAL_GPIO_WritePin(BMP280_CS_PORT, BMP280_CS_PIN, GPIO_PIN_RESET);
}
void BMP280_CS_HIGH(){
	HAL_GPIO_WritePin(BMP280_CS_PORT, BMP280_CS_PIN, GPIO_PIN_SET);
}

uint8_t bmp280_read_register(BMP280 *bmp, uint8_t reg) {
	uint8_t tx = reg | BMP280_SPI_READ;
	uint8_t rx;

    BMP280_CS_LOW();
    HAL_SPI_Transmit(bmp->hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(bmp->hspi, &rx, 1, HAL_MAX_DELAY);
    BMP280_CS_HIGH();

    return rx;
}

void bmp280_read_registers(BMP280 *bmp, uint8_t reg, uint8_t *data, uint8_t length) {

	uint8_t tx = reg | BMP280_SPI_READ;

    BMP280_CS_LOW();
    HAL_SPI_Transmit(bmp->hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(bmp->hspi, data, length, HAL_MAX_DELAY);
    BMP280_CS_HIGH();

}

void bmp280_read_calibration_data(BMP280 *bmp) {
    uint8_t calib[24];
    bmp280_read_registers(bmp, BMP280_DIG_T1_LSB, calib, 24);

    bmp->calib.dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    bmp->calib.dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
    bmp->calib.dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);
    bmp->calib.dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    bmp->calib.dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
    bmp->calib.dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
    bmp->calib.dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
    bmp->calib.dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
    bmp->calib.dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
    bmp->calib.dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
    bmp->calib.dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
    bmp->calib.dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);
}

void bmp280_init(BMP280 *bmp, SPI_HandleTypeDef *hspi) {
	bmp->hspi = hspi;

    // Reset the sensor
    uint8_t reset_cmd = BMP280_RESET_VALUE;
    BMP280_CS_LOW();
    HAL_SPI_Transmit(bmp->hspi, &reset_cmd, 1, HAL_MAX_DELAY);
    BMP280_CS_HIGH();

    // Wait for the reset to complete
    uint8_t status = 0;
    while (status & 0x01) {
        status = bmp280_read_register(bmp, BMP280_STATUS);
    }

    // Read calibration data
    bmp280_read_calibration_data(bmp);

    // Set the control measurement register to start measurements
    uint8_t ctrl_meas = 0x27; // Normal mode, temp and pressure oversampling x1
    BMP280_CS_LOW();
    HAL_SPI_Transmit(bmp->hspi, &ctrl_meas, 1, HAL_MAX_DELAY);
    BMP280_CS_HIGH();
}

int32_t bmp280_compensate_T_int32(BMP280 *bmp, int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)bmp->calib.dig_T1 << 1))) * ((int32_t)bmp->calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bmp->calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp->calib.dig_T1))) >> 12) * ((int32_t)bmp->calib.dig_T3)) >> 14;
    bmp->calib.t_fine = var1 + var2;
    T = (bmp->calib.t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t bmp280_compensate_P_int64(BMP280 *bmp, int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp->calib.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp->calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp->calib.dig_P3) >> 8) + ((var1 * (int64_t)bmp->calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp->calib.dig_P1) >> 33;
    if (var1 == 0) {
        return 0; // Avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp->calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp->calib.dig_P7) << 4);
    return (uint32_t)p;
}

void bmp280_read_temperature_and_pressure(BMP280 *bmp, int32_t *temperature, uint32_t *pressure) {
    uint8_t data[6];
    bmp280_read_registers(bmp, BMP280_PRESS_MSB, data, 6);

    int32_t adc_P = (int32_t)(((uint32_t)(data[0]) << 12) | ((uint32_t)(data[1]) << 4) | ((uint32_t)data[2] >> 4));
    int32_t adc_T = (int32_t)(((uint32_t)(data[3]) << 12) | ((uint32_t)(data[4]) << 4) | ((uint32_t)data[5] >> 4));

    *temperature = bmp280_compensate_T_int32(bmp, adc_T);
    *pressure = bmp280_compensate_P_int64(bmp, adc_P);
}

