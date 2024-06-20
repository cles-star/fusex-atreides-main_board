#include "main.h"
#include "spi.h"  // This header should include your SPI function prototypes
#include "gpio.h"

#define BMP280_CS_PIN BMP280_CS_Pin
#define BMP280_CS_PORT BMP280_CS_GPIO_Port

#define BMP280_CHIP_ID       0xD0
#define BMP280_RESET         0xE0
#define BMP280_STATUS        0xF3
#define BMP280_CTRL_MEAS     0xF4
#define BMP280_CONFIG        0xF5
#define BMP280_PRESS_MSB     0xF7
#define BMP280_PRESS_LSB     0xF8
#define BMP280_PRESS_XLSB    0xF9
#define BMP280_TEMP_MSB      0xFA
#define BMP280_TEMP_LSB      0xFB
#define BMP280_TEMP_XLSB     0xFC

#define BMP280_SPI_READ      0x80

#define BMP280_CHIP_ID_VALUE 0x58
#define BMP280_RESET_VALUE   0xB6

// Calibration parameter registers
#define BMP280_DIG_T1_LSB    0x88
#define BMP280_DIG_T1_MSB    0x89
#define BMP280_DIG_T2_LSB    0x8A
#define BMP280_DIG_T2_MSB    0x8B
#define BMP280_DIG_T3_LSB    0x8C
#define BMP280_DIG_T3_MSB    0x8D
#define BMP280_DIG_P1_LSB    0x8E
#define BMP280_DIG_P1_MSB    0x8F
#define BMP280_DIG_P2_LSB    0x90
#define BMP280_DIG_P2_MSB    0x91
#define BMP280_DIG_P3_LSB    0x92
#define BMP280_DIG_P3_MSB    0x93
#define BMP280_DIG_P4_LSB    0x94
#define BMP280_DIG_P4_MSB    0x95
#define BMP280_DIG_P5_LSB    0x96
#define BMP280_DIG_P5_MSB    0x97
#define BMP280_DIG_P6_LSB    0x98
#define BMP280_DIG_P6_MSB    0x99
#define BMP280_DIG_P7_LSB    0x9A
#define BMP280_DIG_P7_MSB    0x9B
#define BMP280_DIG_P8_LSB    0x9C
#define BMP280_DIG_P8_MSB    0x9D
#define BMP280_DIG_P9_LSB    0x9E
#define BMP280_DIG_P9_MSB    0x9F

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    int32_t t_fine;
} BMP280_CalibParams;

typedef struct {
	SPI_HandleTypeDef *hspi;
    BMP280_CalibParams calib;
} BMP280;

void BMP280_CS_LOW();
void BMP280_CS_HIGH();
uint8_t bmp280_read_register(BMP280 *bmp, uint8_t reg);
void bmp280_read_registers(BMP280 *bmp, uint8_t reg, uint8_t *data, uint8_t length);
void bmp280_read_calibration_data(BMP280 *bmp);
void bmp280_init(BMP280 *bmp, SPI_HandleTypeDef *hspi);
int32_t bmp280_compensate_T_int32(BMP280 *bmp, int32_t adc_T);
uint32_t bmp280_compensate_P_int64(BMP280 *bmp, int32_t adc_P);
void bmp280_read_temperature_and_pressure(BMP280 *bmp, int32_t *temperature, uint32_t *pressure);

