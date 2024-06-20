***Electronic for the main board of the fusex Atréides***

This repository contains the source code to upload on the stm32f411 from the Black Pill. 

The board is composed of a Blackpill, an SD card, a MPU9250-6500 (IMU/accelerometer) and a GY-BMP280 (Pressure/temperature sensor) and 2 UART output to communicate with the transceiver and the motor controller for the experience. 

Both sensors (MPU9250 and BMP280) are connected on SPI interface with the Blackpill. 

The code inits the board and then establish the communication with the sensors to register measurements on the SD card. 

Implemented libraries : 
- spi
- mpu9250
- bmp280 

Codes use the stm32f4xx_hal library. 
