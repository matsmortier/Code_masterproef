/*
 * sht4x.c
 *
 *  Created on: Feb 27, 2024
 *      Author: matsmortier
 */
#include "main.h"
#include "sht4x.h"
#include "stm32wbaxx_hal.h"
#include "stdio.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

extern uint16_t humidity;
extern uint16_t temperature;
float temperature_temp;
float humidity_temp;


#define SHT4X_ADDR 0x44



/*
 * definition: temperature_humidity_measurement
 *
 * in this definition we are sending 0xE0 to the SHT4x-sensor. After this, we are reading 6 bytes of the sensor. The first 2 bytes are for the temperature. The third and fourth
 * bytes are the humidity .
 *
 * 0xFD = measure temperature & relative humidity with high precision (high repeatability) [2 * 8-bit T-data; 8-bit CRC; 2 * 8-bit RH-data; 8-bit CRC]
 * 0xF6 = measure temperature & relative humidity with medium precision (medium repeatability) [2 * 8-bit T-data; 8-bit CRC; 2 * 8-bit RH-data; 8-bit CRC]
 * 0xE0 = measure temperature & relative humidity with lowest precision (low repeatability) [2 * 8-bit T-data; 8-bit CRC; 2 * 8-bit RH-data; 8-bit CRC]
 *
 *
 * Temperature conversion: -45 + 175*(St/2^16 - 1)
 *
 * Relative humidity conversion: -6 + 125*(Srh/2^16 - 1)
 *
 * According to datasheet SHT4X : https://sensirion.com/media/documents/33FD6951/662A593A/HT_DS_Datasheet_SHT4x.pdf
 *
 */

void temperature_Humidity_Measurement(){

	  // Initialize measurement with lowest precision
	  HAL_I2C_Master_Transmit(&hi2c1, (SHT4X_ADDR << 1), (uint8_t[]) {0xE0}, 1, HAL_MAX_DELAY);

	  //Wait for sensor to process command
	  HAL_Delay(10);

	  // Read data from sensor
	  uint8_t data_sensor[6];
	  HAL_I2C_Master_Receive(&hi2c1, (SHT4X_ADDR << 1), data_sensor, 6, HAL_MAX_DELAY);

	  // Process received data
	  if (data_sensor[0] != 0) {
		uint16_t t_ticks, rh_ticks;
		//uint8_t checksum_t, checksum_rh;
		t_ticks = (data_sensor[0] << 8) | data_sensor[1];  // Temperature data
		//checksum_t = sensor_data[2];                     // Temperature checksum (unnecessary)

		rh_ticks = (data_sensor[3] << 8) | data_sensor[4]; // Humidity data
		//checksum_rh = sensor_data[5];                    // Humidity checksum (unnecessary)

		//convert data according to formulas in datasheet
		temperature_temp = -45 + 175.0 * t_ticks / 65535;
		humidity_temp = -6 + 125.0 * rh_ticks / 65535;

		// Ensure humidity value is within valid range
		if (humidity_temp > 100) {
		  humidity_temp = 100;

		}
		if (humidity_temp < 0) {
		  humidity_temp = 0;
		}
	  }
	  //temporary value * 100 to make it an uint16_t to store in adv_value
	  temperature = temperature_temp * 100;
	  humidity = humidity_temp * 100;
}
