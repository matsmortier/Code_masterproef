/*
 * sunlight_co2.h
 *
 *  Created on: Feb 27, 2024
 *      Author: matsmortier
 */

#ifndef INC_SUNLIGHT_CO2_NEW_H_
#define INC_SUNLIGHT_CO2_NEW_H_

#include <stdint.h>
#include "stm32wbaxx_hal.h"

void wakeUpSensor(uint8_t deviceAddress);


void read_Sensor_Config(uint8_t data[]);


void save_State(uint8_t state[]);


void read_ABC_Value(uint8_t state[]);


void read_Sensor_Measurements(uint8_t target,uint8_t state[]);


void background_Calibration(uint8_t target,uint8_t state[]);


int reset_Calibration(uint8_t target);


void change_Measurement_Mode(uint8_t target,uint8_t data[]);


void disable_ABC();


#endif /* INC_SUNLIGHT_CO2_NEW_H_ */
