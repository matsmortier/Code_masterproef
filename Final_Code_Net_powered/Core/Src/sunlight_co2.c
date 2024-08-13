/*
 * sunlight_co2.c
 *
 *  Created on: Feb 27, 2024
 *      Author: matsmortier
 */

#include <sunlight_co2_new.h>
#include "main.h"
#include "stm32wbaxx_hal.h"
#include "stdio.h"


extern I2C_HandleTypeDef hi2c1;


extern int readPeriodMs;

extern int co2_done;

extern uint16_t carbon;




/*
 * definition: wakeUpSensor
 *
 * TODO: explain function...
 *
 *
 */
void wakeUpSensor(uint8_t deviceAddress) {

	HAL_I2C_IsDeviceReady(&hi2c1, deviceAddress << 1, 5, HAL_MAX_DELAY);

}



/*
 * definition: read_sensor_config
 *
 * In this definition, we are reading the sensor configurations. Those are: 1. Meter control, 2. Measurement mode,
 * 3. Measurement period, 4. Number of samples and 5. ABC - period.
 *
 */

void read_Sensor_Config(uint8_t data[]) {
	  uint8_t meter_control[0];

	  wakeUpSensor(0x68);

	  // Read measurement mode, measurement period, number of samples, ABC period
	  HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x95,I2C_MEMADD_SIZE_8BIT , data, 7, HAL_MAX_DELAY);
	  //readRegister(0x68, data, 1, data, 7);

	  // Read meter control
	  HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0xA5 ,I2C_MEMADD_SIZE_8BIT , meter_control, 1, HAL_MAX_DELAY);
	  //readRegister(0x68, meter_control, 1, meter_control, 1);

	  // Measurement mode
	  uint8_t measMode = data[0];

	  // Measurement period
	  uint8_t byteHi = data[1];
	  uint8_t byteLo = data[2];
	  uint16_t measPeriod = ((int16_t)(int8_t) byteHi << 8) | (uint16_t)byteLo;
	  readPeriodMs = measPeriod * 1000;

	  //Number of samples
	  byteHi = data[3];
	  byteLo = data[4];
	  uint16_t numSamples = ((int16_t)(int8_t) byteHi << 8) | (uint16_t)byteLo;

	  //ABCPeriod
	  byteHi = data[5];
	  byteLo = data[6];
	  uint16_t abcPeriod = ((int16_t)(int8_t) byteHi << 8) | (uint16_t)byteLo;


	  // Push data over Serial wire
	  LOG_INFO_APP("Measurement Mode: %d\r\n",measMode);
	  LOG_INFO_APP("Measurement Period: %d\r\n",measPeriod);
	  LOG_INFO_APP("Number of samples: %d\r\n",numSamples);
	  LOG_INFO_APP("ABC Periode: %d\r\n",abcPeriod);
	  LOG_INFO_APP("Read period in ms: %d\r\n",readPeriodMs);

	  // Push data over Jlink
	  SEGGER_RTT_printf(0, "Measurement Mode: %u\n:",measMode);
	  SEGGER_RTT_printf(0, "Measurement Period: %u \n", measPeriod);
	  SEGGER_RTT_printf(0, "Number of samples: %u\n", numSamples);
	  SEGGER_RTT_printf(0, "ABC Periode: %u\n", abcPeriod);
	  SEGGER_RTT_printf(0,"Read period in ms:%u \n", readPeriodMs);
}


/*
 * definition: save_State
 *
 * in this definition we are powering the sensor, then we call the read_ABC_Value-defintion.
 * After this, we power down the sensor
 *
 */
void save_State(uint8_t state[]){
	// Enable sensor
	HAL_GPIO_WritePin(SENSOR_EN_GPIO_Port, SENSOR_EN_Pin, GPIO_PIN_SET);
	HAL_Delay(35);

	read_ABC_Value(state);

	// Disable sensor
	HAL_GPIO_WritePin(SENSOR_EN_GPIO_Port, SENSOR_EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(35);

}



/*
 * definition: read_ABC_Value
 *
 * In this definition, we are reading the ABC values in register. These values are stored in an array named 'state'
 *
 *
 */
void read_ABC_Value(uint8_t state[]) {
	int numBytes = 24;

	wakeUpSensor(0x68);

	// Read state registers
	HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0xC4 ,I2C_MEMADD_SIZE_8BIT , state, numBytes, HAL_MAX_DELAY);
	//readRegister(0x68, state, 1, state, numBytes);

}



/*
 * definition: read_Sensor_Measurement
 *
 * First of all, we write the cmd-array into the registers. Note that we write 0x01 to the first register so we can start a single measurement
 * After 2.4s, we can read the values in registers (0x01-0x07). The last 2 bytes combined is the CO2-value. After this, we read the ABC value again.
 *
 */
void read_Sensor_Measurements(uint8_t target,uint8_t state[]){
	int numRegCmd = 25;
	uint8_t cmdArray[25];
	cmdArray[0] = 0x01;

	for(int n = 1 ; n < numRegCmd ; n++) {
	  cmdArray[n] = state[n-1];
	}

	// Enable sensor
	HAL_GPIO_WritePin(SENSOR_EN_GPIO_Port, SENSOR_EN_Pin, GPIO_PIN_SET);

	// Wait for sensor start-up and stabilization
	HAL_Delay(25);

	wakeUpSensor(target);

	// Write cmd-array in registers
	HAL_I2C_Mem_Write(&hi2c1, (target<<1),0xC3,1,cmdArray,strlen(cmdArray),HAL_MAX_DELAY);

	// according to datasheet we need to wait 2.4s for sensor to be ready
	HAL_Delay(2400);

	wakeUpSensor(target);

	uint8_t data[7];

	// Read registers
	HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x01 ,I2C_MEMADD_SIZE_8BIT , data, 7, HAL_MAX_DELAY);
	//readRegister(0x68, data, 1, data, 7);

    // combine 2 bytes to obtain CO2 value
    uint8_t byteHi = data[5];
    uint8_t byteLo = data[6];
    carbon = ((int16_t)(int8_t) byteHi << 8) | (uint16_t)byteLo;


    // Output to serial viewer
    LOG_INFO_APP("co2-value: %u \n",carbon);
    SEGGER_RTT_printf(0, "co2-value: %u \n",carbon);

    // Readd ABC-value
    read_ABC_Value(state);

    // Disable sensor
    HAL_GPIO_WritePin(SENSOR_EN_GPIO_Port, SENSOR_EN_Pin, GPIO_PIN_RESET);

}


/*
 * definition: background_calibration
 *
 * First of all, we need to reset the last calibration status (send 0x00) and write 0x7C06 to
 * 0x82 to start a background calibration. After this, we need to make a single measurement and store the state-data into
 * the belong registers. In the last step, we read the calibration status. if this is equal to 0x02, then the background
 * calibration is completed!
 *
 *
 *In the real application we need to make sure that the sensor is in a stable air environment!
 */

void background_Calibration(uint8_t target,uint8_t state[]){
	/* Function variables */
	  HAL_StatusTypeDef error;
	  uint8_t clear_Register = 0x00;
	  int numRegCmd = 25;

	  char UART_buffer[50];

	  uint8_t start_calibration[1];
	  start_calibration[0] = 0x7C;
	  start_calibration[1] = 0x06;
	  uint8_t error_data[1];
	  uint8_t calibration_status[1];
	  uint8_t cmdArray[25];

	  // Drive EN pin high
	  HAL_GPIO_WritePin(SENSOR_EN_GPIO_Port, SENSOR_EN_Pin, GPIO_PIN_SET);
	  HAL_Delay(50);

	  wakeUpSensor(target);


	  //clear calibration status register
	  error = HAL_I2C_Mem_Write(&hi2c1, (0x68 << 1 ), 0x81, I2C_MEMADD_SIZE_8BIT, &clear_Register, 1, HAL_MAX_DELAY);


	  if (error == HAL_OK) {

		wakeUpSensor(target);

		// Start calibration
		error = HAL_I2C_Mem_Write(&hi2c1, (target<<1) , 0x82 , I2C_MEMADD_SIZE_8BIT , start_calibration , 2 ,HAL_MAX_DELAY);

	    if (error == HAL_OK) {

	    		cmdArray[0] = 0x01;

	    		for(int n = 1 ; n < numRegCmd ; n++) {
	    		  cmdArray[n] = state[n-1];
	    		}

	    		wakeUpSensor(target);


	    		HAL_I2C_Mem_Write(&hi2c1, (target<<1) ,0xC3, I2C_MEMADD_SIZE_8BIT , cmdArray , strlen(cmdArray) ,HAL_MAX_DELAY);

	    		// Wait 2.4 s for sampling
	    		HAL_Delay(2400);

	      int attempts = 0;
	      do {

	        // The calibration should be finished after next measurement period,
	    	// so to simplicity the process - we just waiting for one (or two if we have a synchronization issue) measurement periods...
	    	//
	    	// from Arduino code!


	        wakeUpSensor(target);



	        // Check error status, possible that the calibration failed because there was no stable environment
	        error = HAL_I2C_Mem_Read(&hi2c1, (target<<1), 0x01, I2C_MEMADD_SIZE_8BIT, error_data, 1,HAL_MAX_DELAY);



	        if(error ==  HAL_OK) {
	            // Error status
	            uint8_t eStatus = error_data[0];


	          	wakeUpSensor(target);

	            // Check cabiation status
	            if(((error = HAL_I2C_Mem_Read(&hi2c1, (target<<1), 0x81, I2C_MEMADD_SIZE_8BIT, calibration_status, 1, HAL_MAX_DELAY))) == HAL_OK) {


	              uint8_t statusCalibration = calibration_status[0];

	              //Check if the calibration completed
	              if(statusCalibration & 0x20) {
	                break;
	              }

	              else if(++attempts == 3) {
	                  // Timeout while waiting for calibration
	            	  break;

	              }
	            }

	        }
	      } while(error != HAL_OK);
	    }
	  }


	  read_ABC_Value(state);

	  // Disable sensor
	  HAL_GPIO_WritePin(SENSOR_EN_GPIO_Port, SENSOR_EN_Pin, GPIO_PIN_RESET);

	  // Output to serial viewer
	  if (error == HAL_OK){
		  LOG_INFO_APP("calibration done!\r\n");

	  }
	  else{
		  LOG_INFO_APP("calibration not completed!\r\n");
	  }
}


/*
 * definition: change_Measurement_Mode
 *
 * First of all we need to read the data in register 0x95, after this we check if this is equal to 0 or 1
 * (0 = continuous mode, 1 = single mode). If it is equal to 0, then we need to change the mode. After changing the mode,
 * we need to restart the sensor.
 *
 */

void change_Measurement_Mode(uint8_t target,uint8_t data[]){
	  HAL_StatusTypeDef error;
	  uint8_t change_meas_per[1];
	  change_meas_per[0] = 0x01;




	  wakeUpSensor(target);


	  uint8_t measMode = data[0];


	  // Change mode if continuous (0)
	  if(measMode != 1) {


	    wakeUpSensor(target);

	    // Write 1 to measurement mode register
	    error = HAL_I2C_Mem_Write(&hi2c1, (target<<1),0x95,I2C_MEMADD_SIZE_8BIT,change_meas_per,1,HAL_MAX_DELAY);

	    HAL_Delay(50);

	    if(error != HAL_OK) {
	    	// Write error
	    }
	    else{
	    	// Mode changed
	    }

	  }
	  else{
		wakeUpSensor(target);

		// Write 0 to measurement mode register
		error = HAL_I2C_Mem_Write(&hi2c1, (target<<1),0x95,I2C_MEMADD_SIZE_8BIT,change_meas_per,0,HAL_MAX_DELAY);

		HAL_Delay(50);

		if(error != HAL_OK) {
			// Write error
		}
		else{
			// Mode changed
		}
	  }
	  // Disable and enable sensor to confirm changes
      HAL_GPIO_WritePin(SENSOR_EN_GPIO_Port, SENSOR_EN_Pin, GPIO_PIN_RESET);
	  HAL_Delay(50);
	  /* Turn-on sensor */
	  HAL_GPIO_WritePin(SENSOR_EN_GPIO_Port, SENSOR_EN_Pin, GPIO_PIN_SET);
	}


void disable_ABC(){
	uint8_t addres_time[1];
	addres_time[0] = 0x00u;
	addres_time[1] = 0x00u;

	 //disable ABC period
	 HAL_I2C_Mem_Write(&hi2c1, (0x68<<1),0x9A ,I2C_MEMADD_SIZE_8BIT , (uint8_t*)&addres_time, 2, HAL_MAX_DELAY);

}
