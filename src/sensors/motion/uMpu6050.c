/*
Autor:			Stellar Creator
Date Of Update: 06.01.2022
File:			uMpu6050.c
*/

#include <math.h>
#include "uGlobal.h"
#include "uLogica.h"
#include "uMath.h"
#include "sensors/motion/uMpu6050.h"

// MPU6050 registers structure with values
const uMpu6050Registers uMpu6050registers = {
		{ // uMpu6050PowerManagmentRegisters
				0x6B, // PWR_MGMT_1
				{ // CLKSEL
						0, 	// Internal8Mhz
						1, 	// PLLX_GR
						2, 	// PLLY_GR
						3, 	// PLLZ_GR
						4,  // PLL32768kHz
						5,  // PLL19MHz2
						7	// ResetMpu6050
				}
		},
		{ // uMpu6050GyroscopeRegisters
				0x1B, // GYRO_CONFIG
				0,	// ± 250 °/s
				8,	// ± 500 °/s
				16,	// ± 1000 °/s
				24	// ± 2000 °/s
		},
		{ // uMpu6050AccelerometerRegisters
				0x1C, // ACCEL_CONFIG
				0,	// ± 2g
				8,	// ± 4g
				16,	// ± 8g
				24	// ± 16g
		},
		{ // _reg_temperature
				0x41,	// TEMP_OUT_H
				0x42	// TEMP_OUT_L
		},
		{ // _reg_gyroscope
				0x43,	// GYRO_XOUT_H
				0x44,	// GYRO_XOUT_L
				0x45,	// GYRO_YOUT_H
				0x46,	// GYRO_YOUT_L
				0x47,	// GYRO_ZOUT_H
				0x48,	// GYRO_ZOUT_L
		},
		{ // _reg_accelerometer
				0x3B,	// ACCEL_XOUT_H
				0x3C,	// ACCEL_XOUT_L
				0x3D,	// ACCEL_YOUT_H
				0x3E,	// ACCEL_YOUT_L
				0x3F,	// ACCEL_ZOUT_H
				0x40,	// ACCEL_ZOUT_L
		}
};

// MPU6050 main init function
uState uMpu6050Init(uMpu6050Data *data){
	// Set state
	data->state = uState_Init;
	// Write base registers to start
	uMpu6050WriteBuffer(data, (uData[2]){uMpu6050registers.powerManagmet._reg, data->configuration.powerManagment}, 2);
	uMpu6050WriteBuffer(data, (uData[2]){uMpu6050registers.accelerometer._reg, data->configuration.accelConfig}, 2);
	uMpu6050WriteBuffer(data, (uData[2]){uMpu6050registers.gyroscope._reg, data->configuration.gyroConfig}, 2);
	// Change state
	if(data->state != uState_Error){
		data->state = uState_Ready;
	}
	// Return state
	return data->state;
}

// MPU6050 base init function
uState uMpu6050BaseInit(uMpu6050Data *data){
	// Set default registers
	data->configuration.powerManagment = uMpu6050registers.powerManagmet.CLKSEL.Internal8Mhz;
	data->configuration.accelConfig = uMpu6050registers.accelerometer.G2;
	data->configuration.gyroConfig = uMpu6050registers.gyroscope.S250;
	// Init MPU6050
	if(uMpu6050Init(data) != uState_Ready){
		// If problems, return error
		data->state = uState_Error;
		return data->state;
	}
	// Set init state
	data->state = uState_Init;
	// If need calibration, calc avg (simple)
	if(data->configuration.calibration == uEnabled){
		// Accel x, y, z
		double x, y, z;
		// 25 times count data
		for(int i = 0; i < 25; i++){
			// Get data from accel
			uMpu6050GetAccelerometer(data);
			// Count axis
			x += data->accelerometer.base.X;
			y += data->accelerometer.base.Y;
			z += data->accelerometer.base.Z;
			// Some delay
			HAL_Delay(5);
		}
		// Div calibration values
		data->accelerometer.calibration.X = (uCoordinate16) (x / 25);
		data->accelerometer.calibration.Y = (uCoordinate16) (y / 25);
		data->accelerometer.calibration.Z = (uCoordinate16) (z / 25);
	}
	// Set state ready
	data->state = uState_Ready;
	// Return state
	return data->state;
}

// Get temperature from MPU6050
uInteger16 uMpu6050GetTemperature(uMpu6050Data *data){
	// Temperature buffer
	uData temperatureBuffer[2];
	// Read data from MPU6050
	uMpu6050ReadBuffer(data, uMpu6050registers._reg_temperature[0], temperatureBuffer, 2);
	// Convert 2 uint8_t to 1 uint16_t
	data->temperature = ((int16_t) temperatureBuffer[0] << 8) | temperatureBuffer[1];
	// Make calculations according to the data sheet
	data->temperature = ( data->temperature / 340 ) + 36.53;
	// Return value
	return data->temperature;
}

// Get MPU6050 gyroscope data
uMpu6050Gyroscope uMpu6050GetGyroscope(uMpu6050Data *data){
	// Gyroscope buffer
	uData gyroscopeBuffer[6];
	// Read data
	uMpu6050ReadBuffer(data, uMpu6050registers._reg_gyroscope[0], gyroscopeBuffer, 6);
	// Check if user count avg values
	if(data->configuration.extended.useAvg != uEnabled){
		// Read data directly into the buffer and convert 2 uint8_t to 1 uint16_t
		data->gyroscope.base.X = ( ((int16_t) gyroscopeBuffer[0] << 8) | gyroscopeBuffer[1] );
		data->gyroscope.base.Y = ( ((int16_t) gyroscopeBuffer[2] << 8) | gyroscopeBuffer[3] );
		data->gyroscope.base.Z = ( ((int16_t) gyroscopeBuffer[4] << 8) | gyroscopeBuffer[5] );
	}else{
		// Read the data directly into the average buffer. When the values ​​are calculated, they will be transferred to the base buffer.
		if(data->gyroscope.avgCounter <= data->configuration.extended.avgValue){
			// Read data directly into the buffer and convert 2 uint8_t to 1 uint16_t
			data->gyroscope.avg.X += ((int16_t) gyroscopeBuffer[0] << 8) | gyroscopeBuffer[1];
			data->gyroscope.avg.Y += ((int16_t) gyroscopeBuffer[2] << 8) | gyroscopeBuffer[3];
			data->gyroscope.avg.Z += ((int16_t) gyroscopeBuffer[4] << 8) | gyroscopeBuffer[5];
			// Inc counter
			data->gyroscope.avgCounter++;
		}else{
			// Transmitt from avg buffer to base
			data->gyroscope.base.X = (int16_t) (data->gyroscope.avg.X / data->gyroscope.avgCounter);
			data->gyroscope.base.Y = (int16_t) (data->gyroscope.avg.Y / data->gyroscope.avgCounter);
			data->gyroscope.base.Z = (int16_t) (data->gyroscope.avg.Z / data->gyroscope.avgCounter);
			// Reset counter
			data->gyroscope.avgCounter = 0;
			data->gyroscope.avg.X = 0;
			data->gyroscope.avg.Y = 0;
			data->gyroscope.avg.Z = 0;
		}
	}
	// Return data
	return data->gyroscope;
}

// Get MPU6050 accelerometer data
uMpu6050Accelerometer uMpu6050GetAccelerometer(uMpu6050Data *data){
	// Accelerometer buffer
	uData accelerometerBuffer[6];
	// Accelerometer cords
	float x, y, z;
	// Read data
	uMpu6050ReadBuffer(data, uMpu6050registers._reg_accelerometer[0], accelerometerBuffer, 6);
	// Check if use avg calculation and if state init. This protection is needed during the initialization process to calculate the avg offset values.
	if(data->configuration.extended.useAvg != uEnabled || data->state == uState_Init){
		// Read data directly into the buffer and convert 2 uint8_t to 1 uint16_t
		x = ( ((int16_t) accelerometerBuffer[0] << 8) | accelerometerBuffer[1] );
		y = ( ((int16_t) accelerometerBuffer[2] << 8) | accelerometerBuffer[3] );
		z = ( ((int16_t) accelerometerBuffer[4] << 8) | accelerometerBuffer[5] );
		// If calibration is enabled and state 'in process', set offset
		if(data->configuration.calibration == uEnabled && data->state == uState_InProcess){
			x = data->accelerometer.calibration.X - x;
			y = data->accelerometer.calibration.Y - y;
			z = data->accelerometer.calibration.Z - z;
		}
		// Set values int structure
		data->accelerometer.base.X = (uCoordinate16) x;
		data->accelerometer.base.Y = (uCoordinate16) y;
		data->accelerometer.base.Z = (uCoordinate16) z;
	}else{
		// Read the data directly into the average buffer. When the values ​​are calculated, they will be transferred to the base buffer.
		if(data->accelerometer.avgCounter <= data->configuration.extended.avgValue){
			x = ( ((int16_t) accelerometerBuffer[0] << 8) | accelerometerBuffer[1] );
			y = ( ((int16_t) accelerometerBuffer[2] << 8) | accelerometerBuffer[3] );
			z = ( ((int16_t) accelerometerBuffer[4] << 8) | accelerometerBuffer[5] );
			// If calibration is enabled and state 'in process', set offset
			if(data->configuration.calibration == uEnabled && data->state == uState_InProcess){
				x = data->accelerometer.calibration.X - x;
				y = data->accelerometer.calibration.Y - y;
				z = data->accelerometer.calibration.Z - z;
			}
			// Add values to avg buffer
			data->accelerometer.avg.X += x;
			data->accelerometer.avg.Y += y;
			data->accelerometer.avg.Z += z;
			// Inc counter
			data->accelerometer.avgCounter++;
		}else{
			// Transmitt from avg buffer to base
			data->accelerometer.base.X = (uCoordinate16) (data->accelerometer.avg.X / data->accelerometer.avgCounter);
			data->accelerometer.base.Y = (uCoordinate16) (data->accelerometer.avg.Y / data->accelerometer.avgCounter);
			data->accelerometer.base.Z = (uCoordinate16) (data->accelerometer.avg.Z / data->accelerometer.avgCounter);
			// Reset counter
			data->accelerometer.avgCounter = 0;
			data->accelerometer.avg.X = 0;
			data->accelerometer.avg.Y = 0;
			data->accelerometer.avg.Z = 0;
		}
	}
	// Return data
	return data->accelerometer;
}

uState uMpu6050Handler(uMpu6050Data *data){
	// If state not error handler in process, else return
	if(data->state != uState_Error){
		data->state = uState_InProcess;
	}else{
		return data->state;
	}
	// Reading temperature
	uMpu6050GetTemperature(data);
	// Reading gyroscope
	uMpu6050GetGyroscope(data);
	// Reading accelerometer
	uMpu6050GetAccelerometer(data);
	// Return state
	return data->state;
}

// I2C write function
uFunction uMpu6050WriteBuffer(uMpu6050Data *data, uData *buffer, uSize size) {
    while(HAL_I2C_Master_Transmit(data->interface, (uint16_t) data->address << 1, (uint8_t*) buffer, (uint16_t) size, (uint32_t) 1000) != HAL_OK){
        if (HAL_I2C_GetError(data->interface) != HAL_I2C_ERROR_AF){
        	// Set state if error
        	data->state = uState_Error;
        	Error_Handler();
        }
    }
    while (HAL_I2C_GetState(data->interface) != HAL_I2C_STATE_READY){}
}

// I2C read function
uFunction uMpu6050ReadBuffer(uMpu6050Data *data, uRegister address, uData *buffer, uSize size){
	uMpu6050WriteBuffer(data, (uData[1]){ (uData) address}, (uSize) 1);
    while(HAL_I2C_Master_Receive(data->interface, (uint16_t) data->address << 1, buffer, (uint16_t) size, (uint32_t) 1000) != HAL_OK){
        if (HAL_I2C_GetError(data->interface) != HAL_I2C_ERROR_AF){
        	// Set state if error
        	data->state = uState_Error;
        	Error_Handler();
        }
    }
    while (HAL_I2C_GetState(data->interface) != HAL_I2C_STATE_READY){}
}
