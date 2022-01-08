/*
Autor:			Stellar Creator
Date Of Update: 06.01.2022
File:			uMpu6050.h
*/

#ifndef UMPU6050_H
#define UMPU6050_H

#include <stdint.h>
#include "i2c.h"
#include "uGlobal.h"
#include "uLogica.h"
#include "uMath.h"
#include "uHAL.h"

// Power management CLKSEL values
typedef struct __attribute__ ((packed)){
	uData Internal8Mhz;
	uData PLLX_GR;
	uData PLLY_GR;
	uData PLLZ_GR;
	uData PLLEX32768kHz;
	uData PLLEX19MHz2;
	uData Stop;
} uMpu6050PowerManagementRegisters_CLKSEL;

// Power management values
typedef struct __attribute__ ((packed)){
	uData	_reg;
	uMpu6050PowerManagementRegisters_CLKSEL	CLKSEL;
} uMpu6050PowerManagementRegisters;

// Accelerometer register values
typedef struct __attribute__ ((packed)){
	uData	_reg;
	uData	G2;
	uData	G4;
	uData	G8;
	uData 	G16;
} uMpu6050AccelerometerRegisters;

// Gyroscope register values
typedef struct __attribute__ ((packed)){
	uData	_reg;
	uData	S250;
	uData	S500;
	uData	S1000;
	uData 	S2000;
} uMpu6050GyroscopeRegisters;

// Extended configuration structure
typedef struct __attribute__ ((packed)){
	uState		useAvg;
	uInteger	avgValue;
	uState		usePhysicalValues;
} uMpu6050ConfigurationExtended;

// MPU6050 configuration structure
typedef struct __attribute__ ((packed)) {
	uRegister	powerManagment;
	uRegister	gyroConfig;
	uRegister	accelConfig;
	uState		calibration;
	uMpu6050ConfigurationExtended extended;
} uMpu6050Configuration;

// MPU6050 accelerometer structure
typedef struct __attribute__ ((packed)) {
	uCoordinate163D	base;
	uCoordinate163D	calibration;
	uCoordinate323D	avg;
	uInteger		avgCounter;
} uMpu6050Accelerometer;

// MPU6050 gyroscope structure
typedef struct __attribute__ ((packed)) {
	uCoordinate163D	base;
	uCoordinate163D	calibration;
	uCoordinate323D	avg;
	uInteger		avgCounter;
} uMpu6050Gyroscope;

// Main MPU6050 structure
typedef struct __attribute__ ((packed)) {
	uMpu6050PowerManagementRegisters powerManagmet;
	uMpu6050GyroscopeRegisters	gyroscope;
	uMpu6050AccelerometerRegisters	accelerometer;
	uRegister	_reg_temperature[2];
	uRegister	_reg_gyroscope[6];
	uRegister	_reg_accelerometer[6];
} uMpu6050Registers;


// Main MPU6050 structure
typedef struct __attribute__ ((packed)) {
	uI2cInterface *interface;
	uAddress address;
	uMpu6050Configuration configuration;
	uState state;
	uData rawData[64];
	uMpu6050Accelerometer accelerometer;
	uMpu6050Gyroscope gyroscope;
	uInteger16 temperature;
	uInteger16 roll;
	uInteger16 pitch;
	uInteger16 yaw;
} uMpu6050Data;

// Functions
uState uMpu6050Init(uMpu6050Data *data);
uState uMpu6050BaseInit(uMpu6050Data *data);
uFunction uMpu6050WriteBuffer(uMpu6050Data *data, uData *buffer, uSize size);
uFunction uMpu6050ReadBuffer(uMpu6050Data *data, uRegister address, uData *buffer, uSize size);
uState uMpu6050Handler(uMpu6050Data *data);
uInteger16 uMpu6050GetTemperature(uMpu6050Data *data);
uMpu6050Gyroscope uMpu6050GetGyroscope(uMpu6050Data *data);
uMpu6050Accelerometer uMpu6050GetAccelerometer(uMpu6050Data *data);

#endif /* UMPU6050_H */
