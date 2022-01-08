**InvenSense MPU6050** - is the world’s first and only 6-axis motion tracking devices designed for the low power, low cost, and high performance requirements of smartphones, tablets and wearable sensors.

## Library use:
1. Download [uMicroLibrary](https://github.com/stellar-creator/uMicroLibrary)
2. Download [uDriverLibrary](https://github.com/stellar-creator/uDriverLibrary) 
3. Include libraries to your project
4. Below is the basic initialization code
```c
  // Include module
  #include <uDriverLibrary/sensors/motion/uMpu6050.h>
  
  ...
  // Init MPU6050 struct
  uMpu6050Data uMpu6050data;
  
  ...
  
  // Set address
  uMpu6050data.address = 0x69;
  // Set I2C interface
  uMpu6050data.interface = &hi2c1;
  // Enable or disable calibration on start
  uMpu6050data.configuration.calibration = uEnabled;
  // Use averages values when counting
  uMpu6050data.configuration.extended.useAvg = uEnabled;
  // Number of average counts
  uMpu6050data.configuration.extended.avgValue = 50;
  // Convert raw values to physical (International System of Units)
  uMpu6050data.configuration.extended.usePhysicalValues = uDisabled;
  // Init MPU6050
  uMpu6050BaseInit(&uMpu6050data);
```
5. Run hanlder in loop (or somewhere)
```c
  uMpu6050Handler(&uMpu6050data);
```
6. Get values from structure
```c
  int someX = uMpu6050data.accelerometer.base.X;
  float someTemperature = uMpu6050data.temperature;
```