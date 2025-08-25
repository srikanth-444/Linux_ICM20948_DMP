Linux ICM20948 DMP library
======================
This repository contains a modified version of the ICM20948 library with Circular Buffer implementation. It also includes an example project for running the ICM-20948 with raspberry pi or other linux distros that support userspace spi setup.

Thanks to the contributor of the original library https://github.com/ZaneL/Teensy-ICM-20948

## Functionality

* Communication over SPI 
* Original DMP firmware with 9DOF sensor fusion
* Easy to set DMP settings
* Provides Accel, Gyro, Mag as Calib, Uncalib and Raw 
* Provides Linear Accel, Gravity Vector, 6&9DoF Quaternion and Euler, Activity Recognition and Step Counter
* Provides a circular buffer to maximize sample retention and minimize data loss during high-frequency sampling
* The sensor is implemented as an Object by using the PIICM20948 class (Check example)

## Setup

Initialize the icm20948 Object, Circular Buffer and inv_sensor_event_t object
```
PIICM20948 icm20948;
CircularBuffer cb;
inv_sensor_event_t event;
```

Initialize the icm20948 Settings
```
ArduinoICM20948Settings icmSettings =
{
  .i2c_speed = 400000,                // i2c clock speed
  .is_SPI = false,                    // Enable SPI, if disable use i2c
  .cs_pin = 10,                       // SPI chip select pin
  .spi_speed = 7000000,               // SPI clock speed in Hz, max speed is 7MHz
  .mode = 1,                          // 0 = low power mode, 1 = high performance mode
  .enable_gyroscope = true,           // Enables gyroscope output
  .enable_accelerometer = true,       // Enables accelerometer output
  .enable_magnetometer = true,        // Enables magnetometer output // Enables quaternion output
  .enable_gravity = true,             // Enables gravity vector output
  .enable_linearAcceleration = true,  // Enables linear acceleration output
  .enable_quaternion6 = true,         // Enables quaternion 6DOF output
  .enable_quaternion9 = true,         // Enables quaternion 9DOF output
  .enable_har = true,                 // Enables activity recognition
  .enable_steps = true,               // Enables step counter
  .gyroscope_frequency = 1,           // Max frequency = 225, min frequency = 1
  .accelerometer_frequency = 1,       // Max frequency = 225, min frequency = 1
  .magnetometer_frequency = 1,        // Max frequency = 70, min frequency = 1 
  .gravity_frequency = 1,             // Max frequency = 225, min frequency = 1
  .linearAcceleration_frequency = 1,  // Max frequency = 225, min frequency = 1
  .quaternion6_frequency = 50,        // Max frequency = 225, min frequency = 50
  .quaternion9_frequency = 50,        // Max frequency = 225, min frequency = 50
  .har_frequency = 50,                // Max frequency = 225, min frequency = 50
  .steps_frequency = 50               // Max frequency = 225, min frequency = 50  
};
```

Load the Settings to the icm20948 at setup
```
icm20948.init(icmSettings);
```

Call task to poll the sensor data for all enabled outputs - do this each time you require new data
```
icm20948.task();
```

Read desired output
```
pulldata(&cb,&event)
event.data.sensor.value----> (check event struct provided by invsens librayr)
```

## Contact

Contact at srikanth.popuri4444@gmail.com

Feel free to give feedback and suggest changes.
