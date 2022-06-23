# MyAHRS_Driver
MyAHRS+ IMU C++ Driver 

```
https://github.com/withrobot/myAHRS_plus/tree/master/tutorial
```

The myAHRS+ is a low cost high performance AHRS(Attitude Heading Reference System). 
Its attitude output is more stable to acceleration and magnetic disturbances then other low cost AHRS/IMU products. 
Communication and configuration are enabled via UART/USB interface for user applications. And I2C interface is available for embedded application like arduino projects.
The GUI(myAHRS+ Monitor) is available, which allows users configure all myAHRS+  settings, view attitude of myAHRS+ and IMU(Inertial Measurement Unit) data in realtime and save sensor data in a text file. Custom user software may be developed using the myAHRS+ SDK.

## Features

* Sensors
  * Triple axis 16-bit gyroscope : ± 2,000 dps
  * Triple axis 16-bit accelerometer : ± 16 g
  * Triple axis 13-bit magnetometer  : ± 1200 μT
* On board software
  * Exteneded Kalman filter
  * max 100 Hz output rate
    * Attitude : Euler angle, Quaternion
    * sensor : acceleration, rotation rate, magnetic field
* Connectivity
  * USB : Virtual COM PORT
  * UART : Standard baud rates up to 460800 bps
  * I2C : up to 1kHz
* GUI(myAHRS+ Monitor)
  * display attitude and sensor data from myAHRS+ on various viewers
  * Configuration
  * magnetometer calibration


