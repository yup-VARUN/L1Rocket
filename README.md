# L1Rocket
# Flight Software &amp; ino schematic files

__Sensors:__
- Pressure Sensor: BMP280 | For altitude
- Accelerometer & Gyroscope: MPU6050 | For Navigation (inertial measurement unit)
- GPS Module: NEO-6M | For location sensing
- Magnetometer: GY-273 | For orientation sensing
- Camera Module: Raspberry Pi Camera Module 3 - 12MP 120 Degree | For capturing the video footage

__Signal Receiver & Transmitters:__
- nRF24
- LoRa Module

__Algorithms:__
- Kalman Filter for sensor fusion
- Low Pass filter

__Pyro Channels:__
- MOSFETs

__Microcontroller:__
- STM32
