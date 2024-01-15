# L1Rocket | High Powered Model Rocket Level-1
### (as per NAR --> Rocket motors from Classes):
![image](https://github.com/yup-VARUN/L1Rocket/assets/110617721/90c8edc8-8cfb-430f-9047-7eff27ad1070)

## Gameplan:
Step 1: Shortlist all of the sensors, actuators, microcontrollers, and other hardware components that would be the perfect fit for the given problem statement.
Step 2: Come up with a schematic.
Step 3: Write the program for each individual component.
Step 4: Test all those individual parts of codes with the microcontroller on the breadboard or in the Proteus Software.
Step 5: Integrate all those components together.
Step 6: Perform tests.
Step 7: Design a nice board, and route all the copper traces onto a PCB of the right size and shape.
Step 8: Get the PCB manufactured, or manufacture it yourself.
Step 9: Solder everything together.

## Avionics Goals to be accomplished:
- Sensor Calibration,
- Test and initiate the telemetry and share the data through telemetry,
- Ignite the rocket motor with a wireless command,
- Detect the apogee, 
- Eject the parachute,
- Data logging(save the sensor locally on the flight computer),
- Location sensing,
- Start looking up whether landed or not 
- Once landed,
- Stop the Camera Recording after 5 seconds,
- Send the location over the LoRa module,
- Write all of the data from flash chips to SD Card,
- Activate the buzzer after some delay.

## Components Specifications:
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

__Memory:__
- Flash memory chip: FLASH - NOR Memory IC 256Mbit SPI - Quad I/O 133 MHz 16-SOIC | Storing the data reliably(soldered connections), quickly(133 MHz)
- SD card, for easier data recovery & redundancy.
