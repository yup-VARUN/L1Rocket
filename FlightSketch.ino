
int counter = 0;
int sensor_error_counter = 0;

// Buttons:
int btn1 = 12;
int btn2 = 13;

// Servo:
#include <Servo.h>
Servo servomotor;

// SD-Card Module:
#include <SD.h>
#include <SPI.h>
File Myfile;
String data;
int cs = 2;

// MPU6050:
#include "Wire.h"                                                                                // This library allows you to communicate with I2C devices.
const int MPU_ADDR = 0x68;                                                                       // default I2C address of the MPU-6050.
int16_t accelerometer_x, accelerometer_y, accelerometer_z, gyro_x, gyro_y, gyro_z, temperature;  // variables for MPU6050 Raw Data
char tmp_str[7];                                                                                 // temporary variable used in convert function
int error;

// BMP180:
#include <Adafruit_BMP085.h>
#define seaLevelPressure_hPa 1013.25
Adafruit_BMP085 bmp;
const int BMP_ADDR = 0x77;
float pressure;
float temp;
float last_altitude;
float current_altitude;
float ground_altitude;
int apogee_counter = 0;
int landing_counter = 0;

// GPS NEO-6M-0-001:
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
TinyGPSPlus gps;
SoftwareSerial ss(4, 3);

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;  // waiting for serial port to connect
  }
  //////////////////////// SENSORS CHECK - Button1 Pressed //////////////////////////////
  while (digitalRead(btn1) == LOW){
    ; // Until the button 1 gets high/on, we will wait before the testing begins
  }
  
  // 1. SD Card:
  if (!SD.begin(cs)) {
    Serial.println("Card failed, or not present");
    exit(0);
  } else {
    Serial.println("card initialized.");
  }
  // 2. MPU 6050:
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);  // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B);                  // PWR_MGMT_1 register
  Wire.write(0);                     // set to zero (wakes up the MPU-6050)
  error = Wire.endTransmission(true);
  if (error != 0) {
    Serial.println("MPU-6050 failed, or not present");
    sensor_error_counter ++;
    exit(1);
  }
  // 3. BMP180:
  Wire.beginTransmission(BMP_ADDR);  // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B);                  // PWR_MGMT_1 register
  Wire.write(0);                     // set to zero (wakes up the MPU-6050)
  error = Wire.endTransmission(true);
  if (error != 0) {
    Serial.println("BMP180 failed, or not present");
    sensor_error_counter ++;
    exit(1);
  }
  // OR:
  // if (!bmp.begin()) {
  // Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  // while (1) {}
  // }
  ground_altitude = bmp.readAltitude();
  // 4. GPS:
  ss.begin(9600);
  if (ss.available()) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      Serial.print("Latitude= ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude= ");
      Serial.println(gps.location.lng(), 6);
    }
  } else {
    Serial.println("GPS failed, or not present");
    exit(2);
  }
  // 5. Servo:
  servomotor.attach(10);

  //////////////////////// COUNT DOWN - Button1 Pressed //////////////////////////////
  while (digitalRead(btn2) == LOW){
    ; // Until the button 2 gets high/on, we will wait before proceeding
  }
  delay(7000);
  tone(6, 200, 500);
  delay(500);
  tone(6, 200, 500);
  delay(500);
  tone(6, 200, 500);
  delay(500);

}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  Myfile = SD.open("file.csv", FILE_WRITE);
  //////////////////////////////// READING THE DATA //////////////////////////////////
  // 1. Reading MPU6050's Data:
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  accelerometer_x = Wire.read() << 8 | Wire.read();  // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read() << 8 | Wire.read();  // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read() << 8 | Wire.read();  // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read() << 8 | Wire.read();      // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read() << 8 | Wire.read();           // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read() << 8 | Wire.read();           // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read() << 8 | Wire.read();           // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  // 2. Reading BMP180's Data:
  pressure = bmp.readPressure();
  temp = bmp.readTemperature();
  current_altitude = bmp.readAltitude();
  if (last_altitude > current_altitude && current_altitude > (ground_altitude + 10)){
    // apogee detected:
    apogee_counter ++;
    if (apogee_counter > 5){
      //apogee confirmed:
      servomotor.write(90);
    }
  }
  else if (current_altitude <= (ground_altitude + 3) && current_altitude >= (ground_altitude - 3)){
    // landing detected:
    landing_counter ++;
    if (landing_counter > 30){
      //////// landing confirmed: /////////
      ;// stop data logging
      Wire.endTransmission(true);// deactivate all I2C sensors
      delay(60*5*1000); // wait for the owner to reach
      tone(6, 200,60*5*1000); // activate the buzzer
    }
  }
  
  // 3. Reading NEO-6M-0-001's Data:
  

  //////////////////////////////// WRITING THE DATA //////////////////////////////////
  // 1. Writing MPU6050's Data:
  Myfile = SD.open("MPU.csv", FILE_WRITE);
  Myfile.println("%i %i %i %i %i %i %i %i", accelerometer_x, accelerometer_y, accelerometer_z, gyro_x, gyro_y, gyro_z, temperature, millis());
  Myfile.close();
  // 2. Writing BMP180's Data:
  Myfile = SD.open("BMP.csv", FILE_WRITE);
  Myfile.println("%f %f %f %i", pressure, temperature, current_altitude, millis());
  Myfile.close();
  // 3. Writing NEO-6M-0-001's Data:
  Myfile = SD.open("GPS.csv", FILE_WRITE);
  Myfile.println(" %i", millis());
  Myfile.close();

  ///////////////////////////////////  IGNITION  /////////////////////////////////////
  if (counter == 5) {
    // IGNITION:
    digitalWrite(8, HIGH);
  }
  else if (counter == 15) {
    // IGNITION:
    digitalWrite(8, LOW);
  }
  ////////////////////////////////////////////////////////////////////////////////////
  counter++;
}

/* PROJECT ROCKET "EXCITEMENT", BY VARUN AHLAWAT:
FOLLOWING STEPS ARE PERFORMED BY THIS CODE:
1.0) Wait till button 1 (or wireless Command from RF module or onboard)
1.1) Check whether all the sensors are working as intended
2.0) Wait till button 2 (or wireless Command from RF module or onboard)
2.1) Ignite the motor after 10 seconds(buzzer sound) | Pin 10 | Ignitor
2.2) Report errors to nRF24 Reciever(if any)
3.1) Reading the sensors' data
3.2) SD card data logging
4  ) Apogee detection
5  ) Parachute ejection
6  ) Landing detection
7  ) deactivate sensors, stop data logging
8  ) Buzzer Noise a few minutes after landing
*/