/*
    Required Arduino libraries:
    Accelerometer
        Adafruit_FXOS8700
        Adafruit_FXAS21002C
        Adafruit Unified Sensor 
    Motor 
        Adafruit_Motor_Shield_V2_Library
    Uses code snippets from examples provided by Adafruit
        https://github.com/adafruit/Adafruit_FXOS8700
        https://github.com/adafruit/Adafruit_FXAS21002C

*/
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x00069, 0x00070);
Adafruit_FXAS21002C gyroscope = Adafruit_FXAS21002C(0x0021002C);
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1); //replace 1 with 2, 3, 4, depending on which port it's connected to (x for Mx, x in {1, 2, 3, 4})
double accel[3] = {0.0, 0.0, 0.0};
double mag[3] = {0.0, 0.0, 0.0};
double gyro[3] = {0.0, 0.0, 0.0};

int32_t timer;
int16_t motorState = 0;


void setup(void) {
  Serial.begin(9600);

  /* Wait for the Serial Monitor */
  while (!Serial) {
    delay(1);
  }

  Serial.println("FXOS8700 Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!accelmag.begin(ACCEL_RANGE_2G)) {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while (1)
      ;
  }

  if (!gyroscope.begin()) {
    /* There was a problem detecting the FXAS21002C ... check your connections
     */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while (1)
      ;
  }


  /* Display some basic information on this sensor */
  displayGyroDetails();
  getState(); 
  AFMS.begin();
}

void loop(void) {
 if (millis() - timer > 500) {
    getState(); 
    adjustMotor();
 }
}

void adjustMotor(void) {
  switch (motorState) {
    case 0:
      myMotor->setSpeed(0);
      myMotor->run(FORWARD);
      break;
    case 4:
      myMotor->setSpeed(0);
      myMotor->run(BACKWARD);
      break;
    case 1:  
    case 3:
    case 5: 
    case 7:
      myMotor->setSpeed(48);
      break;
    case 2:
    case 6:
      myMotor->setSpeed(64);
      break; 
  }
  motorState += 1;
  motorState %= 8;
}
void displayGyroDetails(void) {
  sensor_t sensor;
  gyroscope.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    0x");
  Serial.println(sensor.sensor_id, HEX);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" rad/s");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" rad/s");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" rad/s");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayAccMagDetails(void) {
  sensor_t accel, mag;
  accelmag.getSensor(&accel, &mag);
  Serial.println("------------------------------------");
  Serial.println("ACCELEROMETER");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(accel.name);
  Serial.print("Driver Ver:   ");
  Serial.println(accel.version);
  Serial.print("Unique ID:    0x");
  Serial.println(accel.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(accel.max_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Min Value:    ");
  Serial.print(accel.min_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Resolution:   ");
  Serial.print(accel.resolution, 8);
  Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("MAGNETOMETER");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(mag.name);
  Serial.print("Driver Ver:   ");
  Serial.println(mag.version);
  Serial.print("Unique ID:    0x");
  Serial.println(mag.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(mag.max_value);
  Serial.println(" uT");
  Serial.print("Min Value:    ");
  Serial.print(mag.min_value);
  Serial.println(" uT");
  Serial.print("Resolution:   ");
  Serial.print(mag.resolution);
  Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
  timer = millis();
}

void getState(void) {
  sensors_event_t aevent, mevent, gevent; 

  
  /* Get a new sensor event */
  accelmag.getEvent(&aevent, &mevent);
  gyroscope.getEvent(&gevent);
  timer = millis();
  Serial.print("Time: ");
  Serial.println(timer);
  accel[0] = aevent.acceleration.x;
  accel[1] = aevent.acceleration.y;
  accel[2] = aevent.acceleration.z;
  /* Display the accel results (acceleration is measured in m/s^2) */
  Serial.println("Acceleration (m/s^2) ");
  Serial.println("X    Y    Z");
  Serial.print(accel[0], 4);
  Serial.print(" "); 
  Serial.print(accel[1], 4);
  Serial.print(" ");
  Serial.print(accel[2], 4);
  Serial.println("  "); 

  
  mag[0] = mevent.magnetic.x;
  mag[1] = mevent.magnetic.y;
  mag[2] = mevent.magnetic.z;
  /* Display the mag results (mag data is in uTesla) */
  Serial.println("Magnetic field (uT) ");
  Serial.println("X    Y    Z"); 
  Serial.print(mag[0], 4);
  Serial.print("  "); 
  Serial.print(mag[1], 4);
  Serial.print("  "); 
  Serial.print(mag[2], 4);
  Serial.println("  "); 

  /* Display the results (speed is measured in rad/s) */
  gyro[0] = gevent.gyro.x;
  gyro[1] = gevent.gyro.y;
  gyro[2] = gevent.gyro.z;
  /* Display the mag results (mag data is in uTesla) */
  Serial.println("Gyroscope speed (rad/s) ");
  Serial.println("X    Y    Z"); 
  Serial.print(gyro[0], 4);
  Serial.print("  "); 
  Serial.print(gyro[1], 4);
  Serial.print("  "); 
  Serial.print(gyro[2], 4);
  Serial.println("  "); 
 
}
