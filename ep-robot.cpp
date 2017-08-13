// Copyright 2017 Richard Coleman

// For LCD
#include <LiquidCrystal.h>
// For MPU-9255
#include <MPU9250.h>
#include <quaternionFilters.h>

// Initialise LiquidCrystal as LCD
//                RS  RW  EN  D4  D5  D6  D7
LiquidCrystal LCD(53, 51, 49, 47, 45, 43, 41);

// Define L298N pins
//  Left
int dir1_pin_l = 3;
int dir2_pin_l = 4;
int pwm_pin_l = 2;
//  Right
int dir1_pin_r = 5;
int dir2_pin_r = 6;
int pwm_pin_r = 7;

// Define MPU9250 (actually MPU9255)
MPU9250 IMU;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Running setup()");

  pinMode(dir1_pin_l, OUTPUT);
  pinMode(dir2_pin_l, OUTPUT);
  pinMode(pwm_pin_l, OUTPUT);
  pinMode(dir1_pin_r, OUTPUT);
  pinMode(dir2_pin_r, OUTPUT);
  pinMode(pwm_pin_r, OUTPUT);

  // Run self test on LCD
  Serial.println("\r\nTesting LCD");
  LCD.begin(16, 2);
  LCD.print("ABCDEFGHIJKLMNOP");
  LCD.setCursor(0, 1);
  LCD.print("QRSTUVWXYZ123456");
  delay(1000);
  LCD.clear();
  LCD.print("Initialising...");
  Serial.println("LCD test complete");

  // Run self test on motors
  Serial.println("\r\nRunning motor test");
  left_motor_set_velocity(255, true);
  right_motor_set_velocity(255, false);
  delay(1000);
  left_motor_set_velocity(255, false);
  right_motor_set_velocity(255, true);
  delay(1000);
  left_motor_set_velocity(0, false);
  right_motor_set_velocity(0, false);
  Serial.println("Completed motor test");

  Serial.println("\r\nBeginning IMU tests");
  byte c = IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9255 has address "); Serial.println(c, HEX);
  Serial.print("MPU9255 should have address "); Serial.println(0x73, HEX);
  if (c != 0x73) {
    Serial.println("ERROR: MPU9255 did not have expected address");
    while (1) {}
  }

  // Run self test and calibration on IMU and report results
  Serial.println("Running IMU internal self test");
  IMU.MPU9250SelfTest(IMU.SelfTest);
  Serial.print("x-axis self test: acceleration trim within : ");
  Serial.print(IMU.SelfTest[0], 1); Serial.println("% of factory value");
  Serial.print("y-axis self test: acceleration trim within : ");
  Serial.print(IMU.SelfTest[1], 1); Serial.println("% of factory value");
  Serial.print("z-axis self test: acceleration trim within : ");
  Serial.print(IMU.SelfTest[2], 1); Serial.println("% of factory value");
  Serial.print("x-axis self test: gyration trim within : ");
  Serial.print(IMU.SelfTest[3], 1); Serial.println("% of factory value");
  Serial.print("y-axis self test: gyration trim within : ");
  Serial.print(IMU.SelfTest[4], 1); Serial.println("% of factory value");
  Serial.print("z-axis self test: gyration trim within : ");
  Serial.print(IMU.SelfTest[5], 1); Serial.println("% of factory value");
  Serial.println("Completed IMU internal self test, running IMU calibration");
  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  Serial.println("Calibrated IMU, beginning IMU initialisation");
  IMU.initMPU9250();
  Serial.println("Completed IMU initialisation");

  LCD.clear();
  LCD.print("Ready");
  Serial.println("\r\n--------------- SKETCH READY ---------------\r\n");
}


void left_motor_set_velocity(unsigned int speed, bool clockwise) {
  /*
   * int speed: speed from 0 to 255
   * bool clockwise: whether to turn clockwise or anticlockwise
   */
  Serial.println("Setting left motor velocity");

  if (speed == 0) {
    analogWrite(pwm_pin_l, 0);
  } else {
    analogWrite(pwm_pin_l, speed);
  }

  if (clockwise == true) {
    digitalWrite(dir1_pin_l, LOW);
    digitalWrite(dir2_pin_l, HIGH);
  } else {
    digitalWrite(dir1_pin_l, HIGH);
    digitalWrite(dir2_pin_l, LOW);
  }
}


void right_motor_set_velocity(unsigned int speed, bool clockwise) {
  /*
   * int speed: speed from 0 to 255
   * bool clockwise: whether to turn clockwise or anticlockwise
   */
  Serial.println("Setting right motor velocity");

  if (speed == 0) {
    analogWrite(pwm_pin_r, 0);
  } else {
    analogWrite(pwm_pin_r, speed);
  }

  if (clockwise == true) {
    digitalWrite(dir1_pin_r, LOW);
    digitalWrite(dir2_pin_r, HIGH);
  } else {
    digitalWrite(dir1_pin_r, HIGH);
    digitalWrite(dir2_pin_r, LOW);
  }
}


void loop() {
  Serial.println("Running loop()");
  delay(5000);
}
