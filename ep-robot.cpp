// Copyright 2017 Richard Coleman

// For LCD
#include <LiquidCrystal.h>
// For MPU-9250
#include <MPU9250.h>
#include <quaternionFilters.h>

// Initialise LiquidCrystal as lcd
//                RS  RW  EN  D4  D5  D6  D7
LiquidCrystal lcd(53, 51, 49, 47, 45, 43, 41);

// L298N Pins
int dir1_pin_l = 3;
int dir2_pin_l = 4;
int pwm_pin_l = 2;
int dir1_pin_r = 5;
int dir2_pin_r = 6;
int pwm_pin_r = 7;

// Define MPU9250
MPU9250 IMU;


void setup() {
  Serial.begin(115200);
  Serial.println("Initialising sketch");

  pinMode(dir1_pin_l, OUTPUT);
  pinMode(dir2_pin_l, OUTPUT);
  pinMode(pwm_pin_l, OUTPUT);
  pinMode(dir1_pin_r, OUTPUT);
  pinMode(dir2_pin_r, OUTPUT);
  pinMode(pwm_pin_r, OUTPUT);

  lcd.begin(16, 2);
  lcd.print("LCD working :)");

  Serial.println("Testing motors");
  left_motor_set_velocity(256, true);
  right_motor_set_velocity(256, true);
  delay(1000);
  left_motor_set_velocity(256, false);
  right_motor_set_velocity(256, false);
  delay(1000);
  left_motor_set_velocity(0, false);
  right_motor_set_velocity(0, false);
  Serial.println("Finished testing motors");

  // Run self test on IMU and report results
  Serial.println("Running IMU self test")
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

  Serial.println("Sketch ready");
}


void left_motor_set_velocity(unsigned int speed, bool clockwise) {
  /*
   * int speed: speed from 0 to 256
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
   * int speed: speed from 0 to 256
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

}
