// Copyright 2017 Richard Coleman
#include <LiquidCrystal.h>

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

  Serial.println("Sketch ready");
}

// void motor_set_velocity(bool right_motor, int velocity) {
//   /*
//    * bool right_motor: whether to program the right motor
//    * int velocity: -128 to 128; velocity of motor
//    */
//   if (velocity < -128 || velocity > 128) {
//     Serial.println("Error: velocity out of bounds");
//   } else {
//     Serial.println("-- Setting motor velocity --");
//   }

//   int pwm_pin;
//   int dir1_pin;
//   int dir2_pin;

//   Serial.println("velocity=");
//   Serial.println(velocity);
//   switch (motor) {
//     case 'L':
//       Serial.println("motor=L");
//       pwm_pin = pwm_pin_l;
//       dir1_pin = dir1_pin_l;
//       dir2_pin = dir2_pin_l;

//     case 'R':
//       Serial.println("motor=R");
//       pwm_pin = pwm_pin_r;
//       dir1_pin = dir1_pin_r;
//       dir2_pin = dir2_pin_r;
//   }

//   if (velocity == 0) {
//     analogWrite(pwm_pin, 0);
//   }

//   if (velocity > 0) {
//     analogWrite(pwm_pin, velocity);
//     digitalWrite(dir1_pin, LOW);
//     digitalWrite(dir2_pin, HIGH);
//   }

//   if (velocity < 0) {
//     analogWrite(pwm_pin, (velocity * -1));
//     digitalWrite(dir1_pin, HIGH);
//     digitalWrite(dir2_pin, LOW);
//   }
// }

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
  left_motor_set_velocity(0, true);
  right_motor_set_velocity(128, true);
  delay(1000);
  left_motor_set_velocity(128, true);
  right_motor_set_velocity(0, true);
  delay(1000);
}
