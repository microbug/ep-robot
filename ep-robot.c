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

void motor_set_velocity(char motor, int velocity) {
  /*
   * char motor: 'L' or 'R'; left or right motor
   * int velocity: -128 to 128; velocity of motor
   */
   if (velocity < -128 or velocity > 128) {
    Serial.println("Error: velocity out of bounds");
   }
   else {
    Serial.println("-- Setting motor velocity --")
   }

   int pwm_pin;
   int dir1_pin;
   int dir2_pin;

   Serial.println("velocity=");
   Serial.println(velocity);
   switch (motor) {
    case 'L':
      Serial.println("motor=L");
      pwm_pin = pwm_pin_l;
      dir1_pin = dir1_pin_l;
      dir2_pin = dir2_pin_l;

    case 'R':
      Serial.println("motor=R");
      pwm_pin = pwm_pin_r;
      dir1_pin = dir1_pin_r;
      dir2_pin = dir2_pin_r;
   }

   if (velocity == 0) {
    analogWrite(pwm_pin, 0);
   }

   if (velocity > 0) {
    analogWrite(pwm_pin, velocity);
    digitalWrite(dir1_pin, LOW);
    digitalWrite(dir2_pin, HIGH);
   }

   if (velocity < 0) {
    analogWrite(pwm_pin, (velocity * -1));
    digitalWrite(dir1_pin, HIGH);
    digitalWrite(dir2_pin, LOW);
   }
}

void loop() {
  motor_set_velocity('L', -64);
  motor_set_velocity('R', -64);
  delay(1000);
  motor_set_velocity('R', 0);
  motor_set_velocity('L', 0);
  delay(1000);
  motor_set_velocity('R', 64);
  motor_set_velocity('L', 64);
  delay(1000);


  // if (Serial.available() > 0) {
  //   int inByte = Serial.read();
    
    // switch (inByte) {
    //   case '1':  // Motor A forward
    //     analogWrite(PWMpinL, motorSpeed);
    //     digitalWrite(dir1pinL, LOW);
    //     digitalWrite(dir2pinL, HIGH);
    //     Serial.println("Motor A forward");
    //     break;

    //   case '2':  // Motor A stop/freespin
    //     analogWrite(PWMpinL, 0);
    //     digitalWrite(dir1pinL, LOW);
    //     digitalWrite(dir2pinL, HIGH);
    //     Serial.println("Motor A stop");
    //     break;

    //   case '3':  // Motor A reverse
    //     analogWrite(PWMpinL, motorSpeed);
    //     digitalWrite(dir1pinL, HIGH);
    //     digitalWrite(dir2pinL, LOW);
    //     Serial.println("Motor A reverse");
    //     break;

    //   case '4':  // Motor B forward
    //     analogWrite(PWMpinR, motorSpeed);
    //     digitalWrite(dir1pinR, LOW);
    //     digitalWrite(dir2pinR, HIGH);
    //     Serial.println("Motor B forward");
    //     break;

    //   case '5':  // Motor B stop/freespin
    //     analogWrite(PWMpinR, 0);
    //     digitalWrite(dir1pinR, LOW);
    //     digitalWrite(dir2pinR, HIGH);
    //     Serial.println("Motor B stop");
    //     break;

    //   case '6':  // Motor B reverse
    //     analogWrite(PWMpinR, motorSpeed);
    //     digitalWrite(dir1pinR, HIGH);
    //     digitalWrite(dir2pinR, LOW);
    //     Serial.println("Motor B reverse");
    //     break;

    //   default:
    //     digitalWrite(dir1pinL, LOW);
    //     digitalWrite(dir2pinL, LOW);
    //     digitalWrite(PWMpinL, LOW);
    //     digitalWrite(dir1pinR, LOW);
    //     digitalWrite(dir2pinR, LOW);
    //     digitalWrite(PWMpinR, LOW);
    //     Serial.println("Stopping all motors");
    //     break;
    // }  // End switch
  // }  // End if
}  // End loop
