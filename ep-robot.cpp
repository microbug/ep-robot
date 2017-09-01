// Copyright 2017 Richard Coleman

// For LCD
#include <LiquidCrystal.h>

// Initialise LiquidCrystal as LCD
//                RS  RW  EN  D4  D5  D6  D7
LiquidCrystal LCD(53, 51, 49, 47, 45, 43, 41);

// Define L298N pins
//  Left
const int pwm_pin_l = 2;
const int dir1_pin_l = 3;
const int dir2_pin_l = 4;
//  Right
const int dir1_pin_r = 5;
const int dir2_pin_r = 6;
const int pwm_pin_r = 7;

// Define button pins
const int button_pin_1 = 35;
const int button_pin_2 = 37;

// Define RX pins
const int rx_pin_channel_1 = 50;
const int rx_pin_channel_2 = 48;
const int rx_pin_channel_3 = 46;

// Define RX constants
const int rx_min = 963;
const int rx_max = 1950;

unsigned long loop_count;

#define LEFT_MOTOR true
#define RIGHT_MOTOR false

// Various precompiler settings
#define TEST_MOTORS false
#define PRINT_DELAY_INFO false
#define PRINT_LOOP_FREQUENCY false
#define PRINT_MOTOR_VELOCITY false
#define PRINT_WARNINGS false
#define WAIT_FOR_BUTTON_ON_STARTUP false


void setup() {
    Serial.begin(2000000);
    Serial.println("\r\n--------------- RUNNING SETUP ---------------\r\n");

    pinMode(dir1_pin_l, OUTPUT);
    pinMode(dir2_pin_l, OUTPUT);
    pinMode(pwm_pin_l, OUTPUT);
    pinMode(dir1_pin_r, OUTPUT);
    pinMode(dir2_pin_r, OUTPUT);
    pinMode(pwm_pin_r, OUTPUT);

    // Run self test on LCD
    Serial.println("\r\nTesting LCD");
    byte lcd_test_character[8] = {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,
        0x1F, 0x1F};
    LCD.createChar(0, lcd_test_character);
    LCD.begin(16, 2);
    LCD.print("ABCDEFGHIJKLMNOP");
    LCD.setCursor(0, 1);
    LCD.print("QRSTUVWXYZ123456");
    delay(500);
    LCD.clear();
    for (char row = 0; row <= 1; row++) {
        for (char column = 0; column <= 15; column++) {
            LCD.setCursor(column, row);
            LCD.write(byte(0));
        }
    }
    delay(500);
    Serial.println("LCD test complete");
    LCD.clear();
    LCD.print("Initialising...");

    #if TEST_MOTORS
        // Run self test on motors
        Serial.println("\r\nRunning motor test");
        LCD.setCursor(0, 1);
        // Must print 16 characters to clear any previous text
        LCD.print("Testing motors  ");
        int i;
        for (i = 0; i < 256; i++) {
            motor_set_velocity(LEFT_MOTOR, i);
            motor_set_velocity(RIGHT_MOTOR, i);
            delay(10);
        }
        delay(500);
        motor_set_velocity(LEFT_MOTOR, 0);
        motor_set_velocity(RIGHT_MOTOR, 0);
        Serial.println("Completed motor test");
    #endif

    pinMode(rx_pin_channel_1, INPUT);
    pinMode(rx_pin_channel_2, INPUT);
    pinMode(rx_pin_channel_3, INPUT);

    #if WAIT_FOR_BUTTON_ON_STARTUP
        Serial.println("\r\nWaiting for button");
        LCD.setCursor(0, 1);
        LCD.print("Press button... ");

        // button_pin_1 is 0v, button_pin_2 is an input with internal pullup
        // resistor. When button is pressed, button_pin_2 is pulled low.
        pinMode(button_pin_1, OUTPUT);
        digitalWrite(button_pin_1, LOW);

        pinMode(button_pin_2, INPUT_PULLUP);
        delay(20);
        bool button_pressed = false;

        // Debounce: check if button is depressed twice in 50ms
        while (button_pressed == false) {
            if (digitalRead(button_pin_2) == LOW) {
                delay(50);
                if (digitalRead(button_pin_2) == LOW) {
                    button_pressed = true;
                }
            }
            delay(10);
        }
    #endif

    LCD.clear();
    LCD.print("Ready");
    Serial.println("\r\n--------------- SKETCH READY ---------------\r\n");
}


void loop() {
    // Channel 1: throttle
    int rx_channel_1_raw = pulseIn(rx_pin_channel_1, HIGH, 25000);
    int rx_channel_1 = map(rx_channel_1_raw, rx_min, rx_max, -255, 255);
    rx_channel_1 = constrain(rx_channel_1, -255, 255);
    if (-5 < rx_channel_1 && rx_channel_1 < 5) {
        rx_channel_1 = 0;
    }

    // Channel 2: direction
    int rx_channel_2_raw = pulseIn(rx_pin_channel_2, HIGH, 25000);
    int rx_channel_2 = map(rx_channel_2_raw, rx_min, rx_max, -255, 255);
    rx_channel_2 = constrain(rx_channel_2, -255, 255);
    if (-5 < rx_channel_2 && rx_channel_2 < 5) {
        rx_channel_2 = 0;
    }

    int left_motor, right_motor;
    left_motor = rx_channel_1;
    right_motor = rx_channel_1;

    if (rx_channel_2 != 0) {

    }

    motor_set_velocity(LEFT_MOTOR, left_motor);
    motor_set_velocity(RIGHT_MOTOR, right_motor);
    

    loop_count++;
}


void motor_set_velocity(bool motor, int velocity) {

    int pwm_pin, dir1_pin, dir2_pin;

    if (motor == LEFT_MOTOR) {
        pwm_pin = pwm_pin_l;
        dir1_pin = dir1_pin_l;
        dir2_pin = dir2_pin_l;
    } else {
        pwm_pin = pwm_pin_r;
        dir1_pin = dir1_pin_r;
        dir2_pin = dir2_pin_r;
    }

    analogWrite(pwm_pin, abs(velocity));

    if ((velocity < 0 && motor == LEFT_MOTOR) || (velocity > 0 && motor == RIGHT_MOTOR)) {
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, HIGH);
    } else if ((velocity > 0 && motor == LEFT_MOTOR) || (velocity < 0 && motor == RIGHT_MOTOR)) {
        digitalWrite(dir1_pin, HIGH);
        digitalWrite(dir2_pin, LOW);
    }
}