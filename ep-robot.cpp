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

unsigned long loop_count;

#define FORWARDS true
#define BACKWARDS false

// Various precompiler settings
#define TEST_MOTORS false
#define PRINT_DELAY_INFO false
#define PRINT_LOOP_FREQUENCY false
#define PRINT_MOTOR_VELOCITY false
#define PRINT_WARNINGS false
#define WAIT_FOR_BUTTON_ON_STARTUP true


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
            motors_set_velocity(i, FORWARDS);
            delay(10);
        }
        delay(500);
        motors_set_velocity(0, FORWARDS);
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


    // Read input values
    int rx_channel_1 = pulseIn(rx_pin_channel_1, HIGH, 25000);
    rx_channel_1 = map(rx_channel_1, 906, 1620, 0, 255);

    int rx_channel_2 = pulseIn(rx_pin_channel_2, HIGH, 25000);
    rx_channel_2 = map(rx_channel_2, 906, 1620, 0, 255);

    Serial.print(rx_channel_1);
    Serial.print(",");
    Serial.print(rx_channel_2);
    Serial.print("\r\n");

    loop_count++;
}


void left_motor_set_velocity(unsigned char speed, bool clockwise) {
    /*
     * int speed: speed from 0 to 255
     * bool clockwise: whether to turn clockwise or anticlockwise
     */
    #if PRINT_MOTOR_VELOCITY
        Serial.println("Setting left motor velocity");
    #endif

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


void right_motor_set_velocity(unsigned char speed, bool clockwise) {
    /*
     * int speed: speed from 0 to 255
     * bool clockwise: whether to turn clockwise or anticlockwise
     */
    #if PRINT_MOTOR_VELOCITY
        Serial.println("Setting right motor velocity");
    #endif

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
