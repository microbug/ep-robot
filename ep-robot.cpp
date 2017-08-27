// Copyright 2017 Richard Coleman
// MPU925(0/5) code taken from Kris Winer (under Beerware license)

// TODO:
// - Add dt calculation for filter
// - Add motor control code:
//     - Basic code
//     - Encoders to get proportional velocity

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


// Define complementary filter variables

// Current angle (assuming starting at 0)
float angle = 0;

// Milliseconds of trim to remove from delay time
const int filter_target_period_trim = 1;

// Target filter frequency in Hz
const float filter_target_frequency = 25;

// Target filter time period in Hz (calculated from frequency above)
const float filter_target_period = (1.0 / filter_target_frequency);

// First filter constant
const float filter_constant_a = 0.9259;

// Second filter constant (must be complementary;
// filter_constant_a + filter_constant_b === 1)
const float filter_constant_b = 1.0 - filter_constant_a;

// Used to keep track of time since filter last ran
unsigned long filter_last_time;

// Used to store the last frequency of the filter
// so that it can be printed occasionally
float filter_last_frequency = 0;

// Various precompiler settings
#define TEST_MOTORS false
#define PRINT_ACCEL_DATA false
#define PRINT_ANGLE false
#define PLOT_ANGLE true
#define PRINT_DELAY_INFO false
#define PRINT_GYRO_DATA false
#define PRINT_LOOP_FREQUENCY false
#define PRINT_MOTOR_VELOCITY false
#define PRINT_WARNINGS false
#define WAIT_FOR_BUTTON_ON_STARTUP false


// For MPU9255
#include <SPI.h>
#include <Wire.h>


// Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00  // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L  0x03  // data
#define AK8963_XOUT_H  0x04
#define AK8963_YOUT_L  0x05
#define AK8963_YOUT_H  0x06
#define AK8963_ZOUT_L  0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B  // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75  // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
// Define AD0 to be 0 if tied to GND, 1 if tied to VCC (3.3V)
#define ADO 0
#if ADO
    #define MPU9250_ADDRESS 0x69  // Device address when ADO = 1
#else
    #define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
    #define AK8963_ADDRESS 0x0C   // Address of magnetometer
#endif

// Set initial input parameters
enum Ascale {
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum Gscale {
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

enum Mscale {
    MFS_14BITS = 0,  // 0.6 mG per LSB
    MFS_16BITS       // 0.15 mG per LSB
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
// Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mscale = MFS_16BITS;
// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
uint8_t Mmode = 0x02;
float aRes, gRes, mRes;  // scale resolutions per LSB for the sensors

float SelfTest[6];  // Holds results of gyro and accelerometer self test
float gyroBias[3] = {0, 0, 0};
float accelBias[3] = {0, 0, 0};
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float ax, ay, az, gx, gy, gz;  // Variables to hold latest sensor data values


void setup() {
    Wire.begin();
    Serial.begin(115200);
    Serial.println("\r\n--------------- RUNNING SETUP ---------------\r\n");

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
    Serial.println("LCD test complete");
    LCD.clear();
    LCD.print("Initialising...");

    #if TEST_MOTORS
        // Run self test on motors
        Serial.println("\r\nRunning motor test");
        LCD.setCursor(0, 1);
        // Must print 16 characters to clear any previous text
        LCD.print("Testing motors  ");
        // left_motor_set_velocity(255, true);
        // right_motor_set_velocity(255, false);
        // delay(1000);
        // left_motor_set_velocity(255, false);
        // right_motor_set_velocity(255, true);
        // delay(1000);
        // left_motor_set_velocity(0, false);
        // right_motor_set_velocity(0, false);
        int i;
        for (i = 0; i < 256; i++) {
            left_motor_set_velocity(i, true);
            right_motor_set_velocity(i, false);
            delay(10);
        }
        delay(500);
        left_motor_set_velocity(0, true);
        right_motor_set_velocity(0, true);
        Serial.println("Completed motor test");
    #endif

    Serial.println("\r\nBeginning IMU tests");
    LCD.setCursor(0, 1);
    LCD.print("Testing IMU:addr");
    byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    Serial.print("MPU9255 has address "); Serial.println(c, HEX);
    Serial.print("MPU9255 should have address "); Serial.println(0x73, HEX);
    if (c != 0x73) {
        Serial.println("ERROR: MPU9255 did not have expected address");
        LCD.setCursor(0, 1);
        LCD.print("ERR MPU9255 ADDR");
        while (1) {}
    }

    // Run self test and calibration on IMU and report results
    Serial.println("Running IMU internal self test");
    LCD.setCursor(0, 1);
    LCD.print("Testing IMU:self");
    MPU9250SelfTest(SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(SelfTest[0], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(SelfTest[1], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(SelfTest[2], 1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(SelfTest[3], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(SelfTest[4], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(SelfTest[5], 1); Serial.println("% of factory value");
    Serial.println("Completed IMU internal self test");

    Serial.println("Running IMU calibration");
    LCD.setCursor(0, 1);
    LCD.print("Calibrating IMU ");
    calibrateMPU9250(gyroBias, accelBias);
    Serial.print("MPU9255 accelerometer bias (mg): x=");
    Serial.print(static_cast<int>(1000*accelBias[0]));
    Serial.print(" y=");
    Serial.print(static_cast<int>(1000*accelBias[1]));
    Serial.print(" z=");
    Serial.print(static_cast<int>(1000*accelBias[2]));
    Serial.println(" ");
    Serial.print("MPU9255 gyrometer bias (deg/s): x=");
    Serial.print(gyroBias[0], 1);
    Serial.print(" y=");
    Serial.print(gyroBias[1], 1);
    Serial.print(" z=");
    Serial.print(gyroBias[2], 1);
    Serial.println(" ");


    Serial.println("Calibrated IMU, beginning IMU initialisation");
    LCD.setCursor(0, 1);
    LCD.print("Initialising IMU");
    initMPU9250();
    Serial.println("Completed IMU initialisation");

    Serial.print("\r\nFilter constant A: ");
    Serial.println(filter_constant_a);
    Serial.print("Filter constant B: ");
    Serial.println(filter_constant_b);
    Serial.print("Targetting frequency ");
    Serial.print(filter_target_frequency);
    Serial.print("Hz with corresponding period ");
    Serial.print(filter_target_period);
    Serial.println("s");

    #if WAIT_FOR_BUTTON_ON_STARTUP
        Serial.println("\r\nWaiting for button press...");
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
    filter_last_time = micros();
}


void loop() {
    Serial.println("\r\nRunning loop()");

    //float ax, ay, az, gx, gy, gz;
    //get_imu_data(&ax, &ay, &az, &gx, &gy, &gz);
    get_imu_data();

    #if PRINT_ANGLE
        Serial.print("Angle: ");
        Serial.print(angle);
        Serial.println("°");
    #elif PLOT_ANGLE
        Serial.println(angle);
    #endif

    delay_to_meet_filter_frequency_target();
    update_complementary_filter(gy, ax);

    #if PRINT_LOOP_FREQUENCY
        Serial.print("Loop running at ");
        Serial.print(filter_last_frequency, 2);
        Serial.println("Hz");
    #endif
}


void left_motor_set_velocity(unsigned int speed, bool clockwise) {
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


void right_motor_set_velocity(unsigned int speed, bool clockwise) {
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


void get_imu_data() {
    while (1) {
        // If MPU9255 ready bit is set
        if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
            break;
        } else {
            #if PRINT_WARNINGS
                Serial.println("Warning: couldn't get MPU9250 data, retrying");
            #endif
        }
    }

    readAccelData(accelCount);
    getAres();
    // Calculate acceleration values in Gs
    ax = (static_cast<float>(accelCount[0]) * aRes);
    ay = (static_cast<float>(accelCount[1]) * aRes);
    az = (static_cast<float>(accelCount[2]) * aRes);
    #if PRINT_ACCEL_DATA
        Serial.print("MPU9255 accelerometer reading: x=");
        Serial.print(ax);
        Serial.print("g y=");
        Serial.print(ay);
        Serial.print("g z=");
        Serial.print(az);
        Serial.println("g");
    #endif

    readGyroData(gyroCount);
    getGres();
    // Calculate gyro values in °/s
    gx = (static_cast<float>(gyroCount[0]) * gRes);
    gy = (static_cast<float>(gyroCount[1]) * gRes);
    gz = (static_cast<float>(gyroCount[2]) * gRes);
    #if PRINT_GYRO_DATA
        Serial.print("MPU9255 gyrometer reading: x=");
        Serial.print(gx);
        Serial.print("°/s y=");
        Serial.print(gy);
        Serial.print("°/s z=");
        Serial.print(gz);
        Serial.println("°/s");
    #endif
}


// Update angle using complementary filter
void update_complementary_filter(float gy, float ax) {
    float dt = (micros() - filter_last_time) / 1000000.0;
    #if PRINT_LOOP_FREQUENCY
        // store frequency (in Hz) in filter_last_frequency for later display
        filter_last_frequency = 1/dt;
    #endif
    // update filter
    angle = filter_constant_a*(angle + gy*dt) + filter_constant_b*ax;
    filter_last_time = micros();
}


void delay_to_meet_filter_frequency_target() {
    float dt = (micros() - filter_last_time) / 1000000.0;
    if (dt < filter_target_period) {
        int ms_to_wait = (filter_target_period - dt) * 1000.0;
        ms_to_wait -= filter_target_period_trim;
        if (ms_to_wait > 0) {
            #if PRINT_DELAY_INFO
                Serial.print("Delaying ");
                Serial.print(ms_to_wait);
                Serial.println("ms");
            #endif
            delay(ms_to_wait);
        }
    }
}








//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

                    /****************************************
                     * Functions for MPU9250/MPU9255 module *
                     ****************************************/

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////



void getMres() {
    switch (Mscale) {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
        case MFS_14BITS:
            mRes = 10.*4912./8190.;  // Proper scale to return milliGauss
            break;
        case MFS_16BITS:
            mRes = 10.*4912./32760.0;  // Proper scale to return milliGauss
            break;
    }
}

void getGres() {
    switch (Gscale) {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a way to calculate DPS/(ADC tick) based on that 2-bit value:
        case GFS_250DPS:
            gRes = 250.0/32768.0;
            break;
        case GFS_500DPS:
            gRes = 500.0/32768.0;
            break;
        case GFS_1000DPS:
            gRes = 1000.0/32768.0;
            break;
        case GFS_2000DPS:
            gRes = 2000.0/32768.0;
            break;
    }
}

void getAres() {
    switch (Ascale) {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11)
    // Here's a way to calculate DPS/(ADC tick) based on that 2-bit value:
        case AFS_2G:
            aRes = 2.0/32768.0;
            break;
        case AFS_4G:
            aRes = 4.0/32768.0;
            break;
        case AFS_8G:
            aRes = 8.0/32768.0;
            break;
        case AFS_16G:
            aRes = 16.0/32768.0;
            break;
    }
}


void readAccelData(int16_t * destination) {
    // xyz accel register data stored here
    uint8_t rawData[6];

    // Read the six raw data registers into data array
    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);

    // Turn the MSB and LSB into a signed 16-bit value
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}


void readGyroData(int16_t * destination) {
    // xyz gyro register data stored here
    uint8_t rawData[6];

    // Read the six raw data registers sequentially into rawData
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);

    // Turn the MSB and LSB into a signed 16-bit value
    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];
    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
}

void readMagData(int16_t * destination) {
    // xyz mag register data, ST2 register stored here, must read ST2 at end of
    // data acquisition
    uint8_t rawData[7];

    // If magnetometer ready bit is set
    if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) {
        // Read the six raw data and ST2 registers sequentially into rawData
        readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);

        uint8_t c = rawData[6];  // End data read by reading ST2 register

        // Check if magnetic sensor overflow set, if not then report data
        if (!(c & 0x08)) {
            // For each destination, turn the most significant bit and least
            // significant bit into a single signed 16-bit value
            // Data stored as little Endian
            destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
            destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
            destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
        }
    }
}

int16_t readTempData() {
    uint8_t rawData[2];  // temperature register data stored here

    // Read the data sequentially into rawData
    readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);

    // Turn the MSB and LSB into a signed 16-bit value
    return ((int16_t)rawData[0] << 8) | rawData[1];
}


void initAK8963(float * destination) {
    // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);  // Power down magnetometer
    delay(10);
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F);  // Enter Fuse ROM access mode
    delay(10);

    // Read the xyz-axis calibration values into rawData
    readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);

    // Return x-axis sensitivity adjustment values, etc.
    destination[0] =  static_cast<float>(rawData[0] - 128)/256. + 1.;
    destination[1] =  static_cast<float>(rawData[1] - 128)/256. + 1.;
    destination[2] =  static_cast<float>(rawData[2] - 128)/256. + 1.;

    // Power down magnetometer
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
    delay(10);

    // Configure the magnetometer for continuous read and highest resolution
    // - set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in
    //      CNTL register
    // - enable continuous mode data acquisition Mmode (bits [3:0]),
    //      0010 for 8 Hz and 0110 for 100 Hz sample rates

    // Set magnetometer data resolution and sample output data rate
    writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode);
    delay(10);
}


void initMPU9250() {
    // Clear sleep mode bit (6), enable all sensors
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
    delay(100);

    // Get stable time source: auto select clock source
    // to be PLL gyroscope reference if ready else
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    delay(200);

    // Configure Gyro and Thermometer:
    // - Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
    //   respectively; minimum delay time for this setting is 5.9 ms so
    //   sensor fusion update rates cannot be higher than 1 / 0.0059 = 170 Hz
    // - DLPF_CFG bits 2:0 = 011 -> limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz,
    // 8 kHz or 1 kHz
    writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

    // Set sample rate = (gyroscope output rate)/(1 + SMPLRT_DIV)
    // Use a 200 Hz rate; a rate consistent with the filter update rate
    // determined inset in CONFIG
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);

    // Set gyroscope full scale range
    // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are
    // left-shifted into positions 4:3

    uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x03;  // Clear Fchoice bits [1:0]
    c = c & ~0x18;  // Clear GFS bits [4:3]
    c = c | Gscale << 3;  // Set full scale range for the gyro
    // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
    // GYRO_CONFIG
    // c =| 0x00;
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c);

    // Set accelerometer full-scale range configuration
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | Ascale << 3;  // Set full scale range for the accelerometer
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c);

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by
    // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
    // 1.13 kHz
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
    c = c & ~0x0F;  // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c);
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz
    // because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable:
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
    // until interrupt cleared, clear on read of INT_STATUS, and enable
    // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
    // controlled by the Arduino as master
    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    // Enable data ready (bit 0) interrupt
    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);
    delay(100);
}


// Function that accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.
//
// This assumes that the MPU9250 is motionless and level with the ground
void calibrateMPU9250(float * dest1, float * dest2) {
    uint8_t data[12];  // data array to hold accelerometer and gyro xyz data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // Write 1 to bit 7, which will reset the device
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
    delay(100);

    // Get stable time source; auto select clock source to be PLL gyroscope
    // reference if ready else use the internal oscillator, bits 2:0 = 001
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
    delay(200);

    // Configure device for bias calculation:
    // Disable all interrupts
    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);
    // Disable FIFO
    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
    // Turn on internal clock source
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
    // Disable I2C master
    writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00);
    // Disable FIFO and I2C master modes
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);
    // Reset FIFO and DMP
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);
    delay(15);

    // Configure MPU9250 gyro and accelerometer for bias calculation:
    // Set low-pass filter to 188 Hz
    writeByte(MPU9250_ADDRESS, CONFIG, 0x01);
    // Set sample rate to 1 kHz
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
    // Set gyro full-scale to 250°/s, maximum sensitivity
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
    // Set accelerometer full-scale to 2 g, maximum sensitivity
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for
    // bias calculation
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);  // Enable FIFO
    // Enable gyro and accelerometer sensors for FIFO
    // (max size 512 bytes in MPU-9150)
    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);
    delay(40);  // Accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read:
    // Disable gyro and accelerometer sensors for FIFO
    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);

    // Read FIFO sample count
    readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]);
    fifo_count = ((uint16_t)data[0] << 8) | data[1];

    // How many sets of full gyro and accelerometer data for averaging
    packet_count = fifo_count/12;

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};

        // Read data for averaging
        readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]);

        // Form signed 16-bit integer for each sample in FIFO
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]);
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t) (((int16_t)data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t) (((int16_t)data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t) (((int16_t)data[10] << 8) | data[11]);

        // Sum individual signed 16-bit biases to get accumulated
        // signed 32-bit biases
        accel_bias[0] += (int32_t) accel_temp[0];
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
    }

    // Normalize sums to get average count biases
    accel_bias[0] /= (int32_t) packet_count;
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    // Remove gravity from the z-axis accelerometer bias calculation
    if (accel_bias[2] > 0L) {
        accel_bias[2] -= (int32_t) accelsensitivity;
    } else {
        accel_bias[2] += (int32_t) accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers,
    // which are reset to zero upon device startup

    // Divide by 4 to get 32.9 LSB per °/s to conform to expected bias input
    // format. Biases are additive, so change sign on calculated average gyro
    // biases.
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
    data[1] = (-gyro_bias[0]/4)       & 0xFF;
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
    writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
    writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
    writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
    writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
    writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

    // Output scaled gyro biases for display in the main program
    dest1[0] = static_cast<float>(gyro_bias[0]) / \
        static_cast<float>(gyrosensitivity);
    dest1[1] = static_cast<float>(gyro_bias[1]) / \
        static_cast<float>(gyrosensitivity);
    dest1[2] = static_cast<float>(gyro_bias[2]) / \
        static_cast<float>(gyrosensitivity);

    // Construct the accelerometer biases for push to the hardware accelerometer
    // bias registers. These registers contain factory trim values that must be
    // added to the calculated accelerometer biases; on boot up these registers
    // will hold non-zero values. In addition, bit 0 of the lower byte must be
    // preserved since it is used for temperature compensation calculations.
    // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    // Holds the factory accelerometer trim biases
    int32_t accel_bias_reg[3] = {0, 0, 0};

    // Read factory accelerometer trim values
    readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

    // Define mask for temperature compensation bit 0 of lower byte of
    // accelerometer bias registers
    uint32_t mask = 1uL;
    // Define array to hold mask bit for each accelerometer bias axis
    uint8_t mask_bit[3] = {0, 0, 0};

    for (ii = 0; ii < 3; ii++) {
        if ((accel_bias_reg[ii] & mask)) {
            // If temperature compensation bit is set, set mask_bit to 1
            mask_bit[ii] = 0x01;
        }
    }

    // Construct total accelerometer bias, including calculated average
    // accelerometer bias from above. Subtract calculated averaged
    // accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[0] -= (accel_bias[0]/8);
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    // Preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[1] = data[1] | mask_bit[0];
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1];
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2];

    // Apparently this is not working for the acceleration biases in the
    // MPU-9250
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers
    writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
    writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
    writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
    writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
    writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
    writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

    // Output scaled accelerometer biases for display in the main program
    dest2[0] = static_cast<float>(accel_bias[0]) / \
        static_cast<float>(accelsensitivity);
    dest2[1] = static_cast<float>(accel_bias[1]) / \
        static_cast<float>(accelsensitivity);
    dest2[2] = static_cast<float>(accel_bias[2]) / \
        static_cast<float>(accelsensitivity);
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values, ±14% or less
// is a pass
void MPU9250SelfTest(float * destination) {
    uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
    uint8_t selfTest[6];
    int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
    float factoryTrim[6];
    uint8_t FS = 0;

    // Set gyro sample rate to 1 kHz
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
    // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    writeByte(MPU9250_ADDRESS, CONFIG, 0x02);
    // Set full scale range for the gyro to 250 dps
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS << 3);
    // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02);
    // Set full scale range for the accelerometer to 2 g
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS << 3);

    // get average current values of gyro and acclerometer
    for (int ii = 0; ii < 200; ii++) {
        // Read the six raw data registers into rawData
        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
        // Turn the MSB and LSB into a signed 16-bit value
        aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
        gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
    }

    // Get average of 200 values and store as average current readings
    for (int ii =0; ii < 3; ii++) {
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

    // Configure the accelerometer for self-test
    // Enable self test on all three axes and set accelerometer range to +/- 2 g
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0);
    // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0);
    delay(25);  // Delay a while to let the device stabilize

    // get average self-test values of gyro and acclerometer
    for (int ii = 0; ii < 200; ii++) {
        // Read the six raw data registers into rawData
        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
        // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
        gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
        gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
    }

    // Get average of 200 values and store as average self-test readings
    for (int ii =0; ii < 3; ii++) {
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }

    // Configure the gyro and accelerometer for normal operation
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
    delay(25);  // Delay a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
    selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
    selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
    selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
    selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
    selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results


    // Retrieve factory self-test value from self-test code reads

    // FT[Xa] factory trim calculation
    factoryTrim[0] = static_cast<float>(2620/1 << FS) * \
        pow(1.01, (static_cast<float>(selfTest[0]) - 1.0));

    // FT[Ya] factory trim calculation
    factoryTrim[1] = static_cast<float>(2620/1 << FS) * \
        pow(1.01, (static_cast<float>(selfTest[1]) - 1.0));

    // FT[Za] factory trim calculation
    factoryTrim[2] = static_cast<float>(2620/1 << FS) * \
        pow(1.01, (static_cast<float>(selfTest[2]) - 1.0) );

    // FT[Xg] factory trim calculation
    factoryTrim[3] = static_cast<float>(2620/1 << FS) * \
        pow(1.01, (static_cast<float>(selfTest[3]) - 1.0));

    // FT[Yg] factory trim calculation
    factoryTrim[4] = static_cast<float>(2620/1 << FS) * \
        pow(1.01, (static_cast<float>(selfTest[4]) - 1.0));

    // FT[Zg] factory trim calculation
    factoryTrim[5] = static_cast<float>(2620/1 << FS) * \
        pow(1.01, (static_cast<float>(selfTest[5]) - 1.0));

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
    // of the Self-Test Response. To get percent, must multiply by 100
     for (int i = 0; i < 3; i++) {
        // accelerometer
        destination[i]   = 100.0*(static_cast<float>(aSTAvg[i] - aAvg[i])) / \
            factoryTrim[i] - 100.;
        // gyrometer
        destination[i+3] = 100.0*(static_cast<float>(gSTAvg[i] - gAvg[i])) / \
            factoryTrim[i+3] - 100.;
     }
}


// Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
    uint8_t data;
    // Initialize the Tx buffer
    Wire.beginTransmission(address);
    // Put slave register address in Tx buffer
    Wire.write(subAddress);
    // Send the Tx buffer, but send a restart to keep connection alive
    Wire.endTransmission(false);
    // Read one byte from slave register address
    Wire.requestFrom(address, (uint8_t) 1);
    // Fill Rx buffer with result
    data = Wire.read();
    return data;
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count,
               uint8_t * dest) {
    // Initialize the Tx buffer
    Wire.beginTransmission(address);
    // Put slave register address in Tx buffer
    Wire.write(subAddress);
    // Send the Tx buffer, but send a restart to keep connection alive
    Wire.endTransmission(false);
    uint8_t i = 0;
    // Read bytes from slave register address
    Wire.requestFrom(address, count);
    // Put read results in the Rx buffer
    while (Wire.available()) {
        dest[i++] = Wire.read();
    }
}








