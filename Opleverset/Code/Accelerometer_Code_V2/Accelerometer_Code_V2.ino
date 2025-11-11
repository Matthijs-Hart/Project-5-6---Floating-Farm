//Tutorial: https://www.youtube.com/watch?v=L4WfHT_58Dg 
//Github Library Unity3D X arduino: https://github.com/jrowberg/i2cdevlib

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_QUATERNION

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define MOTOR1_PIN 4
#define MOTOR2_PIN 5
#define MOTOR3_PIN 6
#define MOTOR4_PIN 7

bool motor1Status = false;
bool motor2Status = false;
bool motor3Status = false;
bool motor4Status = false;

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; 
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);

    mpu.initialize();

    pinMode(INTERRUPT_PIN, INPUT);

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);

        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(MOTOR1_PIN, INPUT_PULLUP); // input for testing using buttons
    pinMode(MOTOR2_PIN, INPUT_PULLUP); // input for testing using buttons
    pinMode(MOTOR3_PIN, INPUT_PULLUP); // input for testing using buttons
    pinMode(MOTOR4_PIN, INPUT_PULLUP); // input for testing using buttons
    
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("ROT:");
            Serial.print(q.x);
            Serial.print(",");
            Serial.print(q.y);
            Serial.print(",");
            Serial.println(q.w);
        #endif
  
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        //Unity needs a delay to read data
        delay(33);
    }

    if (digitalRead(MOTOR1_PIN) != motor1Status){
        motor1Status = !motor1Status;
        Serial.print("MOTOR1:");
        Serial.println(!motor1Status);
    } else if (digitalRead(MOTOR2_PIN) != motor2Status){
        motor2Status = !motor2Status;
        Serial.print("MOTOR2:");
        Serial.println(!motor2Status);
    } else if (digitalRead(MOTOR3_PIN) != motor3Status){
        motor3Status = !motor3Status;
        Serial.print("MOTOR3:");
        Serial.println(!motor3Status);
    } else if (digitalRead(MOTOR4_PIN) != motor4Status){
        motor4Status = !motor4Status;
        Serial.print("MOTOR4:");
        Serial.println(!motor4Status);
    } 
}
