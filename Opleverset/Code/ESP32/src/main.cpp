#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Using both cores
TaskHandle_t sensorHandle;
TaskHandle_t motorHandle;
void sensor(void * pvParameters);
void motor(void * pvParameters);


MPU6050 mpu;

#define OUTPUT_READABLE_QUATERNION

#define LED_PIN 2 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define MOTOR1_PIN 26
#define MOTOR2_PIN 27
#define MOTOR3_PIN 14
#define MOTOR4_PIN 12

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
    #endif

    Serial.begin(115200);

    mpu.initialize();

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

    xTaskCreatePinnedToCore(sensor, "Sensor", 10000, NULL, 1, &sensorHandle, 0);
    xTaskCreatePinnedToCore(motor, "Motor", 10000, NULL, 1, &motorHandle, 1);
    
}


void sensor(void * pvParameters){
    Serial.print("Sensor running on core: ");
    Serial.println(xPortGetCoreID());
    for(;;){
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
            vTaskDelay(33);
        }   
    }
}


void motor(void * pvParameters){
    Serial.print("Motor running on core: ");
    Serial.println(xPortGetCoreID());
    for(;;){
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
        vTaskDelay(33);
    }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
}
