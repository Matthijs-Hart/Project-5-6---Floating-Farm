#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <PID_v1.h>


#define LED_PIN  2
#define MOTOR1_PIN 12
#define MOTOR2_PIN 13
#define MOTOR3_PIN 14
#define MOTOR4_PIN 27

Adafruit_MPU6050 mpu;

float angleRoll, anglePitch, angleYaw = 0.0; // absolute orientation

double rollOffset, pitchOffset = 0.0; // offsets used for water reference

const double alpha = 0.98; // rely more on gyro or 1 - alpha erly on accelerometer

float degreeMarge = 5.0; // reacts after a difference of 2 degrees or more

//Multicore
TaskHandle_t sensorHandle;
TaskHandle_t motorHandle;

// TIMING
unsigned long lastMicros = 0;
unsigned long lastZeroTime = 0;
const unsigned long intervalSetWaterReferenceMs = 60000; // elke 60 seconden wordt de waterpas opnieuw berekend

float gyroBiasX, gyroBiasY, gyroBiasZ = 0;

bool blinkState = false;

// Motor structure
struct Motor {
    int pin;
    int id;
    bool status;
};

enum SystemState {
    idle,
    activePositive,
    activeNegative,
};

struct System
{
    Motor motor1;
    Motor motor2;
    SystemState state;
};

// Motor objects
Motor motor1 = {MOTOR1_PIN, 1, false};
Motor motor2 = {MOTOR2_PIN, 2, false};
Motor motor3 = {MOTOR3_PIN, 3, false};
Motor motor4 = {MOTOR4_PIN, 4, false};

System system1 = {
    motor1,
    motor2,
    idle
};
System system2 = {
    motor3,
    motor4,
    idle
};

sensors_event_t a, g, temp;

// declare functions
void eulerToQuaternion(float rollDeg, float pitchDeg, float yawDeg, float &x, float &y, float &z, float &w);
void setWaterReference();
void handleSensorData(void * pvParameters);
void handleMotorData(void * pvParameters);
void SystemRun(System &system);
void motorPrintStatus(Motor &motor);

void setup(){
    Serial.begin(115200);
    while (!Serial) {delay(10);}

    Wire.begin();
    while (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire, 0)) {
        Serial.println("Failed to find MPU6050 chip");
        delay(200);
    }
    Serial.println("MPU6050 Found!");

    pinMode(LED_PIN, OUTPUT);
    pinMode(MOTOR1_PIN, OUTPUT); // input for testing using buttons
    pinMode(MOTOR2_PIN, OUTPUT); // input for testing using buttons
    pinMode(MOTOR3_PIN, OUTPUT); // input for testing using buttons
    pinMode(MOTOR4_PIN, OUTPUT); // input for testing using buttons

    xTaskCreatePinnedToCore(handleSensorData, "Sensor", 10000, NULL, 1, &sensorHandle, 0);
    xTaskCreatePinnedToCore(handleMotorData, "Motor", 10000, NULL, 1, &motorHandle, 1);

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(200);

    // Calibratie van de sensor
    const int calSamples = 200;
    double sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < calSamples; i++) {
        mpu.getEvent(&a, &g, &temp);
        sx += g.gyro.x;
        sy += g.gyro.y;
        sz += g.gyro.z;
        delay(5);
    }
    gyroBiasX = sx / calSamples;
    gyroBiasY = sy / calSamples;
    gyroBiasZ = sz / calSamples;

    lastMicros = micros();
    lastZeroTime = millis();
}

void loop(){
}

void handleMotorData(void * pvParameters){
    for(;;){
        // print both angles (use printf so we print both values)
        // Serial.printf("angles: roll=%.2f pitch=%.2f\n", angleRoll, anglePitch);

        // use an exclusive if/else-if chain so only one state is chosen
        if (angleRoll > degreeMarge) {
            system1.state = activePositive;
        } else if (angleRoll < -degreeMarge) {
            system1.state = activeNegative;
        } else if (angleRoll > -degreeMarge && angleRoll < degreeMarge) {
            // centred within the dead-zone -> idle
            system1.state = idle;
        }
        if (anglePitch > degreeMarge) {
            system2.state = activePositive;
        } else if (anglePitch < -degreeMarge) {
            system2.state = activeNegative;
        } else if (anglePitch > -degreeMarge && anglePitch < degreeMarge) {
            system2.state = idle;
        }
        SystemRun(system1);
        SystemRun(system2);
        vTaskDelay(33 / portTICK_PERIOD_MS);
    }
}

// Helper: compute roll and pitch (in degrees) from accelerometer data
void computeAccelAngles(float &roll, float &pitch) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    roll  = atan2(ay, az) * RAD_TO_DEG;
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
}

void handleSensorData(void *pvParameters) {
    for (;;) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Time step
        unsigned long now = micros();
        float dt = (now - lastMicros) / 1e6f;
        lastMicros = now;

        // Gyro (rad/s → deg/s) with bias removal
        float gx_deg = (g.gyro.x - gyroBiasX) * RAD_TO_DEG;
        float gy_deg = (g.gyro.y - gyroBiasY) * RAD_TO_DEG;
        float gz_deg = (g.gyro.z - gyroBiasZ) * RAD_TO_DEG;

        // Integrate gyro
        angleRoll  += gx_deg * dt;
        anglePitch += gy_deg * dt;
        angleYaw   += gz_deg * dt;

        // Complementary filter using accelerometer data
        float accelRoll, accelPitch;
        computeAccelAngles(accelRoll, accelPitch);
        angleRoll  = alpha * angleRoll  + (1.0 - alpha) * accelRoll;
        anglePitch = alpha * anglePitch + (1.0 - alpha) * accelPitch;

        // Apply water reference offsets
        float rollRelative  = angleRoll  - rollOffset;
        float pitchRelative = anglePitch - pitchOffset;
        float yawRelative   = angleYaw; // no water ref for yaw

        float qx, qy, qz, qw;
        eulerToQuaternion(rollRelative, pitchRelative, yawRelative, qx, qy, qz, qw);

        Serial.print("ROT:");
        Serial.print(qx, 4);
        Serial.print(",");
        Serial.print(qy, 4);
        Serial.print(",");
        Serial.println(qw, 4);

        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        vTaskDelay(33 / portTICK_PERIOD_MS);
    }
}

void setWaterReference() {
    float accelRoll, accelPitch;
    computeAccelAngles(accelRoll, accelPitch);

    // Force filter state to match gravity
    angleRoll  = accelRoll;
    anglePitch = accelPitch;
    angleYaw = 0;

    Serial.println("Re-aligned to gravity (water reference).");
}



void eulerToQuaternion(float rollDeg, float pitchDeg, float yawDeg, float &x, float &y, float &z, float &w) {
  float roll  = rollDeg  * DEG_TO_RAD;
  float pitch = pitchDeg * DEG_TO_RAD;
  float yaw   = yawDeg   * DEG_TO_RAD;

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  w = cr * cp * cy + sr * sp * sy;
  x = sr * cp * cy - cr * sp * sy;
  y = cr * sp * cy + sr * cp * sy;
  z = cr * cp * sy - sr * sp * cy;
}

void SystemRun(System &system) {
    if (system.state == activePositive) {
        system.motor1.status = true;
        system.motor2.status = false;
        digitalWrite(system.motor1.pin, HIGH);
        digitalWrite(system.motor2.pin, LOW);
    } else if (system.state == activeNegative) {
        system.motor1.status = false;
        system.motor2.status = true;
        digitalWrite(system.motor1.pin, LOW);
        digitalWrite(system.motor2.pin, HIGH);
    } else if (system.state == idle) {
        system.motor1.status = false;
        system.motor2.status = false;
        digitalWrite(system.motor1.pin, LOW);
        digitalWrite(system.motor2.pin, LOW);
    } else {
        Serial.println("Unknown system state!");
    }
    motorPrintStatus(system.motor1);
    motorPrintStatus(system.motor2);
}

void motorPrintStatus(Motor &motor) {
    Serial.println("MOTOR" + String(motor.id) + ":" + motor.status);
}