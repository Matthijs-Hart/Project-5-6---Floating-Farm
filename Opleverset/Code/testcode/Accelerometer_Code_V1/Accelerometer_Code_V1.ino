#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Absolute orientation (before offsets)
float angleRoll  = 0.0;
float anglePitch = 0.0;
float angleYaw   = 0.0;

// Offsets for "water reference"
float rollOffset  = 0.0;
float pitchOffset = 0.0;

// Complementary filter constant
const float alpha = 0.98;

// Timing
unsigned long lastMicros = 0;
unsigned long lastZeroTime = 0;
const unsigned long intervalMs = 20000; // re-zero every 60 sec

// Gyro bias
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Wire.begin();
  while (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire, 0)) {
    Serial.println("Failed to find MPU6050 chip");
    delay(200);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(200);

  // --- Gyro bias calibration ---
  const int calSamples = 200;
  double sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < calSamples; i++) {
    sensors_event_t a, g, temp;
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

  // Initial water reference
  setWaterReference();
}

void loop() {
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

  // Accelerometer roll/pitch
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  float accelRoll  = atan2(ay, az) * RAD_TO_DEG;
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // Complementary filter
  angleRoll  = alpha * angleRoll  + (1.0 - alpha) * accelRoll;
  anglePitch = alpha * anglePitch + (1.0 - alpha) * accelPitch;

  // Apply water reference offsets
  float rollRelative  = angleRoll  - rollOffset;
  float pitchRelative = anglePitch - pitchOffset;
  float yawRelative   = angleYaw; // no water ref for yaw

  // Print
  Serial.print(rollRelative, 2); Serial.print("\t");
  Serial.println(pitchRelative, 2);// Serial.print("\t");
  //Serial.println(yawRelative, 2);

  // Re-zero periodically
  if (millis() - lastZeroTime >= intervalMs) {
    setWaterReference();
    lastZeroTime = millis();
  }

  delay(5);
}

void setWaterReference() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  // Compute absolute angles from gravity
  float accelRoll  = atan2(ay, az) * RAD_TO_DEG;
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // Force filter state to match gravity
  angleRoll  = accelRoll;
  anglePitch = accelPitch;
  angleYaw   = 0; // can't correct yaw without magnetometer

  Serial.println("Re-aligned to gravity (water reference).");
}

