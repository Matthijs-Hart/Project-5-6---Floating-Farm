#if defined(ESP32)
  #include <WiFiMulti.h>
  WiFiMulti wifiMulti;
  #define DEVICE "ESP32"
#elif defined(ESP8266)
  #include <ESP8266WiFiMulti.h>
  ESP8266WiFiMulti wifiMulti;
  #define DEVICE "ESP8266"
#endif

// MPU6050 libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

// WiFi AP SSID
#define WIFI_SSID "kyrill"
// WiFi password
#define WIFI_PASSWORD "12345678"

#define INFLUXDB_URL "https://floatingfarm.kyrillk.nl:443"
#define INFLUXDB_TOKEN "0FEhLDdgpzUmI-nHQolnEexb51tNxlUF29wYJlRzA8s7tQ819lPT1OB0tceaL8TV7bMCKLGlesjucOFPWYbZXg=="
#define INFLUXDB_ORG "09d2d9094596dc00"
#define INFLUXDB_BUCKET "arduino test"

// Time zone info
#define TZ_INFO "UTC2"

// Declare InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Declare Data point
Point sensor("motion_data");

// Initialize MPU6050
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
const unsigned long intervalMs = 20000; // re-zero every 20 sec

// Gyro bias
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

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

void setup() {
  Serial.begin(115200);

  // Setup wifi
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

  delay(50);

  Serial.println("Connecting to wifi");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  // Accurate time is necessary for certificate validation and writing in batches
  // We use the NTP servers in your area as provided by: https://www.pool.ntp.org/zone/
  // Syncing progress and the time will be printed to Serial.
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  // IMPORTANT - You must set the device as insecure before making the connection
  client.setInsecure(true);
  
  // Test the connection to InfluxDB before doing anything else
  Serial.println("Testing InfluxDB connection...");
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  // Add tags to the data point
  sensor.addTag("device", DEVICE);
  sensor.addTag("SSID", WiFi.SSID());

  // Initialize MPU6050
  Wire.begin();
  
  // Try to initialize MPU6050
  int attempts = 0;
  while (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire, 0)) {
    Serial.println("Failed to find MPU6050 chip");
    delay(500);
    attempts++;
    if (attempts > 10) {
      Serial.println("Could not find MPU6050. Check wiring!");
      break;
    }
  }
  
  if (attempts <= 10) {
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
}

void loop() {
  float rollRelative = 0, pitchRelative = 0, yawRelative = 0;
  
  if (mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire, 0)) {
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
    rollRelative  = angleRoll  - rollOffset;
    pitchRelative = anglePitch - pitchOffset;
    yawRelative   = angleYaw; // no water ref for yaw

    // Print
    Serial.print(rollRelative, 2); Serial.print("\t");
    Serial.println(pitchRelative, 2);
    
    // Re-zero periodically
    if (millis() - lastZeroTime >= intervalMs) {
      setWaterReference();
      lastZeroTime = millis();
    }
  } else {
    Serial.println("MPU6050 disconnected");
  }

  // Clear fields for reusing the point. Tags will remain the same as set above.
  sensor.clearFields();
    
  // Store measured values into point
  sensor.addField("rssi", WiFi.RSSI());
  sensor.addField("ROT-X", rollRelative);
  sensor.addField("ROT-Y", pitchRelative);
  sensor.addField("ROT-Z", yawRelative);
  
  // Print what are we exactly writing
  Serial.print("Writing: ");
  Serial.println(sensor.toLineProtocol());

  // Check WiFi connection and reconnect if needed
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("Wifi connection lost");
    delay(1000);
  }

  // Write point
  if (!client.writePoint(sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
    Serial.print("HTTP status code: ");
    Serial.println(client.getLastStatusCode());
  }

  Serial.println("Waiting 1 second");
  delay(1000);
}