# README - MPU6050 Project

## Gebruikte Libraries

Voor dit project zijn de volgende Arduino-libraries gebruikt:

```cpp
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
```

### Library-versies

* **Adafruit MPU6050**: `adafruit/Adafruit MPU6050@^2.2.6`
* **Adafruit Sensor**: mee geïnstalleerd via de MPU6050 dependency (versie afhankelijk van Arduino Library Manager)
* **Wire (I2C)**: standaard Arduino-library

## Outputformaten voor Serial-communicatie

Het dashboard verwacht specifieke formats voor de seriële output. Zorg dat alle printstatements exact zo geschreven worden:

### Rotatiewaarden (Quaternion)

```cpp
Serial.print("ROT:");
Serial.print(qx, 4);
Serial.print(",");
Serial.print(qy, 4);
Serial.print(",");
Serial.println(qw, 4);
```

Dit format zorgt ervoor dat het dashboard de quaternion-rotatie correct kan uitlezen.

### Motorstatus

```cpp
Serial.println("MOTOR" + String(motor.id) + ":" + motor.status);
```

* **motor.id** = ID van de motor
* **motor.status** = statuswaarde die wordt uitgelezen

Deze string wordt door het dashboard gebruikt om motorinformatie te verwerken.

---

Zorg dat deze formats exact worden aangehouden, zodat het dashboard goed kan communiceren met de Arduino.
