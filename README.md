a# SmartDisc
School project for Haaga Helia. 
  
In this project we are creating a smart device that will be attached to a disc golf disc. The device will have a buzzer to help find the disc if it is thrown in to the bush. It will also have sensors for acceleration and gyro, so we can get the top speed and rotation speed of the disc. Gyro sensor will also be used to get the angle of the disk.
  
The idea is that the player can use these information to help better their play.


## Project Plan

- Assignment for Haaga-Helia's ICT-Infrastructure project.
- Purpose of our project is to build a smart-frisbee using ESP-32 dev board, it's chips and open data.

## Development Tools

- Project developed using ArduinoIDE
- Dev board used: ESP32-WROOM  
<img src="/Pictures/ESP32.jpg" width= "300" height="280">
- Chips used: MPU-6050 (3 access gyro & accelometer)  
<img src="/Pictures/Gyro.jpg" width= "300" height="280">
<sub>**Further development aims to add features and integrate the chips to the golf disc**</sub>

# SmartDisc
  
- In this project we are creating a smart device that will be attached to a disc golf disc.
- The device will have a buzzer to help find the disc if it is thrown in to the bush.
- It will also have sensors for acceleration and gyro, so we can get the top speed and rotation speed of the disc. Gyro sensor will also be used to get the angle of the disk.
- The idea is that the player can use these information to help better their play.

## User Stories

- As a aspiring frisbee golf player I want to get data from from my throws to improve my game.
- As a player I want to easily find my lost disc using the inbuilt speaker that pinpoint its location.

## Target Audience

- The target audience for SmartDisc are frisbee golf professionals, enthusiasts and recreational playet that want to imnprove their game and easily find their lost discs

## ArduinoIDE
### Linux
When we first tried uploading code to the ESP32 we got an error code:
```
A fatal error occurred: Could not open /dev/ttyUSB0, the port doesn't exist
```
this was fixed with giving right permissions to the port
```
sudo chmod a+rw /dev/ttyUSB0
```

## Tests

Verifying that the chip works by uploading a basic code block to it using Arduino IDE with success.

```
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
}

void loop() {
  // Send "Hello, World!" over serial
  Serial.println("Hello, World!");
  delay(1000); // Wait for a second
}
```
![image](/Pictures/HelloWorld_test.png)

We also tested that the wifi works on the ESP32.  
![image](/Pictures/Wifi_test.png)

## Acceleration and Gyro
First we connected the Acceleration/Gyro sensor to the ESP32.
![image](https://i0.wp.com/randomnerdtutorials.com/wp-content/uploads/2020/12/MPU6050_ESP32_Wiring-Schematic-Diagram.png?w=726&quality=100&strip=all&ssl=1)

Installed nessessaru libraries (Adafruit MPU6050) for the MPU6050
Then we tested that the sensor works with example code.  
<details>
<summary>The code was found in **file** --> **Examples** --> **Adafruit MPU6050** --> **basic_readings**</summary>
<br>
// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");
^
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}
</details>
