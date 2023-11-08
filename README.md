# SmartDisc

School project for Haaga Helia.

In this project we are creating a smart device that will be attached to a disc golf disc. The device will have a buzzer to help find the disc if it is thrown in to the bush. It will also have sensors for acceleration and gyro, so we can get the top speed and rotation speed of the disc. Gyro sensor will also be used to get the angle of the disk.

The idea is that the player can use these information to help better their play.

## Project Plan

- Assignment for Haaga-Helia's ICT-Infrastructure project.
- Purpose of our project is to build a smart-frisbee using ESP-32 dev board, it's chips and open data.

## Development Tools

- Project developed using [ArduinoIDE](https://www.arduino.cc/en/software)
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

# Tests

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

## Piezo Buzzer (HW-508)

We also tested that the Piezo Buzzer works.
![buzz](/Pictures/buzz.mp4)

<details>
<summary>Code</summary>
<br>
<pre>
  #include <Arduino.h>

// Define the buzzer pin
const int buzzerPin = 16; // Define the buzzer pin

void setup() {
// Set the buzzer pin as output
pinMode(buzzerPin, OUTPUT);
}

void loop() {
// Turn on the buzzer
digitalWrite(buzzerPin, HIGH);

// Delay for 1 second
delay(1000);

// Turn off the buzzer
digitalWrite(buzzerPin, LOW);

// Delay for 1 second
delay(1000);
}

</pre>
</details>

## Acceleration and Gyro (MPU6050)

First we connected the Acceleration/Gyro sensor to the ESP32.
![image](https://i0.wp.com/randomnerdtutorials.com/wp-content/uploads/2020/12/MPU6050_ESP32_Wiring-Schematic-Diagram.png?w=726&quality=100&strip=all&ssl=1)

Installed nessessaru libraries (Adafruit MPU6050) for the MPU6050
Then we tested that the sensor works with example code.

The code was found in `file` --> `Examples` --> `Adafruit MPU6050` --> `basic_readings`

<details>
<summary>Code</summary>
<br>  
<pre>
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

/_ Get new sensor events with the readings _/
sensors_event_t a, g, temp;
mpu.getEvent(&a, &g, &temp);

/_ Print out the values _/
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

</pre>
</details>

Whit this we get reading from the sensor but the readings aren't calibrated.  
![image](/Pictures/Accelero_test.png)
So we needed to calibrate the sensor. We got the information on hot to do it [here](https://www.instructables.com/MPU6050-Setup-and-Calibration-Guide/)  
We calibrated the sensor with the example code in `files` --> `examples` --> `MPU6050` --> `IMU_Zero`

<details>
<summary>Code</summary>
<br>
<pre>
// MPU6050 offset-finder, based on Jeff Rowberg's MPU6050_RAW
// 2016-10-19 by Robert R. Fenichel (bob@fenichel.net)

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
// 2019-07-11 - added PID offset generation at begninning Generates first offsets
// - in @ 6 seconds and completes with 4 more sets @ 10 seconds
// - then continues with origional 2016 calibration code.
// 2016-11-25 - added delays to reduce sampling rate to ~200 Hz
// added temporizing printing during long computations
// 2016-10-25 - requires inequality (Low < Target, High > Target) during expansion
// dynamic speed change when closing in
// 2016-10-22 - cosmetic changes
// 2016-10-19 - initial release of IMU_Zero
// 2013-05-08 - added multiple output formats
// - added seamless Fastwire support
// 2011-10-07 - initial release of MPU6050_RAW

/\* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

If an MPU6050
_ is an ideal member of its tribe,
_ is properly warmed up,
_ is at rest in a neutral position,
_ is in a location where the pull of gravity is exactly 1g, and \* has been loaded with the best possible offsets,
then it will report 0 for all accelerations and displacements, except for
Z acceleration, for which it will report 16384 (that is, 2^14). Your device
probably won't do quite this well, but good offsets will all get the baseline
outputs close to these target values.

Put the MPU6050 on a flat and horizontal surface, and leave it operating for
5-10 minutes so its temperature gets stabilized.

Run this program. A "----- done -----" line will indicate that it has done its best.
With the current accuracy-related constants (NFast = 1000, NSlow = 10000), it will take
a few minutes to get there.

Along the way, it will generate a dozen or so lines of output, showing that for each
of the 6 desired offsets, it is
_ first, trying to find two estimates, one too low and one too high, and
_ then, closing in until the bracket can't be made smaller.

The line just above the "done" line will look something like
[567,567] --> [-1,2] [-2223,-2223] --> [0,1] [1131,1132] --> [16374,16404] [155,156] --> [-1,1] [-25,-24] --> [0,3] [5,6] --> [0,4]
As will have been shown in interspersed header lines, the six groups making up this
line describe the optimum offsets for the X acceleration, Y acceleration, Z acceleration,
X gyro, Y gyro, and Z gyro, respectively. In the sample shown just above, the trial showed
that +567 was the best offset for the X acceleration, -2223 was best for Y acceleration,
and so on.

# The need for the delay between readings (usDelay) was brought to my attention by Nikolaus Doppelhammer.

\*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA = ',';
const char BLANK = ' ';
const char PERIOD = '.';

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int usDelay = 3150; // empirical, to hold sampling to 200 Hz
const int NFast = 1000; // the bigger, the better (but slower)
const int NSlow = 10000; // ..
const int LinesBetweenHeaders = 5;
int LowValue[6];
int HighValue[6];
int Smoothed[6];
int LowOffset[6];
int HighOffset[6];
int Target[6];
int LinesOut;
int N;

void ForceHeader()
{ LinesOut = 99; }

void GetSmoothed()
{ int16_t RawValue[6];
int i;
long Sums[6];
for (i = iAx; i <= iGz; i++)
{ Sums[i] = 0; }
// unsigned long Start = micros();

    for (i = 1; i <= N; i++)
      { // get sums
        accelgyro.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz],
                             &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
        if ((i % 500) == 0)
          Serial.print(PERIOD);
        delayMicroseconds(usDelay);
        for (int j = iAx; j <= iGz; j++)
          Sums[j] = Sums[j] + RawValue[j];
      } // get sums

// unsigned long usForN = micros() - Start;
// Serial.print(" reading at ");
// Serial.print(1000000/((usForN+N/2)/N));
// Serial.println(" Hz");
for (i = iAx; i <= iGz; i++)
{ Smoothed[i] = (Sums[i] + N/2) / N ; }
} // GetSmoothed

void Initialize()
{
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
Fastwire::setup(400, true);
#endif

    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    Serial.println("PID tuning Each Dot = 100 readings");

/_A tidbit on how PID (PI actually) tuning works.
When we change the offset in the MPU6050 we can get instant results. This allows us to use Proportional and
integral of the PID to discover the ideal offsets. Integral is the key to discovering these offsets, Integral
uses the error from set-point (set-point is zero), it takes a fraction of this error (error _ ki) and adds it
to the integral value. Each reading narrows the error down to the desired offset. The greater the error from
set-point, the more we adjust the integral value. The proportional does its part by hiding the noise from the
integral math. The Derivative is not used because of the noise and because the sensor is stationary. With the
noise removed the integral value lands on a solid offset after just 600 readings. At the end of each set of 100
readings, the integral value is used for the actual offsets and the last proportional reading is ignored due to
the fact it reacts to any noise.
\*/
accelgyro.CalibrateAccel(6);
accelgyro.CalibrateGyro(6);
Serial.println("\nat 600 Readings");
accelgyro.PrintActiveOffsets();
Serial.println();
accelgyro.CalibrateAccel(1);
accelgyro.CalibrateGyro(1);
Serial.println("700 Total Readings");
accelgyro.PrintActiveOffsets();
Serial.println();
accelgyro.CalibrateAccel(1);
accelgyro.CalibrateGyro(1);
Serial.println("800 Total Readings");
accelgyro.PrintActiveOffsets();
Serial.println();
accelgyro.CalibrateAccel(1);
accelgyro.CalibrateGyro(1);
Serial.println("900 Total Readings");
accelgyro.PrintActiveOffsets();
Serial.println();  
 accelgyro.CalibrateAccel(1);
accelgyro.CalibrateGyro(1);
Serial.println("1000 Total Readings");
accelgyro.PrintActiveOffsets();
Serial.println("\n\n Any of the above offsets will work nice \n\n Lets proof the PID tuning using another method:");
} // Initialize

void SetOffsets(int TheOffsets[6])
{ accelgyro.setXAccelOffset(TheOffsets [iAx]);
accelgyro.setYAccelOffset(TheOffsets [iAy]);
accelgyro.setZAccelOffset(TheOffsets [iAz]);
accelgyro.setXGyroOffset (TheOffsets [iGx]);
accelgyro.setYGyroOffset (TheOffsets [iGy]);
accelgyro.setZGyroOffset (TheOffsets [iGz]);
} // SetOffsets

void ShowProgress()
{ if (LinesOut >= LinesBetweenHeaders)
{ // show header
Serial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
LinesOut = 0;
} // show header
Serial.print(BLANK);
for (int i = iAx; i <= iGz; i++)
{ Serial.print(LBRACKET);
Serial.print(LowOffset[i]),
Serial.print(COMMA);
Serial.print(HighOffset[i]);
Serial.print("] --> [");
Serial.print(LowValue[i]);
Serial.print(COMMA);
Serial.print(HighValue[i]);
if (i == iGz)
{ Serial.println(RBRACKET); }
else
{ Serial.print("]\t"); }
}
LinesOut++;
} // ShowProgress

void PullBracketsIn()
{ boolean AllBracketsNarrow;
boolean StillWorking;
int NewOffset[6];

    Serial.println("\nclosing in:");
    AllBracketsNarrow = false;
    ForceHeader();
    StillWorking = true;
    while (StillWorking)
      { StillWorking = false;
        if (AllBracketsNarrow && (N == NFast))
          { SetAveraging(NSlow); }
        else
          { AllBracketsNarrow = true; }// tentative
        for (int i = iAx; i <= iGz; i++)
          { if (HighOffset[i] <= (LowOffset[i]+1))
              { NewOffset[i] = LowOffset[i]; }
            else
              { // binary search
                StillWorking = true;
                NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
                if (HighOffset[i] > (LowOffset[i] + 10))
                  { AllBracketsNarrow = false; }
              } // binary search
          }
        SetOffsets(NewOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // closing in
            if (Smoothed[i] > Target[i])
              { // use lower half
                HighOffset[i] = NewOffset[i];
                HighValue[i] = Smoothed[i];
              } // use lower half
            else
              { // use upper half
                LowOffset[i] = NewOffset[i];
                LowValue[i] = Smoothed[i];
              } // use upper half
          } // closing in
        ShowProgress();
      } // still working

} // PullBracketsIn

void PullBracketsOut()
{ boolean Done = false;
int NextLowOffset[6];
int NextHighOffset[6];

    Serial.println("expanding:");
    ForceHeader();

    while (!Done)
      { Done = true;
        SetOffsets(LowOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // got low values
            LowValue[i] = Smoothed[i];
            if (LowValue[i] >= Target[i])
              { Done = false;
                NextLowOffset[i] = LowOffset[i] - 1000;
              }
            else
              { NextLowOffset[i] = LowOffset[i]; }
          } // got low values

        SetOffsets(HighOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // got high values
            HighValue[i] = Smoothed[i];
            if (HighValue[i] <= Target[i])
              { Done = false;
                NextHighOffset[i] = HighOffset[i] + 1000;
              }
            else
              { NextHighOffset[i] = HighOffset[i]; }
          } // got high values
        ShowProgress();
        for (int i = iAx; i <= iGz; i++)
          { LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
            HighOffset[i] = NextHighOffset[i]; // ..
          }
     } // keep going

} // PullBracketsOut

void SetAveraging(int NewN)
{ N = NewN;
Serial.print("averaging ");
Serial.print(N);
Serial.println(" readings each time");
} // SetAveraging

void setup()
{ Initialize();
for (int i = iAx; i <= iGz; i++)
{ // set targets and initial guesses
Target[i] = 0; // must fix for ZAccel
HighOffset[i] = 0;
LowOffset[i] = 0;
} // set targets and initial guesses
Target[iAz] = 16384;
SetAveraging(NFast);

    PullBracketsOut();
    PullBracketsIn();

    Serial.println("-------------- done --------------");

} // setup

void loop()
{
} // loop

</pre>
</details>

With this we get the offset values.  
For us those are:

```bash
XAccel			YAccel				ZAccel			XGyro			YGyro			ZGyro
[13,14] --> [0,17]	[557,558] --> [-6,13]	[711,712] --> [16369,16389]	[-8,-7] --> [-2,2]	[5,6] --> [-1,2]	[13,14] --> [-2,1]
```

In our case this method is obsolete.  
We managed to find suitable method for calibrating gyro. We needed to install Adafruit Sensor Lab.  
Then we got the code from `Files` --> `Examples` --> `Adafruti Sensor Lab` --> `Calibration` --> `gyro_zerorate_simplecal`

<details>
<summary>Code</summary>
<br>
<pre>
/***************************************************************************
  This is an example for the Adafruit SensorLab library
  It will look for a supported gyroscope and collect
  rad/s data for a few seconds to calcualte the zero rate
  calibration offsets
  
  Written by Limor Fried for Adafruit Industries.
 ***************************************************************************/

#include <Adafruit_SensorLab.h>
Adafruit_SensorLab lab;

#define NUMBER_SAMPLES 500

Adafruit_Sensor \*gyro;
sensors_event_t event;

float min_x, max_x, mid_x;
float min_y, max_y, mid_y;
float min_z, max_z, mid_z;

void setup(void) {
Serial.begin(115200);
while (!Serial) delay(10); // will pause Zero, Leonardo, etc until serial console opens

Serial.println(F("Sensor Lab - Gyroscope Calibration!"));
lab.begin();

Serial.println("Looking for a gyro");
gyro = lab.getGyroscope();
if (! gyro) {
Serial.println(F("Could not find a gyro, check wiring!"));
while(1) delay(10);
}
gyro->printSensorDetails();
delay(100);

gyro->getEvent(&event);
min_x = max_x = event.gyro.x;
min_y = max_y = event.gyro.y;
min_z = max_z = event.gyro.z;
delay(10);

Serial.println(F("Place gyro on flat, stable surface!"));

Serial.print(F("Fetching samples in 3..."));
delay(1000);
Serial.print("2...");
delay(1000);
Serial.print("1...");
delay(1000);
Serial.println("NOW!");

float x, y, z;
for (uint16_t sample = 0; sample < NUMBER_SAMPLES; sample++) {
gyro->getEvent(&event);
x = event.gyro.x;
y = event.gyro.y;
z = event.gyro.z;
Serial.print(F("Gyro: ("));
Serial.print(x); Serial.print(", ");
Serial.print(y); Serial.print(", ");
Serial.print(z); Serial.print(")");

    min_x = min(min_x, x);
    min_y = min(min_y, y);
    min_z = min(min_z, z);

    max_x = max(max_x, x);
    max_y = max(max_y, y);
    max_z = max(max_z, z);

    mid_x = (max_x + min_x) / 2;
    mid_y = (max_y + min_y) / 2;
    mid_z = (max_z + min_z) / 2;

    Serial.print(F(" Zero rate offset: ("));
    Serial.print(mid_x, 4); Serial.print(", ");
    Serial.print(mid_y, 4); Serial.print(", ");
    Serial.print(mid_z, 4); Serial.print(")");

    Serial.print(F(" rad/s noise: ("));
    Serial.print(max_x - min_x, 3); Serial.print(", ");
    Serial.print(max_y - min_y, 3); Serial.print(", ");
    Serial.print(max_z - min_z, 3); Serial.println(")");
    delay(10);

}
Serial.println(F("\n\nFinal zero rate offset in radians/s: "));
Serial.print(mid_x, 4); Serial.print(", ");
Serial.print(mid_y, 4); Serial.print(", ");
Serial.println(mid_z, 4);
}

void loop() {
delay(10);
}

</pre>
</details>

This gave us offset values:

```bash
XGyro = 0.0057	YGyro = -0.0031	ZGyro = -0.0080
```

These values were added to the test code

```bash
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x - 0.0057);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y - (-0.0031));
  Serial.print(", Z: ");
  Serial.print(g.gyro.z - (0.0080));
  Serial.println(" rad/s");
```

And these gave us almost perfect 0 value to the gyros when stationary.
![image](/Pictures/gyroCali.png)

### Finalizing Accelerometer Calibration and Data Reading

This code uses the Adafruit_MPU6050 library to interface with an MPU6050 accelerometer (gyroscope) and perform calibration and data reading on an Arduino platform. The code performs the following actions:

1. Initializes the MPU6050 sensor and checks for its availability.
2. Performs calibration of the accelerometer, which includes the following steps:
   - Computes offset values for the accelerometer (average for x, y, and z axes).
   - Calculates calibration values (gain) for the accelerometer by tilting it in different directions and taking averages for each axis.
3. In the loop, it reads accelerometer data, applies calibration values (offset and gain), prints the x-axis acceleration value to the serial monitor, and introduces a 1-second delay.

This code helps obtain more accurate acceleration values from the MPU6050 sensor by calibrating its offset and gain values.

Below is the Arduino code embedded in a dropdown menu:

<details>
<summary>Code</summary>

```cpp
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;

float accelerometer_offset[3];
float accelerometer_gain[3];

void setup() {
  Serial.begin(115200);

  // Initialize the accelerometer
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);  // Halt the program if the sensor initialization fails
  }

  // Calibrate the accelerometer offset
  for (int i = 0; i < 3; i++) {
    accelerometer_offset[i] = 0.0f;

    // Take 100 accelerometer readings and average them
    for (int j = 0; j < 100; j++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);  // Pass in all three sensor_event_t objects

      accelerometer_offset[i] += a.acceleration.x;
    }

    accelerometer_offset[i] /= 100.0f;
  }

  // Calibrate the accelerometer gain
  for (int i = 0; i < 3; i++) {
    accelerometer_gain[i] = 1.0f;

    // Take 100 accelerometer readings while tilting the accelerometer in different directions
    // and average them
    for (int j = 0; j < 100; j++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);  // Pass in all three sensor_event_t objects

      accelerometer_gain[i] += abs(a.acceleration.x);
    }

    accelerometer_gain[i] /= 100.0f;
  }
}

void loop() {
  // Read the accelerometer data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  // Pass in all three sensor_event_t objects

  // Apply the offset and gain calibration
  a.acceleration.x -= accelerometer_offset[0];
  a.acceleration.x *= accelerometer_gain[0];

  // Print the accelerometer data to the serial monitor
  Serial.print("Accelerometer X: ");
  Serial.println(a.acceleration.x);

  // Delay for 1 second
  delay(1000);
}
```

</details>
The above code only calibrates the x axis and gives it's speed.
This code below calculates the magnitude of the acceleration vector using all three axes (x, y, and z) and applies calibration to provide the actual speed of the vector, displayed in the serial monitor.

<details>
<summary>Code</summary>

```cpp
#include <Adafruit_MPU6050.h>
#include <Math.h>

Adafruit_MPU6050 mpu;

float accelerometer_offset[3];
float accelerometer_gain[3];

void setup() {
  Serial.begin(115200);

  // Initialize the accelerometer
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);  // Halt the program if the sensor initialization fails
  }

  // Calibrate the accelerometer offset
  for (int i = 0; i < 3; i++) {
    accelerometer_offset[i] = 0.0f;

    // Take 100 accelerometer readings and average them
    for (int j = 0; j < 100; j++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);  // Pass in all three sensor_event_t objects

      accelerometer_offset[i] += a.acceleration[i];
    }

    accelerometer_offset[i] /= 100.0f;
  }

  // Calibrate the accelerometer gain
  for (int i = 0; i < 3; i++) {
    accelerometer_gain[i] = 1.0f;

    // Take 100 accelerometer readings while tilting the accelerometer in different directions
    // and average them
    for (int j = 0; j < 100; j++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);  // Pass in all three sensor_event_t objects

      accelerometer_gain[i] += abs(a.acceleration[i]);
    }

    accelerometer_gain[i] /= 100.0f;
  }
}

void loop() {
  // Read the accelerometer data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  // Pass in all three sensor_event_t objects

  // Calculate the acceleration vector magnitude
  float acceleration_magnitude = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));

  // Apply the offset and gain calibration to the magnitude
  acceleration_magnitude -= sqrt(pow(accelerometer_offset[0], 2) + pow(accelerometer_offset[1], 2) + pow(accelerometer_offset[2], 2));
  acceleration_magnitude *= sqrt(pow(accelerometer_gain[0], 2) + pow(accelerometer_gain[1], 2) + pow(accelerometer_gain[2], 2));

  // Print the acceleration vector magnitude to the serial monitor
  Serial.print("Acceleration Magnitude: ");
  Serial.println(acceleration_magnitude);

  // Delay for 1 second
  delay(1000);
}
```

</details>

## Bluetooth

Using [this](https://randomnerdtutorials.com/esp32-bluetooth-classic-arduino-ide/) we tried to make bluetooth work and possibly control ESP32 through it.  
We uploaded the example code from `File` --> `Examples` --> `BluetoothSerial` --> `SerialtoSerialIBT`

<details>
<summary>Code</summary>
<br>
<pre>
//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include "BluetoothSerial.h"

//#define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
const char \*pin = "1234"; // Change this to more secure PIN.

String device_name = "ESP32-BT-Slave";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

void setup() {
Serial.begin(115200);
SerialBT.begin(device_name); //Bluetooth device name
Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
//Serial.printf("The device with name \"%s\" and MAC address %s is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str(), SerialBT.getMacString()); // Use this after the MAC method is implemented
#ifdef USE_PIN
SerialBT.setPin(pin);
Serial.println("Using PIN");
#endif
}

void loop() {
if (Serial.available()) {
SerialBT.write(Serial.read());
}
if (SerialBT.available()) {
Serial.write(SerialBT.read());
}
delay(20);
}

</pre>
</details>

To turn on the bluetooth you press the ESP32 Enable button on the ESP32.

Connected the ESP32 to mobile phone with [Serial Bluetooth Terminal](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal&hl=en)

First we open the the menu.  
![sbt_settings](/Pictures/sbt_settings.jpg)

We choose devices and select the the device we wan't to connect.  
![sbt_connect](Pictures/sbt_connect.jpg)

And then we connect to the device in terminal.
![sbt_terminal](Pictures/sbt_terminal.jpg)
### Bluetooth command
We tried to add command to start the data reading with [Serial Bluetooth Terminal](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal&hl=en). To do this we added bluetooth to the example sensor code.
```bash
#include "BluetoothSerial.h"

const char *pin = "1234";

String device_name = "ESP32-BT-Slave";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif


#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif


BluetoothSerial SerialBT;
```

After adding these we can connect to the bluetooth while the sensor is working.  
Next we needed to add the control to start the reading when given command in [Serial Bluetooth Terminal](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal&hl=en).  
```bash
int incoming;

void setup(void){
	SerialBT.begin(device_name); 
	Serial.println("Bluetooth Device is Ready to Pair");
}

void loop() {

if (SerialBT.available()) //Check if we receive anything from Bluetooth
{
  incoming = SerialBT.read();
  Serial.print("Received:"); Serial.println(incoming);
  if (incoming == 49){
  }
}
```

<details>
<summary>The complete code</summary>
<pre>
  // Basic demo for accelerometer readings from Adafruit MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "BluetoothSerial.h"

const char *pin = "1234";
Adafruit_MPU6050 mpu;
String device_name = "ESP32-BT-Slave";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif


#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif


BluetoothSerial SerialBT;
int incoming;

void setup(void) {
Serial.begin(115200);
while (!Serial)
delay(10); // will pause Zero, Leonardo, etc until serial console opens

SerialBT.begin(device_name); 
Serial.println("Bluetooth Device is Ready to Pair");
Serial.println("Adafruit MPU6050 test!");

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

if (SerialBT.available()) //Check if we receive anything from Bluetooth
{
  incoming = SerialBT.read();
  Serial.print("Received:"); Serial.println(incoming);
  if (incoming == 49){
    // Get new sensor events with the readings //
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);


    // Print out the values //
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
}
}

</pre>
</details>

The code partially works.  
By sendin `1` in Serial Bluetooth Terminal the code gives us reading ones and then automatically shuts down.  
![bltTest](Pictures/bltCommandTest.png)
