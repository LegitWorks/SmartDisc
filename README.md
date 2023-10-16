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
![image](https://media.discordapp.net/attachments/913061042148499487/1163406805397934190/20230929_100038.jpg?ex=653f7605&is=652d0105&hm=e6591beaeddf57bf956c4a0796f6172c5883d8edade91e0dddd5870adc060f13&=&width=717&height=660)
- Chips used: MPU-6050 (3 access gyro & accelometer)  
![image](https://media.discordapp.net/attachments/913061042148499487/1163406804307415110/20230929_100133.jpg?ex=653f7605&is=652d0105&hm=afe2fe04bccabb63cbace95940f46b6895e7d53410db9a23560b789f7d67ff7f&=&width=642&height=733)
![image](https://media.discordapp.net/attachments/913061042148499487/1163406805003681873/20230929_100121.jpg?ex=653f7605&is=652d0105&hm=0e4370464c5edadeb06fa380a6f3979f38086337775d5840384f3da678e4bc1a&=&width=586&height=733)

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
