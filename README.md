# SmartDisc
School project for Haaga Helia. 
  
In this project we are creating a smart device that will be attached to a disc golf disc. The device will have a buzzer to help find the disc if it is thrown in to the bush. It will also have sensors for acceleration and gyro, so we can get the top speed and rotation speed of the disc. Gyro sensor will also be used to get the angle of the disk.
  
The idea is that the player can use these information to help better their play.


## Project Plan

- Assignment for Ohjelmistoprojekti II (Software Project II)
- Purpose of our project is to design a web application using open data

## Development Tools

- Web application created with React App
- Mockup created with [Figma](https://www.figma.com/)
- As database using (currently only for Authentication) [Firebase Auth, Firestore](https://firebase.google.com/)
- We are using [Spoonacular](https://www.spoonacular.com/) to fetch recipes and ingredients

<sub>**Further development aims to expand the app to be mobile responsive, and for the user to be able to favourite recipes**</sub>

## Yummyfier

- Food recipe web application for users interested in cooking
- App has various options and categories from vegetarian to meat
- App aims to reduce food waste, save time and inspire

Idea in short:
- Do you have ingredients but don't know what you can make of them
- Write your ingredient(s) to the app and it suggests recipe ideas that can be made of them
- Create an account, so that if you come to like any of the recipes you can save them to your profile and access them from all of your devices

<sub>**However, if you only have few basic ingredients (eggs, flours...) we can't ensure you can make something only with those**</sub>

## User Stories

- As an uninspired cook, I want to get ideas of what I can do with the random items in my fridge, so that I don't need to go to the store.
- As a User I want to be able to list ingredients I have to get recipes
- As a User I want to be able to save the recipes I really liked, so that I can use them again
- As a developer I want to have free and broad data which I can just fetch easily with no limits

## Target Audience

- The target audience for Yummyfier are individuals who are interested in cooking and are seeking inspiration for their meals
- This includes people who may not have a lot of experience in the kitchen and are looking for new recipe ideas, as well as experienced cooks who want to try out new dishes
- Yummyfier appeals to those who want to reduce food waste and save time by utilizing ingredients they already have on hand, rather than making an unnecessary trip to the store
- By allowing users to save recipes they enjoy, the app encourages repeat use
- Overall, Yummyfier is designed to meet the needs of a broad range of users who are interested in cooking, with a focus on simplicity, convenience, and accessibility.

## Tests
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
