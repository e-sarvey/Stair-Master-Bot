#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <AccelStepper.h>
#include <Wire.h>

    #define STEP_PIN  17  
    #define DIR_PIN   16  
    #define EN_PIN 4
    #define MICROSTEPS 16  // Microstepping factor (1/16)
    #define STEPS_PER_REV 200  // Steps per revolution for a standard stepper motor
    #define MASS_FOR_STEPS 2000
    #define MASS_BACK_STEPS 2000

    AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);


    void moveStepper(int steps) {
        stepper.move(steps * MICROSTEPS);

        while (stepper.distanceToGo() != 0) {
            stepper.run();
        }
    }

void setup() {
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW); // Enable the motor driver

    stepper.setMaxSpeed(1000);  // Set maximum speed (steps per second)
    stepper.setAcceleration(500);  // Set acceleration (steps per second^2)

    moveStepper(475);
    //moveStepper(-100);
}

void loop() {

}