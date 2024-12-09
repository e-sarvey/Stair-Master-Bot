#include <TMCStepper.h>

// Define control pins
#define DIR_PIN 16
#define STEP_PIN 17
#define EN_PIN 4

// Motor steps
#define STEPS_FORWARD 200 // Adjust to the desired forward steps
#define STEPS_BACKWARD 200 // Adjust to the desired backward steps
#define DELAY_BETWEEN_STEPS 1000 // Microseconds delay between steps

// Define microstepping value
#define MICROSTEPPING 16 // Change this value as needed (e.g., 1, 2, 4, 8, 16)

// Create a TMC2208Stepper instance
TMC2208Stepper driver = TMC2208Stepper(&Serial2, 0.11); // Replace with actual UART parameters

void setup() {
  // Initialize serial for UART communication with TMC2208
  Serial2.begin(115200); 

  // Initialize control pins
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  // Enable the driver
  digitalWrite(EN_PIN, LOW); // LOW to enable the driver

  // Configure the driver
  driver.begin(); // Initialize the driver
  driver.toff(4); // Enable the driver
  driver.rms_current(600); // Set current (in mA, adjust as needed)
  setMicrostepping(MICROSTEPPING); // Set microstepping dynamically
}

void loop() {
  // Move forward
  moveStepper(STEPS_FORWARD, true);
  
  // Pause before changing direction
  delay(1000);

  // Move backward
  moveStepper(STEPS_BACKWARD, false);

  // Pause before repeating
  delay(1000);
}

void moveStepper(int steps, bool forward) {
  digitalWrite(DIR_PIN, forward); // Set direction
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH); // Pulse step pin
    delayMicroseconds(DELAY_BETWEEN_STEPS);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(DELAY_BETWEEN_STEPS);
  }
}

void setMicrostepping(int microsteps) {
  // Set microstepping value dynamically
  switch (microsteps) {
    case 1: driver.microsteps(1); break;
    case 2: driver.microsteps(2); break;
    case 4: driver.microsteps(4); break;
    case 8: driver.microsteps(8); break;
    case 16: driver.microsteps(16); break;
    default: 
      Serial.println("Invalid microstepping value, setting to default 16.");
      driver.microsteps(16);
      break;
  }
  Serial.print("Microstepping set to: ");
  Serial.println(microsteps);
}