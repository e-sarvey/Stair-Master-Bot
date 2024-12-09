#include <AccelStepper.h>

// Pin Definitions
// H-Bridge 1
#define MOTOR1_IN1  26  // Front-left motor
#define MOTOR1_IN2  25
#define MOTOR2_IN1  33  // Front-right motor
#define MOTOR2_IN2  32

// initialize steppers
#define STEPPER1_STEP  17  // Step pin for stepper 1
#define STEPPER1_DIR   16  // Direction pin for stepper 1

// Create Stepper Motor Instances i used accel but could be any i guess
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP, STEPPER1_DIR);

// Steps
#define FRONT_LIFT_STEPS = 3000
#define BACK_LIFT_STEPS = -3000
#define FRONT_LOWER_STEPS = 500
#define BACK_LOWER_STEPS = -500
#define MASS_FOR_STEPS = 2000
#define MASS_BACK_STEPS = 2000

//Timing
#define FORWARD_TIME = 2000
#define DELAY = 100


// Helper Functions
void stopMotors() {
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
}

void motorOn(int motor1_in1, int motor1_in2, int motor2_in1, int motor2_in2) {
  digitalWrite(motor1_in1, HIGH);
  digitalWrite(motor1_in2, LOW);
  digitalWrite(motor2_in1, HIGH);
  digitalWrite(motor2_in2, LOW);
}

void moveForward(int move_time){
  motorOn(MOTOR1_IN1, MOTOR1_IN2, MOTOR2_IN1, MOTOR2_IN2); // Front motors
  delay(move_time); // Run for defined time
  stopMotors();
}

void moveStepper1(int steps) {
  stepper1.move(steps);
  
  while (stepper1.distanceToGo() != 0)
    stepper1.run();
  }
}

// Function to calculate time to take a specific number of steps maybe? we need to look at the stepper to see what the built in delay is. or make our own delay after each step. 
unsigned long calculateTimeForSteps(int numSteps, int delay_ms) {
    // Calculate the number of steps per second
    float stepsPerSecond = 1000.0 / delay_ms;

    // Calculate the time required for the number of steps in milliseconds
    unsigned long timeMs = numSteps / stepsPerSecond * 1000;

    return timeMs;
}

// BACK ESP
void moveOneStair(){
  // Step 1: Move back motors forward
  moveForward(FORWARD_TIME);
  // Lift front
  moveStepper1(FRONT_LIFT_STEPS);
  // Move back motors forward
  moveForward(FORWARD_TIME);
  // Lower front
  moveStepper1(FRONT_LOWER_STEPS);
  // Lift back
  moveStepper1(BACK_LIFT_STEPS);
  delay(FORWARD_TIME); //Pause for front wheels forward
}

void setup() {
  // Initialize all motor pins as outputs
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);

  // Stop all motors initially
  stopMotors();

  for (int i = 0; i < 8; i++) {
      moveOneStair();
      delay(500);
  }
}

void loop() {

}
