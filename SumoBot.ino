#include "Ultrasonic.h"  // Grove Ultrasonic library (not really used, custom function is written below)

// -----------------------------
// Pin definitions
// -----------------------------
#define ULTRASONIC_PIN 4   // Pin connected to ultrasonic sensor
#define IR_LEFT A0         // Left IR sensor pin
#define IR_RIGHT A1        // Right IR sensor pin

// Motor control pins
#define M1A 5   // Motor 1 backward control (PWM)
#define M1B 6   // Motor 1 forward control (PWM)
#define M2A 9   // Motor 2 backward control (PWM)
#define M2B 10  // Motor 2 forward control (PWM)

// -----------------------------
// Parameters / constants
// -----------------------------
#define PATROL_SPEED 120       // Speed while patrolling/searching
#define ATTACK_SPEED 255       // Max speed while attacking
#define TURN_SPEED 150         // Speed while turning
#define SAFE_DISTANCE_CM 20    // Distance at which enemy is considered "close"
#define IR_CALIBRATION_SAMPLES 10 // Number of samples to calibrate IR sensors

// -----------------------------
// Variables for IR sensor handling
// -----------------------------
int irThreshold = 700;          // Threshold for detecting the white line
const int debounceDelay = 50;   // Debounce time for IR line detection
unsigned long lastIrChange = 0; // Last time IR state changed
bool irLineDetected = false;    // Whether line is detected

// -----------------------------
// Variables for reversing/turning when line detected
// -----------------------------
unsigned long reverseStart = 0;     // Time when reversing started
const unsigned long reverseDuration = 400; // How long to reverse (ms)
const unsigned long turnDuration = 600;    // How long to turn after reversing (ms)
bool reversing = false;
bool turning = false;

// -----------------------------
// State machine definition
// -----------------------------
enum State { STARTUP, PATROLLING, ATTACKING, REVERSING } currentState = STARTUP;

unsigned long lastEnemySeen = 0;   // Last time an enemy was detected
unsigned long startupBeginTime = 0; // When startup delay began

// Patrolling turn management
unsigned long lastPatrolTurnTime = 0;
bool patrolTurnRight = true; // Alternate turning left/right during patrol

// Ultrasonic object
Ultrasonic ultrasonic(ULTRASONIC_PIN);

// -----------------------------
// Setup function
// -----------------------------
void setup() {
  Serial.begin(9600); // Serial monitor for debugging
  
  // Pin setup
  pinMode(ULTRASONIC_PIN, OUTPUT);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  
  // Calibrate IR sensors
  Serial.println("Calibrating IR sensors...");
  calibrateIRThreshold();  // Set threshold based on average readings
  Serial.print("IR Threshold set to: ");
  Serial.println(irThreshold);
  
  // Startup delay (5 seconds before robot begins moving)
  Serial.println("Sumo Bot Ready! Waiting 5 seconds before start...");
  startupBeginTime = millis();
}

// -----------------------------
// Main loop
// -----------------------------
void loop() {
  // -------- STARTUP state --------
  if (currentState == STARTUP) {
    if (millis() - startupBeginTime >= 5000) { // Wait 5 seconds
      currentState = PATROLLING;               // Start patrolling
      lastPatrolTurnTime = millis();
      Serial.println("Startup complete! Beginning patrol.");
    } else {
      stopMotors(); // Stay still during startup
      return;
    }
  }

  // -------- Sensor readings --------
  int irLeft = analogRead(IR_LEFT);
  int irRight = analogRead(IR_RIGHT);
  long distance = readUltrasonicCM();

  // Debug print
  Serial.print("IR L: "); Serial.print(irLeft);
  Serial.print(" IR R: "); Serial.print(irRight);
  Serial.print(" Dist: ");
  if (distance == -1) Serial.print("No Echo");
  else Serial.print(distance);
  Serial.print(" cm | State: ");

  // -------- Line detection --------
  bool lineNow = (irLeft > irThreshold || irRight > irThreshold);
  if (lineNow != irLineDetected && millis() - lastIrChange > debounceDelay) {
    // Only register change if debounce delay passed
    irLineDetected = lineNow;
    lastIrChange = millis();
  }

  // If line detected, trigger reverse + turn sequence
  if (irLineDetected && !reversing && !turning) {
    reversing = true;
    reverseStart = millis();
    currentState = REVERSING;
    stopMotors();
    Serial.println("Line DETECTED -> Reversing");
  }

  // -------- Reversing sequence --------
  if (reversing) {
    driveMotors(-PATROL_SPEED, -PATROL_SPEED); // Move backward
    if (millis() - reverseStart >= reverseDuration) {
      reversing = false;
      turning = true;        // Switch to turning
      reverseStart = millis();
      Serial.println("Reverse done -> Turning");
    }
    return; // Skip rest of loop while reversing
  }

  // -------- Turning sequence --------
  if (turning) {
    driveMotors(-TURN_SPEED, TURN_SPEED); // Turn in place
    if (millis() - reverseStart >= turnDuration) {
      turning = false;
      currentState = PATROLLING;
      Serial.println("Turn done -> Patrolling");
    }
    return; // Skip rest of loop while turning
  }

  // -------- Enemy detection (ATTACK) --------
  if (distance != -1 && distance <= SAFE_DISTANCE_CM && distance > 3) {
    lastEnemySeen = millis();
    currentState = ATTACKING;
    driveMotors(ATTACK_SPEED, ATTACK_SPEED); // Charge forward
    Serial.println("Enemy DETECTED -> Attacking!");
  } else {
    // If lost enemy after attacking, switch back to patrol
    if (currentState == ATTACKING && millis() - lastEnemySeen > 2000) {
      currentState = PATROLLING;
      Serial.println("Enemy lost -> Patrolling");
    }

    // -------- PATROLLING (searching for enemy) --------
    if (currentState == PATROLLING) {
      // Alternate turning left/right every 2 seconds
      if (millis() - lastPatrolTurnTime > 2000) {
        patrolTurnRight = !patrolTurnRight;
        lastPatrolTurnTime = millis();
      }
      // Execute patrol turn
      if (patrolTurnRight)
        driveMotors(TURN_SPEED, -TURN_SPEED);
      else
        driveMotors(-TURN_SPEED, TURN_SPEED);
      Serial.println("Patrolling - Searching");
    }
  }
  delay(10); // Small loop delay
}

// -----------------------------
// Ultrasonic reading function
// -----------------------------
long readUltrasonicCM() {
  pinMode(ULTRASONIC_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_PIN, LOW);

  pinMode(ULTRASONIC_PIN, INPUT);
  long duration = pulseIn(ULTRASONIC_PIN, HIGH, 30000); // Max timeout 30ms

  if(duration == 0) return -1; // No echo

  long distanceCm = duration * 0.034 / 2; // Convert to cm
  if(distanceCm < 3 || distanceCm > 350) return -1; // Out of range

  return distanceCm;
}

// -----------------------------
// IR calibration function
// -----------------------------
void calibrateIRThreshold() {
  int sum = 0;
  for(int i = 0; i < IR_CALIBRATION_SAMPLES; i++) {
    sum += analogRead(IR_LEFT);
    sum += analogRead(IR_RIGHT);
    delay(20);
  }
  // Set threshold slightly above average value
  irThreshold = (sum / (IR_CALIBRATION_SAMPLES * 2)) + 100;
}

// -----------------------------
// Motor control function
// -----------------------------
void driveMotors(int speedA, int speedB) {
  // Constrain speed values
  speedA = constrain(speedA, -255, 255);
  speedB = constrain(speedB, -255, 255);

  // Motor A control
  if(speedA >= 0) {
    digitalWrite(M1A, LOW);
    analogWrite(M1B, speedA); // Forward
  } else {
    digitalWrite(M1A, HIGH);
    analogWrite(M1B, -speedA); // Reverse
  }

  // Motor B control
  if(speedB >= 0) {
    digitalWrite(M2A, LOW);
    analogWrite(M2B, speedB); // Forward
  } else {
    digitalWrite(M2A, HIGH);
    analogWrite(M2B, -speedB); // Reverse
  }
}

// -----------------------------
// Stop motors function
// -----------------------------
void stopMotors() {
  analogWrite(M1B, 0);
  analogWrite(M2B, 0);
  digitalWrite(M1A, LOW);
  digitalWrite(M2A, LOW);
}
