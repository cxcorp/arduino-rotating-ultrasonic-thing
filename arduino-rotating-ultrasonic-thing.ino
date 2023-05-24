// Arduino Servo library
#include <Servo.h>
// HCSR04 library by Martin Sosic
#include <HCSR04.h>
#include "RotaryEncoderKy040.h"

// pins
#define SERVO_PIN 9

#define ROTARY_CLK 2
#define ROTARY_DT 3

#define SONAR_TRIG 12
#define SONAR_ECHO 11
#define SONAR_MAX_DIST_CM 200
// 50ms
#define SONAR_MAX_TIMEOUT_MICROS 50000000

// constants
const unsigned int ROTARY_THROTTLE_MS = 50;
const unsigned int ROTARY_CLICK_THROTTLE_MS = 250;
const unsigned int SONAR_THROTTLE_MS = 500;
const int DEGREES_PER_ROTARY_ENCODER_STEP = 180 / 20;

// definitions
void rotaryInterruptServiceRoutine();

// variables

Servo servo;
RotaryEncoder rotary(&rotaryInterruptServiceRoutine, ROTARY_DT, ROTARY_CLK);
UltraSonicDistanceSensor distanceSensor(SONAR_TRIG, SONAR_ECHO, SONAR_MAX_DIST_CM, SONAR_MAX_TIMEOUT_MICROS);

int prevServoPos = 0;

// We update this in the interrupt routine so they must be marked as volatile.
// Volatile tells the compiler essentially that these can be updated outside of
// its code paths so it must not optimize away reads or writes to these variables
// when it thinks it can just reuse the last value.
volatile int rotaryPosition = 0;
int prevRotaryPosition = 0;
int rotaryDiff = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  servo.attach(SERVO_PIN);

  pinMode(SERVO_PIN, OUTPUT);
  pinMode(SONAR_TRIG, OUTPUT);
  pinMode(SONAR_ECHO, INPUT);
  rotary.setup();

  // attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), rotaryInterruptServiceRoutine, FALLING);
  Serial.println("rotaryPos;servoPos");
}

void rotaryInterruptServiceRoutine() {
  const unsigned int state = rotary.GetState();
  if (state & DIR_CW) {
    rotaryPosition--;
  }
  if (state & DIR_CCW) {
    rotaryPosition++;
  }
}

void updateRotaryPosition() {
  rotaryDiff = rotaryPosition - prevRotaryPosition;
  prevRotaryPosition = rotaryPosition;
}

int calculateNewServoPosition() {
  return constrain(prevServoPos + rotaryDiff * DEGREES_PER_ROTARY_ENCODER_STEP, 0, 180);
}

void applyServoPosition(int pos) {
  if (pos == prevServoPos) {
    return;
  }

  servo.write(pos);
  prevServoPos = pos;
}

// void updateSonar() {
//   digitalWrite(SONAR_TRIG, LOW);
//   delayMicroseconds(2);
//   digitalWrite(SONAR_TRIG, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(SONAR_TRIG, LOW);

//   float duration = pulseIn(SONAR_ECHO, HIGH);
//   float distance = (duration * 0.0343) / 2;
//   Serial.println(distance);
// }

unsigned int lastServoThrottleMs = 0;
unsigned int lastSonarThrottleMs = 0;

void loop() {
  unsigned int now = millis();

  if (now - lastServoThrottleMs > ROTARY_THROTTLE_MS) {
    updateRotaryPosition();
    int newServoPos = calculateNewServoPosition();
    applyServoPosition(newServoPos);
    lastServoThrottleMs = now;
  }

  if (now - lastSonarThrottleMs > SONAR_THROTTLE_MS) {
    Serial.println(distanceSensor.measureDistanceCm());
    lastSonarThrottleMs = now;
  }
}
