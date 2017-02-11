//#include <PciManager.h>
//#include <Sonar.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

# include "AFmotor.h"  // Uncomment for Adafruit Motor Shield

const int START_SONAR_PIN = 6;
const int LEFT_SONAR_PIN = 2;
const int CENTER_SONAR_PIN = 3;
const int RIGHT_SONAR_PIN = 4;

const int MIN_SIDE_DISTANCE = 36;  // inches
const int MIN_CENTER_DISTANCE = 14;
const int SERVO_CENTER = 1450; // microseconds

enum {Left, Right};

#if (AVOID_OBSTACLES)
Sonar leftSonar(LEFT_SONAR_PIN);
Sonar centerSonar(CENTER_SONAR_PIN);
Sonar rightSonar(RIGHT_SONAR_PIN);
#endif

void setup() {
    initSerial();
    
    Serial.println("Initializing");

#if AVOID_OBSTACLES    
    startSonar(START_SONAR_PIN);
#endif

    initMotors();

    Serial.println("Ready");
}

void printDistances(int left, int center, int right) {
  Serial.print("Left: ");
  Serial.print(left);

  Serial.print("  Center: ");
  Serial.print(center);

  Serial.print("  Right: ");
  Serial.print(right);
  Serial.println();
}

int userSteering = 0;
int userSpeed = 0;

/* void k_turn(int left, int center, int right) { */
/*   Serial.println("Performing K turn"); */
  
/*   if (left > right) { */
/*     servo.writeMicroseconds(SERVO_MAX); */
/*   } else { */
/*     servo.writeMicroseconds(SERVO_MIN); */
/*   } */
  
/*   delay(250); */
/*   motor->run(BACKWARD); */
/*   delay(2000); */
  
/*   motor->setSpeed(0); */
/*     servo.writeMicroseconds(SERVO_CENTER); */
/*   delay(200); */
  
/*   motor->run(FORWARD); */
/*   motor->setSpeed(userSpeed); */
/* } */
  

void loop() {
  int servo_ms;
  
  pollSerial();

  setMotorSpeed(Left, userSpeed + userSteering);
  setMotorSpeed(Right, userSpeed - userSteering);
  
#ifdef AVOID_OBSTACLES
  int left = leftSonar.inches();
  int center = centerSonar.inches();
  int right = rightSonar.inches();
// printDistances(left, center, right);
  

  if (center < MIN_CENTER_DISTANCE) {
    k_turn(left, center, right);
    return;
  }
  
  if (left < MIN_SIDE_DISTANCE || right < MIN_SIDE_DISTANCE) {
    servo_ms = SERVO_CENTER - SERVO_GAIN * (left - right);
    if (servo_ms < SERVO_MIN) {
      servo_ms = SERVO_MIN;
    } else if (servo_ms > SERVO_MAX) {
      servo_ms = SERVO_MAX;
    }
  } else {
    servo_ms = SERVO_CENTER + userSteering;
  }
#endif  
}

String inString = "";

void initSerial() {
    inString.reserve(100);

    Serial.begin(115200);
}

void setSpeed(int speed) {
  Serial.print("Setting speed to ");
  Serial.println(speed);
  userSpeed = speed;
}

void setSteering(int steering) {
  Serial.print("Setting steering to ");
  Serial.println(steering);
  userSteering = steering;
}

void parseCommand(String &str) {  
  if (str.startsWith("speed=")) {
    setSpeed(str.substring(strlen("speed=")).toInt());
  } else   if (str.startsWith("steering=")) {
    setSteering(str.substring(strlen("steering=")).toInt());
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      parseCommand(inString);
      inString = "";
    } else {
      inString += ch;
    }
  }
}



