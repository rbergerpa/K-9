// Wrapper for Adafruit Motor Shield
#include <Adafruit_MotorShield.h>
#include "AFMotor.h"

static const int numMotors=4;

static Adafruit_MotorShield AFMS = Adafruit_MotorShield();
static Adafruit_DCMotor* motors[numMotors];

void initMotors() {
    AFMS.begin();

    for (int i = 0; i < numMotors; i++) {
         motors[i] = AFMS.getMotor(i+1);
         motors[i]->setSpeed(0);
    }
}

void setMotorSpeed(int  _motor, int speed) {
    Adafruit_DCMotor* motor = motors[_motor];

    motor->setSpeed(min(abs(speed), 255));
    if (speed >= 0) {
      motor->run(FORWARD);
    } else {
      motor->run(BACKWARD);
    }
}
