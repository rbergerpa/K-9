// Wrapper for Adafruit Motor Shield
#include <Adafruit_MotorShield.h>
#include "AFMotor.h"

static const int _numMotors=4;

static Adafruit_MotorShield _AFMS = Adafruit_MotorShield();
static Adafruit_DCMotor* _motors[_numMotors];

void initMotors() {
    _AFMS.begin();

    for (int i = 0; i < _numMotors; i++) {
         _motors[i] = _AFMS.getMotor(i+1);
         _motors[i]->setSpeed(0);
    }
}

void setMotorSpeed(int motor, int speed) {
    Adafruit_DCMotor* _motor = _motors[motor];

    _motor->setSpeed(min(abs(speed), 255));
    if (speed >= 0) {
      _motor->run(FORWARD);
    } else {
      _motor->run(BACKWARD);
    }
}
