// Wrapper for Adafruit Motor Shield
#include <Adafruit_MotorShield.h>

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

void setMotorSpeed(int  _motor, float _speed) {
    int speed = _speed * 255.0;

    Adafruit_DCMotor* motor = _motors[_motor];

    motor->setSpeed(min(abs(speed), 255));
    if (speed >= 0) {
      motor->run(FORWARD);
    } else {
      motor->run(BACKWARD);
    }
}
