#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int pin)
{
	_pin = pin;
}

void Motor::init()
{
	this->attach(_pin);
	pinMode(_pin, OUTPUT);
}

void Motor::drive(float speed, float limit)
{
	this->writeMicroseconds(1500 + (speed * 500 * limit));
}

void Motor::stop()
{
	this->writeMicroseconds(1500);
}