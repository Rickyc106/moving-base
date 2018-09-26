#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor
{
	public:
		Motor(int pin);

		void init();
		void drive(float speed, float limit);
		void stop();

	private:
		int _pin;
};

#endif
