#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor
{
	public:
		Motor(int pin);

		void init();
		void drive();
		void stop();
		void vectorSum();
		void PID();

	private:
		int _pin;
};

#endif
