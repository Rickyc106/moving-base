/*
Port Manipulation Class
------------------------------------
Functions Supported Currently:
	- DigitalRead
Boards Supported Currently:
	- Atmega328P (Arduino Uno/Nano)
*/

#ifndef PortManipulation_h
#define PortManipulation_h

#include "Arduino.h"

class PortManipulation
{
	public:
		PortManipulation();

		int digitalReadFast( int _pin );
		int digitalWriteFast( int _pin ); 
};

#endif
