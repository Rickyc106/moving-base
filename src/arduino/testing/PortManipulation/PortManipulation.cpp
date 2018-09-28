/*
Port Manipulation Class
------------------------------------
Functions Supported Currently:
	- DigitalRead
Boards Supported Currently:
	- Atmega328P (Arduino Uno/Nano)
*/

#include "Arduino.h"
#include <PortManipulation.h>

PortManipulation::PortManipulation()
{

}

PortManipulation::digitalReadFast( int _pin )
{
	if ( _pin >= 0 && _pin < 8 )
	{
		int value = (PIND & (0x01 << _pin));
		return value;
	}

	if ( _pin >= 8 && _pin < 14 )
	{
		int value = (PINB & (0x01 << (_pin - 8)));
		return value;
	}

	if ( _pin >= 14 && _pin < 21 )
	{
		int value = (PINC & (0x01 << (_pin - 14))); // Might break code if A0 is used
		return value;
	}
}


PortManipulation::digitalWriteFast( int _pin)
{
	// Nothing here yet
}


