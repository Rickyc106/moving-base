#ifndef PortManipulation_h
#define PortManipulation_h

#include "Arduino.h"

class PortManipulation
{
	public:
		PortManipulation();

		int digitalReadFast( _pin );
		int digitalWriteFast( _pin ); 
};

#endif
