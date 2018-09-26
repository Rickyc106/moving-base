#ifndef Encoder_h
#define Encoder_h

#include "Arduino.h"
#include "PortManipulation.h"

class Encoder: public PortManipulation
{
	public:
		Encoder();

		void readSignal();
		unsigned short outputSignal();
};

#endif
