#ifndef Encoder_h
#define Encoder_h

#include "Arduino.h"
#include <avr/interrupt.h>
#include <InterruptHandler.h>
#include <PortManipulation.h>

class Encoder: public PortManipulation, public InterruptHandler
{
	public:
		Encoder();

		static void EXT_ISR_0();
		static void EXT_ISR_1();

		static void ISR(PCINT0_vect);
		static void ISR(PCINT1_vect);
		static void ISR(PCINT2_vect);

		static void 

		void readSignal();
		unsigned short outputSignal();
};

ISR(PCINT0_vect)
{
	
}

#endif
