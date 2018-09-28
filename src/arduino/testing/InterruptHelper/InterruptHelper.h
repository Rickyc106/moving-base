#ifndef InterruptHelper_h
#define InterruptHelper_h

#include "Arduino.h"
#include <avr/interrupt.h>
#include <PortManipulation.h>

class InterruptHelper
{
	public:
		InterruptHelper();
		
		void closeInterrupts();
		void reOpenInterrupts();
		void portSelect( int _pin );
		void pinMask( int _pin );
		void attachPinInterrupt( int _pin );
};

#endif
