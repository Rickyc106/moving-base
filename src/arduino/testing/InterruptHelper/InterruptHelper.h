#ifndef InterruptHelper_h
#define InterruptHelper_h

#include "Arduino.h"

class InterruptHelper
{
	public:
		InterruptHelper();

		void portSelect( _pin );
		void pinMask( _pin );
		void attachPinInterrupt( _pin );
		void closeInterrupts();
		void reOpenInterrupts();
};

#endif
