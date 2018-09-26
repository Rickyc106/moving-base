#include "Arduino.h"
#include "InterruptHelper.h"

InterruptHelper::InterruptHelper()
{

}

void InterruptHelper::closeInterrupts()
{
	cli();
}

void InterruptHelper::reOpenInterrupts()
{
	sei();
}

void InterruptHelper::portSelect( int _pin )
{
	if ( _pin >= 0 && _pin < 8 ) PCICR |= 0b00000100;	// Port D
	if ( _pin >= 8 && _pin < 14 ) PCICR |= 0b00000001;	// Port B
	if ( _pin >= 14 && _pin < 21 ) PCICR |= 0b00000010;	// Port C
}

void InterruptHelper::pinMask( int _pin )
{
	// Ignore RX/TX, INT0, and INT1 pins
	if ( _pin >= 0 && _pin < 4 ) PCMSK2 |= (0x0);

	// Pins 4-7, 8-14, 14-21
	if ( _pin >= 4 && _pin < 8) PCMSK2 |= (0x01) << _pin;
	if ( _pin >= 8 && _pin < 14) PCMSK1 |= (0x01) << _pin - 8;
	if ( _pin >= 14 && _pin < 21) PCMSK0 |= (0x01) << pin - 14;	// Might break code if A0 is used
}

void InterruptHelper::attachPinInterrupt( int pin )
{
	closeInterrupts();
	portSelect( pin );
	pinMask( pin );
	reOpenInterrupts();
}



