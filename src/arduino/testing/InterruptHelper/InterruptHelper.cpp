#include "Arduino.h"
#include <avr/interrupt.h>
#include <InterruptHelper.h>

typedef void (*voidFuncPtr)(void);

InterruptHelper::InterruptHelper()
{
	int _pin;
	voidFuncPtr pinChangeInt();
}

Port::Port()
{
	int _pin;
	voidFuncPtr pinChangeInt();
}

PinInterrupt::PinInterrupt()
{
	int _port;
	int _pin;
	voidFuncPtr pinChangeInt();
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
	if ( _pin >= 0 && _pin < 4 ) 
	{
		PCMSK2 |= (0x0);
		if ( _pin == 2 ) attachInterrupt( _pin, EXT_ISR_0, CHANGE);
		if ( _pin == 3 ) attachInterrupt( _pin, EXT_ISR_1, CHANGE);
	}

	// Pins 4-7, 8-14, 14-21
	if ( _pin >= 4 && _pin < 8) PCMSK2 |= (0x01) << _pin;
	if ( _pin >= 8 && _pin < 14) PCMSK1 |= (0x01) << (_pin - 8);
	if ( _pin >= 14 && _pin < 21) PCMSK0 |= (0x01) << (_pin - 14);	// Might break code if A0 is used
}

void InterruptHelper::attachPinInterrupt( int pin, voidFuncPtr userFunc)	// Only using interrupt on Change for now
{
	closeInterrupts();
	portSelect( pin );
	pinMask( pin );
	reOpenInterrupts();

	PinInterrupt* p = new PinInterrupt;
	p->_pin = pin;
	p->pinChangeInt = userFunc;
}

void Port::UserSpecifiedFunction()
{
	while(pins_interrupted > 0)
	{
		p->pinChangeInt();
	}
}

Port EXT_0, EXT_1, PORTA, PORTB, PORTC;


EXT_ISR_0()
{
	EXT_0.UserSpecifiedFunction();
}

EXT_ISR_1()
{
	EXT_1.UserSpecifiedFunction();	
}

ISR(PCINT0_vect)
{
	PORTA.UserSpecifiedFunction();
}

ISR(PCINT1_vect)
{
	PORTB.UserSpecifiedFunction();	
}

ISR(PCINT2_vect)
{
	PORTC.UserSpecifiedFunction();
}



