#include "Arduino.h"
#include "Encoder.h"
#include "PortManipulation.h"

Encoder::Encoder()
{
	
}

static void Encoder::EXT_ISR_0()
{
	if (digitalReadFast(2)) uint16_t start_time = micros();
	else uint16_t stop_time = micros();
}

static void Encoder::EXT_ISR_1()
{
	if (digitalReadFast(3)) uint16_t start_time = micros();
	else uint16_t stop_time = micros();
}

static void Encoder::ISR(PCINT0_vect)
{
	if (digitalReadFast(2)) uint16_t start_time = micros();
	else uint16_t stop_time = micros();
}

static void Encoder::ISR(PCINT1_vect)
{
	if (digitalReadFast(2)) uint16_t start_time = micros();
	else uint16_t stop_time = micros();
}

static void Encoder::ISR(PCINT2_vect)
{
	if (digitalReadFast(2)) uint16_t start_time = micros();
	else uint16_t stop_time = micros();
}


Encoder::readSignal()
{
	
}

Encoder::outputSignal()
{
	
}


