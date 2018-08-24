#include <avr/interrupt.h>

#define PORT_D_pins 6
#define PORT_B_pins 2

volatile int pulse_count[8];
volatile unsigned long start_time[8], stop_time[8];

unsigned long period[8];
float alpha = 0.7;
unsigned long comp_period[8];

int specified_bit;
uint16_t value;

void setup() {
  Serial.begin(9600);

  cli();

  PCICR |= 0b00000101;    // turn on ports b and d
  PCMSK0 |= 0b00000011;   // turn on pins PB0 and PB1 -- PCINT0 & PCINT1, physical ATMEGA pin 14 & 15, UNO pins 8 & 9
  PCMSK2 |= 0b11111100;   // Pins for Port D instead

  sei();

  //attachInterrupt(0, encoderA, RISING);
  //attachInterrupt(1, encoderB, RISING);
}

void loop() {
  //Serial.print(pulse_count / (11.6666666667 * 70 * 1.02)); // 1 pulse = 2 Interrupt Lo -> Hi & Hi -> Lo
  // PG71 Incremental Encoder: 7 pulses per rev
  // Gear Overduction: Factor of 1.2 Faster output --- (2 * 7) / 1.2 = 11.66667
  /*
    for (int i = 0; i < PORT_D_pins; i++) {
    Serial.print((PIND & (0x01 << i + PORT_B_pins)) >> i + PORT_B_pins);
    Serial.print("\t");
    }
    for (int i = 0; i < PORT_B_pins; i++) {
    Serial.print((PINB & (0x01 << i)) >> i);
    Serial.print("\t");
    }
  */

  /*
    for (int i = 0; i < 8; i++) {
    Serial.print(pulse_count[i]);
    Serial.print("\t");
    }
  */

  /*
    for (int i = 0; i < 8; i++) {
    Serial.print((byte) pulse_count[i]);
    Serial.print("\t");
    }
  */

  for (int i = 0; i < 8; i++) {
    if (stop_time[i] > start_time[i]) period[i] = stop_time[i] - start_time[i];
    if (period[i] > 15000) period[i] = 15000;

    comp_period[i] = alpha * (comp_period[i]) + ((1 - alpha) * period[i]);

    //Serial.write(comp_period[i] >> 8);
    //Serial.write(comp_period[i] << 8);

    //Serial.write(comp_period[i]); // Stuck here -- Need to transmit 16 bit value for each period or maybe just 2 bytes?

    for (int j = 16; j > 0; j--) {
      specified_bit = comp_period[i] >> j;
      
      if (specified_bit & 1) value & (0x01 >> j);
      else value & (0x0 >> j); 
    }

    Serial.print(comp_period[i]);
    Serial.print("\t");
  }

  /*
  for (int i = 0; i < 2; i++) {
    ext_int_period[i] = current_pulse[i] - previous_pulse[i];

    Serial.print(ext_int_period[i]);
    Serial.print("\t");
  }
  */

  Serial.println();
}

/////
// Caution: 1st and 3rd encoders are flipped currently!!
/////

ISR(PCINT0_vect) {
  for (int i = 0; i < PORT_B_pins; i++) {
    if (((PINB & (0x01 << i)) >> i) == 0x01) start_time[i + PORT_D_pins] = micros();
    else stop_time[i + PORT_D_pins] = micros();
  }
}

ISR(PCINT2_vect) {    // Port D, PCINT16 - PCINT23
  for (int i = 0; i < PORT_D_pins; i++) {
    if (((PIND & (0x01 << i + PORT_B_pins)) >> i + PORT_B_pins) == 0x01) start_time[i] = micros();
    else stop_time[i] = micros();
  }
}
