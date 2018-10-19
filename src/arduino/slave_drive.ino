#include <EasyTransfer.h>
#include <avr/interrupt.h>

volatile unsigned long start_time[4], stop_time[4];
volatile bool motor_dir[4];

unsigned short period[4];
unsigned short comp_period[4];
int counter[4];
long pulse_count[4];

float alpha = 0.99; // 0.9

uint8_t byte_1, byte_2;

EasyTransfer ETout_drive;

struct DRIVE_DATA_STRUCTURE {
  unsigned short filter_period[4];
  float wheel_rotation[4];
};

DRIVE_DATA_STRUCTURE tx_drive;

void setup() {
  Serial.begin(115200);

  ETout_drive.begin(details(tx_drive), &Serial);

  cli();  // Turn off interrupts temporarily

  PCICR |= 0b00000011;    // turn on ports b and c
  PCMSK0 |= 0b00010000;   // turn on pin PB5 -- PCINT5, physical ATMEGA pin 19, UNO pin 13
  PCMSK1 |= 0b00000001;   // turn on pin PC0 -- PCINT8, physical ATMEGA pin 23, UNO pin A0

  sei();  // Re-open interrupts

  attachInterrupt(0, encoderA, CHANGE);
  attachInterrupt(1, encoderB, CHANGE);
}

void loop() {
  //Serial.print(pulse_count / (11.6666666667 * 70 * 1.02)); // 1 pulse = 2 Interrupt Lo -> Hi & Hi -> Lo
  // PG71 Incremental Encoder: 7 pulses per rev
  // Gear Overduction: Factor of 1.2 Faster output --- (2 * 7) / 1.2 = 11.66667

  for (int i = 0; i < 4; i++) {
    if (stop_time[i] > start_time[i]) period[i] = stop_time[i] - start_time[i];   // Measure period between pulse (Time duration LOW or HIGH)
    else period[i] = start_time[i] - stop_time[i];
    
    if (period[i] > 15000) period[i] = 15000;   // Cap period to 15000 micros or 15 ms -- MAX 16 bit = 65,536 Decimal

    comp_period[i] = alpha * (comp_period[i]) + ((1 - alpha) * period[i]);    // Low pass filter to attenuate spike measurements
    tx_drive.filter_period[i] = period[i];
    tx_drive.wheel_rotation[i] = pulse_count[i] / (11.6666666667 * 70 * 1.02);
    
    ETout_drive.sendData();
  }
}

ISR(PCINT0_vect) {    // Motor 1
  if (PINB & (0x01 << 4)) {
    start_time[1] = micros();

    if (PINC & (0x01 << 1)) {
      motor_dir[1] = 1;
      pulse_count[1]++;
    }
    else {
      motor_dir[1] = 0;
      pulse_count[1]--;
    }
  }

  else {
    stop_time[1] = micros();

    if (PINC & (0x01 << 1)) {
      motor_dir[1] = 0;
      pulse_count[1]--;
    }
    else {
      motor_dir[1] = 1;
      pulse_count[1]++;
    }
  }
}

ISR(PCINT1_vect) {    // Motor 2
  if (PINC & (0x01 << 0)) {
    start_time[2] = micros();

    if (PINC & (0x01 << 2)) {
      motor_dir[2] = 1;
      pulse_count[2]++;
     }
    else {
      motor_dir[2] = 0;
      pulse_count[2]--;
    }
  }
  else {
    stop_time[2] = micros();

    if (PINC & (0x01 << 2)) {
      motor_dir[2] = 0;
      pulse_count[2]--;
    }
    else {
      motor_dir[2] = 1;
      pulse_count[2]++;
    }
  }
}

void encoderA() {   // Motor 0
  if (PIND & (0x01 << 2)) {
    start_time[0] = micros();

    if (PIND & (0x01 << 4)) {
      motor_dir[0] = 1;
      pulse_count[0]++;
    }
    else {
      motor_dir[0] = 0;
      pulse_count[0]--;
    }
  }

  else {
    stop_time[0] = micros();

    if (PIND & (0x01 << 4)) {
      motor_dir[0] = 0;
      pulse_count[0]--;
    }
    else {
      motor_dir[0] = 1;
      pulse_count[0]++;
    }
  }
}

void encoderB() {   // Motor 3
  if (PIND & (0x01 << 3)) {
    start_time[3] = micros();

    if (PIND & (0x01 << 5)) {
      motor_dir[3] = 1;
      pulse_count[3]++;
    }
    else {
      motor_dir[3] = 0;
      pulse_count[3]--;
    }
  }

  else {
    stop_time[3] = micros();

    if (PIND & (0x01 << 5)) {
      motor_dir[3] = 0;
      pulse_count[3]--;
    }
    else {
      motor_dir[3] = 1;
      pulse_count[3]++;
    }
  }
}
