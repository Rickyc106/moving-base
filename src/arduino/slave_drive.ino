#include <EasyTransfer.h>
#include <avr/interrupt.h>

#define PORT_D_pins 6
#define PORT_B_pins 2

volatile int pulse_count[4];
volatile unsigned long start_time[4], stop_time[4];
volatile bool motor_dir[4];

unsigned short period[4];
unsigned short last_period[4];

bool previous_B[8];

float alpha = 0.99; // 0.9
double beta = 15;

int specified_bit;

unsigned long randNumber;
volatile int val;

byte data_out[6];
unsigned short motor_speed[4];
unsigned short comp_period[4];

uint8_t byte_1, byte_2;

EasyTransfer ETout_steer, ETout_drive;

struct STEER_DATA_STRUCTURE {
  unsigned short filter_period[4];
};

struct DRIVE_DATA_STRUCTURE {
  unsigned short filter_period[4];
};

STEER_DATA_STRUCTURE tx_steer;
DRIVE_DATA_STRUCTURE tx_drive;

void setup() {
  Serial.begin(115200);

  //ETout_steer.begin(details(tx_steer), &Serial);
  ETout_drive.begin(details(tx_drive), &Serial);

  cli();  // Turn off interrupts temporarily

  PCICR |= 0b00000101;    // turn on ports b and d
  PCMSK0 |= 0b00000001;   // turn on pins PB0 and PB1 -- PCINT0 & PCINT1, physical ATMEGA pin 14 & 15, UNO pins 8 & 9
  PCMSK2 |= 0b01000000;   // Pins for Port D instead

  sei();  // Re-open interrupts

  attachInterrupt(0, encoderA, CHANGE);
  attachInterrupt(1, encoderB, CHANGE);

  //pinMode(2, INPUT);
  //pinMode(3, INPUT);
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

  //byte_1 = 0;
  //byte_2 = 0;

  //specified_bit = 0;
  
/*
  randNumber = random(65536);

  for (int i = 15; i >= 0; i--) {
    specified_bit = randNumber >> i;
  
    if ((specified_bit & 1) && (i >= 8 && i < 16)) byte_1 |= 1UL << i - 8;
    else if ((specified_bit & 1) && (i < 8)) byte_2 |= 1UL << i;
  }

  motor_speed[0] = byte_1 << 8 | byte_2;

  Serial.print("Decimal: ");
  Serial.print(randNumber);
  Serial.print("\t");
  Serial.print("Binary: ");
  Serial.print(byte_1);
  Serial.print(" ");
  Serial.print(byte_2);
  Serial.print("\t");
  Serial.print("Shifted: ");
  Serial.print(" ");
  Serial.print(motor_speed[0]);
  Serial.println();
*/

  for (int i = 0; i < 4; i++) {
    if (stop_time[i] > start_time[i]) period[i] = stop_time[i] - start_time[i];   // Measure period between pulse (Time duration LOW or HIGH)
    else period[i] = start_time[i] - stop_time[i];
    
    if (period[i] > 15000) period[i] = 15000;   // Cap period to 15000 micros or 15 ms -- MAX 16 bit = 65,536 Decimal

    comp_period[i] = alpha * (comp_period[i]) + ((1 - alpha) * period[i]);    // Low pass filter to attenuate spike measurements
    tx_drive.filter_period[i] = comp_period[i];

    ETout_drive.sendData();
/*
    for (int j = 15; j >= 0; j--) {
      specified_bit = comp_period[i] >> j;
    
      if ((specified_bit & 1) && (j >= 8 && j < 16)) byte_1 |= 1UL << j - 8;
      else if ((specified_bit & 1) && (j < 8)) byte_2 |= 1UL << j;
    }
*/
    //data_out[0] = byte('$');
    //data_out[1] = i;
    //data_out[2] = byte_1;
    //data_out[3] = byte_2;
    //data_out[4] = motor_dir[i];
    //data_out[5] = byte('%');

    for (int k = 0; k < 6; k++) {
      //Serial.write(data_out[k]);
      //delay(1);
    }

    //Serial.print("Period: ");
    //Serial.print(comp_period[i]);
    //Serial.print("\t");
    //Serial.print("Dir: ");
    //Serial.print(motor_dir[i]);
    //Serial.print("\t");

    //motor_speed[i] = byte_1 << 8 | byte_2;
    //Serial.print(motor_speed[i]);
    //Serial.print("\t");
    //Serial.print(comp_period[i]);
    //Serial.print("\t");
  }
  //delay(1);

  //Serial.print("\t");
  //Serial.print(motor_speed[1]);
  //Serial.print("\t");
  //Serial.print(comp_period[1]);
  //Serial.print("\t");
  //Serial.print(motor_speed[2]);
  //Serial.print("\t");
  //Serial.print(comp_period[2]);
  
  //Serial.print(comp_period[2]);
  //Serial.print("\t");
  //Serial.print(byte_1);
  //Serial.print("\t");
  //Serial.print(byte_2);
  //Serial.print("\t");
  //Serial.print(motor_speed[2]);
  //Serial.println();

  //Serial.write(byte('%'));

  //long uno_pulse = pulseIn(3,HIGH);
  //if (uno_pulse > 1000) uno_pulse = 1000;

  //Serial.print(uno_pulse);

  //Serial.print(start_time[0]);
  //Serial.print("\t");
  //Serial.print(stop_time[0]);

  /*
    for (int i = 0; i < 8; i++) {
    if (PINB & (0x01 << i)) Serial.print(1);
    else Serial.print(0);
    }
  */

  /*
    for (int i = 0; i < 2; i++) {
    ext_int_period[i] = current_pulse[i] - previous_pulse[i];

    Serial.print(ext_int_period[i]);
    Serial.print("\t");
    }
  */
  //Serial.print(val);
}

long runningAverage(unsigned long value) {
  #define LM_SIZE 10
  static int LM[LM_SIZE];
  static byte index = 0;
  static long sum = 0;
  static byte count = 0;

  static int data_points[10];

  return sum / count;
}

/////
// Caution: 1st and 3rd encoders are flipped currently!!
/////

ISR(PCINT0_vect) {
  /*
    for (int i = 0; i < PORT_B_pins; i += 2) {
    if (PINB & (0x01 << i)) start_time[i + PORT_D_pins] = micros();
    else stop_time[i + PORT_D_pins] = micros();
    }
  */

  //if (PINB & (0x01 << 0)) start_time[3] = micros();
  //else stop_time[3] = micros();

  if (PINB & (0x01 << 0)) {
    start_time[3] = micros();

    if (PINC & (0x01 << 3)) motor_dir[3] = 1;
    else motor_dir[3] = 0;
  }

  else {
    stop_time[3] = micros();

    if (PINC & (0x01 << 3)) motor_dir[3] = 0;
    else motor_dir[3] = 1;
  }
}

ISR(PCINT2_vect) {    // Port D, PCINT16 - PCINT23
  /*
    for (int i = 0; i < PORT_D_pins; i +=2) {
    if (PIND & (0x01 << i + PORT_B_pins)) start_time[i] = micros();
    else stop_time[i] = micros();
    }
  */

  /*
    if (PIND & (0x01 << 6)) start_time[0] = micros();
    else stop_time[0] = micros();

    if (PIND & (0x01 << 4)) start_time[1] = micros();
    else stop_time[1] = micros();

    if (PIND & (0x01 << 2)) start_time[2] = micros();
    else stop_time[2] = micros();
  */

  if (PIND & (0x01 << 6)) {
    start_time[2] = micros();

    if (PINC & (0x01 << 2)) motor_dir[2] = 1;
    else motor_dir[2] = 0;
  }
  else {
    stop_time[2] = micros();

    if (PINC & (0x01 << 2)) motor_dir[2] = 0;
    else motor_dir[2] = 1;
  }
}

void encoderA() {
  if (PIND & (0x01 << 2)) {
    start_time[0] = micros();

    if (PINC & (0x01 << 0)) motor_dir[0] = 1;
    else motor_dir[0] = 0;
  }

  else {
    stop_time[0] = micros();

    if (PINC & (0x01 << 0)) motor_dir[0] = 0;
    else motor_dir[0] = 1;
  }
}

void encoderB() {
  if (PIND & (0x01 << 3)) {
    start_time[1] = micros();

    if (PINC & (0x01 << 1)) motor_dir[1] = 1;
    else motor_dir[1] = 0;
  }

  else {
    stop_time[1] = micros();

    if (PINC & (0x01 << 1)) motor_dir[1] = 0;
    else motor_dir[1] = 1;
  }
}

