#include <ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <moving_base/arduino_status.h>
#include <tf/tf.h>
#include <Servo.h>
#include <avr/interrupt.h>

ros::NodeHandle nh;

float x, y;
float steer_angle;
float steer_mag, val;

float rotate_angle[4];
float rotate_mag[4];

float res_x, res_y;
float resultant_angle[4];
float resultant[4];

float error;
float sum;
float previous;

float output;
float p_gain, i_gain, d_gain;

bool CCW, no_rotate;
bool forwards[4];

std_msgs::Float32 temp;
std_msgs::Float32 testing;
std_msgs::Float32MultiArray temp_array;
std_msgs::Float32MultiArray mag_array;
//moving_base::arduino_status stats;

// ------------------------------------------------------------------------------------

bool enc_CW = false;
bool enc_CCW = false;
int CW_pulse, CCW_pulse;
bool stop_signal = false;

Servo motors[8];
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

int var[4];
float aState, bState, aPrev, bPrev, aCount, bCount;
float drive_speed;
float steer_speed;

volatile float pulse_count[16];   // Steer: 0-7, Drive 8-15
volatile int previous_pulse[16];   // Stores previous pulse -- One for each pair of quadrature signals

byte data_in[3];
uint8_t comp_period[8];

double motor_speed[8];
bool motor_dir[8];

void xbox_cb(const sensor_msgs::Joy& msg) {
  steer_angle = atan2(msg.axes[1], -msg.axes[0]);

  if ((msg.buttons[4] && !msg.buttons[5]) || (msg.buttons[6] && !msg.buttons[7])) {
    CCW = true;
    no_rotate = false;
  }
  if ((msg.buttons[5] && !msg.buttons[4]) || (msg.buttons[7] && !msg.buttons[6])) {
    CCW = false;
    no_rotate = false;
  }
  if (!msg.buttons[4] && !msg.buttons[5] && !msg.buttons[6] && !msg.buttons[7]) no_rotate = true;

  // DO ALL TRIG MATH HERE ------------------------------------------------------------

  x = cos(steer_angle) * -msg.axes[0];
  y = sin(steer_angle) * msg.axes[1];

  steer_mag = sqrt(x * x + y * y);

  for (int i = 0; i < 4; i++) {
    if (CCW && !no_rotate) {
      rotate_angle[i] += M_PI / 22.5;
      rotate_mag[i] = 1.0;

      steer_speed = mapf(msg.axes[2], -1.0, 1.0, 1.0, 0);
      //val = 0.5;
    }
    else if (!CCW && !no_rotate) {
      rotate_angle[i] -= M_PI / 22.5;
      rotate_mag[i] = 1.0;

      steer_speed = mapf(msg.axes[5], -1.0, 1.0, -1.0, 0);
      //val = -0.5;
    }

    if (i == 0 || i == 1) rotate_mag[i] = -1.0;

    if (no_rotate) {
      rotate_mag[i] = 0.0;

      steer_speed = 0.0;
      //val = 0.0;
    }

    if (steer_mag != 0.0 || rotate_mag[i] != 0.0) {
      resultant[i] = vector_sum(steer_angle, steer_mag, rotate_angle[i], rotate_mag[i], i);
    }

    if ((msg.buttons[6] && (currentMillis - previousMillis > 5000)) || (msg.buttons[7] && (currentMillis - previousMillis > 5000))) {
      stop_signal = true;
      previousMillis = currentMillis;
    }
  }

  currentMillis = millis();

  //steer_speed = mapf(msg.axes[2], -1.0, 1.0, 1.0, 0);

  drive_speed = msg.axes[1] * 0.40;

  //uint8_t oldSREG = SREG;
  //cli(); 

  for (int i = 0; i < 4; i++) {
    if (msg.buttons[i]) var[i] = i;
  }
    
  //SREG = oldSREG;
  //sei();

  if (!msg.buttons[0] && !msg.buttons[1] && !msg.buttons[2] && !msg.buttons[3]) motor_stop();

  if (!msg.buttons[6] && !msg.buttons[7]) stop_signal = false;

  //motors[3].writeMicroseconds(1500 + (val * 500));    // STEER MOTOR -- 1.45 max
  //motors[7].writeMicroseconds(1500 + (msg.axes[1] * 500 * 0.35));   // DRIVE MOTOR -- 32 max right now

  /*
    if (currentMillis - previousMillis > 20) {
    previousMillis = currentMillis;
    }
  */

  // FINISH TRIG MATH -----------------------------------------------------------------

  steer_angle *= (180 / 3.141592653);

  temp.data = data_in[2];
  
  temp_array.data = resultant;  // WARNING: DO NOT CHANGE RIGHT NOW
  testing.data = 90 + msg.axes[1] * 90;
  mag_array.data = resultant;
  //temp.data = rotate_angle[0] * (180 / M_PI);
  //temp.data = 90 + val * 90;
  //stats.steer_angle = steer_angle;
}

ros::Subscriber < sensor_msgs::Joy > joy_sub("joy_queued", &xbox_cb);
ros::Publisher temp_pub("temp_data", &temp);
ros::Publisher testing_pub("testing", &testing);
ros::Publisher temp_array_pub("temp_array_data", &temp_array);
//ros::Publisher mag_array_pub("mag_array_data", &mag_array);
//ros::Publisher status_pub("arduino_status", &stats);

void setup() {
  //nh.getHardware()->setBaud(115200); // Default 57600

  initialize();
  
  nh.initNode();

  nh.subscribe(joy_sub);
  nh.advertise(temp_pub);
  nh.advertise(testing_pub);
  nh.advertise(temp_array_pub);
  //nh.advertise(mag_array_pub);
  //nh.advertise(status_pub);
  
}

void loop() {
  //drive_speed = mapf(analogRead(A0), 1, 1000, 0, 1);

  if (Serial1.available()) {
    while (Serial1.available() > 0) {
      byte incomingByte = Serial1.read();

      if (incomingByte == 'x') {
        for (int i = 0; i < 3; i++) {
          data_in[i] = Serial1.read();
        }

        switch (data_in[1]) {
        // Steer motors ----------------------------------------------
          case 0:
            motor_speed[0] = data_in[2] << 8 | data_in[3];
            motor_dir[0] = data_in[4];
            break;
          case 1:
            motor_speed[1] = data_in[2] << 8 | data_in[3];
            motor_dir[1] = data_in[4];
            break;
          case 2:
            motor_speed[2] = data_in[2] << 8 | data_in[3];
            motor_dir[2] = data_in[4];
            break;
          case 3:
            motor_speed[3] = data_in[2] << 8 | data_in[3];
            motor_dir[3] = data_in[4];
            break;

        // Drive motors ----------------------------------------------
        
          case 4:
            motor_speed[4] = data_in[2] << 8 | data_in[3];
            motor_dir[4] = data_in[4];
            break;
          case 5:
            motor_speed[5] = data_in[2] << 8 | data_in[3];
            motor_dir[5] = data_in[4];
            break;
          case 6:
            motor_speed[6] = data_in[2] << 8 | data_in[3];
            motor_dir[6] = data_in[4];
            break;
          case 7:
            motor_speed[7] = data_in[2] << 8 | data_in[3];
            motor_dir[7] = data_in[4];
            break; 
        }
      }
    }  
  }

  for (int i = 0; i < 4; i++) {
    motor_debugging(var[i], steer_speed, drive_speed, stop_signal);  
  }

  /*
  
  for (int i = 0; i < 4; i++) {
    if (i == 1) motor_debugging(i, 0, drive_speed, false); 
    else motor_debugging(i, 0, 0, false); 

    //motor_debugging(i, 0, drive_speed, false);
  }
  */
  
  temp_pub.publish(&temp);
  testing_pub.publish(&testing);
  temp_array_pub.publish(&temp_array);
  //mag_array_pub.publish(&mag_array);
  //status_pub.publish(&stats);

  nh.spinOnce();
}

void PID(float setpoint, float current_angle) {
  error = setpoint - current_angle;
  sum += error;

  if (sum > 500) sum = 500;
  else if (sum < -500) sum = -500;

  output = p_gain * error + i_gain * sum + d_gain * (previous - error);
  previous = error;
}

/*
  void initialize() {
  for (int i = 0; i < 4; i++) {
    PID(0, stats.steer_motor[i]);    // Set all steer motors back to zero position
  }

  stats.steer_motor[0] -= 45;
  stats.steer_motor[1] += 45;
  stats.steer_motor[2] -= 45;
  stats.steer_motor[3] += 45;

  if (CCW) {
    forwards[0] = false;
    forwards[1] = false;
    forwards[2] = true;
    forwards[3] = true;
  }
  else if (!CCW) {
    forwards[0] = true;
    forwards[1] = true;
    forwards[2] = false;
    forwards[3] = false;
  }
  }
*/

void initialize() {
  Serial1.begin(9600);
  Serial2.begin(9600);
  
  temp_array.data_length = 4;
  //array_angle.data_length = 4;
  //mag_array.data_length = 4;

  rotate_angle[0] -= M_PI / 4;
  rotate_angle[1] += M_PI / 4;
  rotate_angle[2] -= M_PI / 4;
  rotate_angle[3] += M_PI / 4;
  
  cli();  // Turn off interrupts temporarily

  PCICR |= 0b00000101;  // PCICR - Pin Change Interrupt Control Register Mapping
                        //
                        // Bit 0 - Opens PCINT7:0
                        // Bit 1 - Opens PCINT15:8
                        // Bit 2 - Opens PCINT23:16
                        //
                        //Currently using PCINT23:16 and PCINT 7:0 aka bits 0 and 2

  PCMSK0 |= 0b11111111; // Mask pins PCINT7:0 -- Arduino pins 10-13 & 50-53
  PCMSK2 |= 0b11111111; // Mask pins PCINT 23:16 -- Arduino pins A8-A15

  sei();  // Re-open interrupts

  for (int i = 0; i < 8; i++) {
    motors[i].attach(i + 32);   // Attach motors -- Pins 32 -> 39
    pinMode(i + 32, OUTPUT); // 22

    //if (i < 4) pinMode(i + 10, INPUT);   // Half of steer encoders -- Arduino Mega pins 10-13
    //else pinMode(i + 46, INPUT);   // Remaining half of steer encoders -- Arduino Mega pins 50-53
  }

  DDRK = 0b00000000;
  pinMode(A0, INPUT);
}

float vector_sum(float steer_angle, float steer_mag, float rotate_angle, float rotate_mag, int counter) {
  res_x = (steer_mag * cos(steer_angle)) + (rotate_mag * cos(rotate_angle));
  res_y = (steer_mag * sin(steer_angle)) + (rotate_mag * sin(rotate_angle));

  resultant_angle[counter] = atan2(res_y, res_x);
  resultant_angle[counter] *= 180 / M_PI;

  //resultant_angle[counter + 4] = sqrt(res_x * res_x + res_y * res_y);

  return sqrt(res_x * res_x + res_y * res_y);
}

void motor_debugging(int var, float val, float drive, bool stopping_signal) {
  switch (var) {
    case 0:
      motors[0].writeMicroseconds(1500 + (drive * 500 * 0.35));    // DRIVE MOTOR -- 32 max right now
      motors[1].writeMicroseconds(1500 + (val * 500 * 0.35));   // STEER MOTOR -- 1.45 max
      break;
    case 1:
      motors[2].writeMicroseconds(1500 + (drive * 500 * 0.35));    // DRIVE MOTOR -- 32 max right now
      motors[3].writeMicroseconds(1500 + (val * 500 * 0.35));   // STEER MOTOR -- 1.45 max
      break;
    case 2:
      //if (!stopping_signal) motors[2].writeMicroseconds(1500 + (val * 500 * 0.5));    // STEER MOTOR -- 1.45 max
      motors[4].writeMicroseconds(1500 + (drive * 500 * 0.35));    // DRIVE MOTOR -- 32 max right now
      motors[5].writeMicroseconds(1500 + (val * 500 * 0.35));   // STEER MOTOR -- 1.45 max
      break;
    case 3:
      motors[6].writeMicroseconds(1500 + (drive * 500 * 0.35));    // DRIVE MOTOR -- 32 max right now
      motors[7].writeMicroseconds(1500 + (val * 500 * 0.35));   // STEER MOTOR -- 1.45 max
      break;
  }
}

void motor_stop() {
  for (int i = 0; i < 4; i++) {
    motors[i].writeMicroseconds(1500);    // STEER MOTOR -- 1.45 max
    motors[i + 4].writeMicroseconds(1500);   // DRIVE MOTOR -- 32 max right now

    var[i] = 1000;  // Reset switch cases for all motors
  }
}

void read_encoder() {
  if (digitalRead(22) && digitalRead(23) == 0) {
    enc_CW = true;
    CW_pulse++;
  }
  else if (digitalRead(22) && digitalRead(23) == 1) {
    enc_CCW = true;
    CCW_pulse++;
  }
  CW_pulse++;
}

// Following are Interrupt Service Routines -- Functions same as external interrupts
// ... but only detects changes on specified port

// First need to read which pin caused interrupt
// Then whether or not it was falling or rising

ISR(PCINT0_vect) {  // Port B, PCINT7:0
  /*
  for (int i = 0; i < 4; i += 2) {
    if (!digitalRead(i) && previous_pulse[i + 1] == 0 && i < 4) enc_CW = true;
    if (!digitalRead(i) && previous_pulse[i + 1] == 1 && i < 4) enc_CW = false;
    
    if (digitalRead(i) && previous_pulse[i + 1] == 1 && i < 4) enc_CW = true;
    if (digitalRead(i) && previous_pulse[i + 1] == 0 && i < 4) enc_CW = false; 
  }

  for (int i = 0; i < 4; i++) {
    pulse_count[i]++;
    previous_pulse[i] = digitalRead(i + 50);
  }
  for (int i = 0; i < 4; i++) { 
    pulse_count[i + 4]++;
    previous_pulse[i + 4] = digitalRead(i + 10);
  }
  */

  for (int i = 0; i < 8; i++) {
    pulse_count[i]++;  

    if (i % 2 && pulse_count[i] > pulse_count[i + 1]) enc_CW = true;
    else if (i % 2 && pulse_count[i] < pulse_count[i + 1]) enc_CW = false;
  }
}

ISR(PCINT2_vect) {  // Port K, PCINT23:16
  //pulse_count[8]++;  // Arduino Pins A8-A15

  //pulse_count[8] = bitRead(PORTK, 0);

  //pulse_count[8] = digitalRead(A8);

  
  for (int i = 0; i < 0; i++) {
    pulse_count[i + 8] = PINK & (0x01 << i);  
  }
  
  
  /*
  if (digitalRead(A9)) pulse_count[9]++;
  if (digitalRead(A10)) pulse_count[10]++;
  if (digitalRead(A11)) pulse_count[11]++;
  
  if (digitalRead(A12)) pulse_count[12]++;
  if (digitalRead(A13)) pulse_count[13]++;
  if (digitalRead(A14)) pulse_count[14]++;
  if (digitalRead(A15)) pulse_count[15]++;
  */
  
  for (int i = 8; i < 16; i += 2) {
    if (pulse_count[i] > pulse_count[i + 1]) enc_CW = true;
    else enc_CCW = false;
  }
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

