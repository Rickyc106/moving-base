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
float steer_mag;

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

byte data_in[5], data_in_drive[5];
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
    }
    else if (!CCW && !no_rotate) {
      rotate_angle[i] -= M_PI / 22.5;
      rotate_mag[i] = 1.0;

      steer_speed = mapf(msg.axes[5], -1.0, 1.0, -1.0, 0);
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
  }

  drive_speed = msg.axes[1] * 0.40;

  for (int i = 0; i < 4; i++) {
    if (msg.buttons[i]) var[i] = i;
  }

  if (!msg.buttons[0] && !msg.buttons[1] && !msg.buttons[2] && !msg.buttons[3]) motor_stop();

  // FINISH TRIG MATH -----------------------------------------------------------------

  steer_angle *= (180 / 3.141592653);

  temp.data = motor_speed[4];
  
  temp_array.data = resultant;  // WARNING: DO NOT CHANGE RIGHT NOW
  testing.data = 90 + msg.axes[1] * 90;
}

ros::Subscriber < sensor_msgs::Joy > joy_sub("joy_queued", &xbox_cb);

ros::Publisher temp_pub("temp_data", &temp);
ros::Publisher testing_pub("testing", &testing);
ros::Publisher temp_array_pub("temp_array_data", &temp_array);

void setup() {
  initialize();   // Attach motors, establish serial comms, etc.
  nh.initNode();  // Create ROS nodehandle

  nh.subscribe(joy_sub);    // Subscribe to joy controller message
  
  nh.advertise(temp_pub);       // Debugging purposes -- Float32
  nh.advertise(testing_pub);    // Debugging purposes -- Float32
  nh.advertise(temp_array_pub); // Debugging purposes -- Float32MultiArray
}

void loop() {
  if (Serial1.available()) {
    while (Serial1.available() > 0) {
      byte incomingByte = Serial1.read();

      if (incomingByte == '$') {
        for (int i = 0; i < 5; i++) {
          data_in[i] = Serial1.read();
        }

        switch (data_in[0]) {
        // STEER MOTORS ----------------------------------------------
          case 0:
            motor_speed[0] = data_in[1] << 8 | data_in[2];
            motor_dir[0] = data_in[3];
            break;
          case 1:
            motor_speed[1] = data_in[1] << 8 | data_in[2];
            motor_dir[1] = data_in[3];
            break;
          case 2:
            motor_speed[2] = data_in[1] << 8 | data_in[2];
            motor_dir[2] = data_in[3];
            break;
          case 3:
            motor_speed[3] = data_in[1] << 8 | data_in[2];
            motor_dir[3] = data_in[3];
            break;
        }
      }

      if (data_in[4] == '%') break;
    }
  }

  if (Serial2.available()) {
    while (Serial2.available() > 0) {
      byte incomingByte = Serial2.read();

      if (incomingByte == '$') {
        for (int i = 0; i < 5; i++) {
          data_in_drive[i] = Serial2.read();
        }

        switch (data_in_drive[0]) {
        // DRIVE MOTORS ----------------------------------------------
          case 0:
            motor_speed[4] = data_in_drive[1] << 8 | data_in_drive[2];
            motor_dir[4] = data_in_drive[3];
            break;
          case 1:
            motor_speed[5] = data_in_drive[1] << 8 | data_in_drive[2];
            motor_dir[5] = data_in_drive[3];
            break;
          case 2:
            motor_speed[6] = data_in_drive[1] << 8 | data_in_drive[2];
            motor_dir[6] = data_in_drive[3];
            break;
          case 3:
            motor_speed[7] = data_in_drive[1] << 8 | data_in_drive[2];
            motor_dir[7] = data_in_drive[3];
            break;
        }
      }
      
      if (data_in_drive[4] == '%') break;
    }
  }

  for (int i = 0; i < 4; i++) {
    motor_debugging(var[i], steer_speed, drive_speed);  
  }
  
  temp_pub.publish(&temp);
  testing_pub.publish(&testing);
  temp_array_pub.publish(&temp_array);

  nh.spinOnce();
}

////////////////////////////////////////////////////////////////////
//        Unused currently -- Future Work in Progress             //
////////////////////////////////////////////////////////////////////

void PID(float setpoint, float current_angle) {
  error = setpoint - current_angle;
  sum += error;

  if (sum > 500) sum = 500;
  else if (sum < -500) sum = -500;

  output = p_gain * error + i_gain * sum + d_gain * (previous - error);
  previous = error;
}

////////////////////////////////////////////////////////////////////
//          Attach motors, establish serial comms, etc.           //
////////////////////////////////////////////////////////////////////

void initialize() {
  Serial1.begin(9600);
  Serial2.begin(9600);
  
  temp_array.data_length = 4;

  rotate_angle[0] -= M_PI / 4;
  rotate_angle[1] += M_PI / 4;
  rotate_angle[2] -= M_PI / 4;
  rotate_angle[3] += M_PI / 4;

  for (int i = 0; i < 8; i++) {
    motors[i].attach(i + 32);   // Attach motors -- Pins 32 -> 39
    pinMode(i + 32, OUTPUT); // 22
  }
}

////////////////////////////////////////////////////////////////////
//      Vector Math Subroutine -- Outputs resultant vectors       //
//     Two vector inputs: Steer (Joystick) & Rotate (Bumpers)     //
//   Sum of two vectors -> Resultant vector (magnitude and dir.)  //
////////////////////////////////////////////////////////////////////


float vector_sum(float steer_angle, float steer_mag, float rotate_angle, float rotate_mag, int counter) {
  res_x = (steer_mag * cos(steer_angle)) + (rotate_mag * cos(rotate_angle));
  res_y = (steer_mag * sin(steer_angle)) + (rotate_mag * sin(rotate_angle));

  resultant_angle[counter] = atan2(res_y, res_x);
  resultant_angle[counter] *= 180 / M_PI;

  //resultant_angle[counter + 4] = sqrt(res_x * res_x + res_y * res_y);

  return sqrt(res_x * res_x + res_y * res_y);
}

////////////////////////////////////////////////////////////////////
// Motor drive subroutine -- Range: 1000 ms - 2000 ms pulse width //
//     Variable 'var' corresponds to which motor is driven        //
//      Variabel 'steer' is the speed of PG71 Gearmotor           //
//        Variable 'drive' is the speed of CIM motor              //
////////////////////////////////////////////////////////////////////

void motor_debugging(int var, float steer, float drive) {
  switch (var) {
    case 0:
      motors[0].writeMicroseconds(1500 + (drive * 500 * 0.35));    // DRIVE MOTOR -- 32 max right now
      motors[1].writeMicroseconds(1500 + (steer * 500 * 0.35));   // STEER MOTOR -- 1.45 max
      break;
    case 1:
      motors[2].writeMicroseconds(1500 + (drive * 500 * 0.35));    // DRIVE MOTOR -- 32 max right now
      motors[3].writeMicroseconds(1500 + (steer * 500 * 0.35));   // STEER MOTOR -- 1.45 max
      break;
    case 2:
      motors[4].writeMicroseconds(1500 + (drive * 500 * 0.35));    // DRIVE MOTOR -- 32 max right now
      motors[5].writeMicroseconds(1500 + (steer * 500 * 0.35));   // STEER MOTOR -- 1.45 max
      break;
    case 3:
      motors[6].writeMicroseconds(1500 + (drive * 500 * 0.35));    // DRIVE MOTOR -- 32 max right now
      motors[7].writeMicroseconds(1500 + (steer * 500 * 0.35));   // STEER MOTOR -- 1.45 max
      break;
  }
}

////////////////////////////////////////////////////////////////////
//     Motor stop subroutine -- Stops all motors when called      //
//          TO-DO: Check to stop individual motors                //
////////////////////////////////////////////////////////////////////

void motor_stop() {
  for (int i = 0; i < 4; i++) {
    motors[i].writeMicroseconds(1500);      // Drive motor stop -- 1500 ms -> Neutral Position    
    motors[i + 4].writeMicroseconds(1500);  // Steer motor stop -- 1500 ms -> Neutral Position

    var[i] = 1000;  // Reset switch cases for all motors
  }
}

////////////////////////////////////////////////////////////////////
//    Regular arduino map function but works with float values    //
////////////////////////////////////////////////////////////////////

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

