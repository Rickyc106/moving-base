#include <ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <moving_base/arduino_status.h>
#include <tf/tf.h>
#include <Servo.h>

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

// STEER MOTORS
#define M8_pin 2;
#define M9_pin 3;
#define M10_pin 4;
#define M11_pin 5;

// DRIVE MOTORS
#define M12_pin 6;
#define M13_pin 7;
#define M14_pin 8;
#define M15_pin 9;

Servo motors[8];
unsigned long previousMillis = 0;

int var[4];
float aState, bState, aPrev, bPrev, aCount, bCount;
float drive_speed;
float steer_speed;

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
  }

  unsigned long currentMillis = millis();

  //steer_speed = mapf(msg.axes[2], -1.0, 1.0, 1.0, 0);

  drive_speed = msg.axes[1];

  for (int i = 0; i < 4; i++) {
    if (msg.buttons[i]) var[i] = i;
    motor_debugging(var[i], steer_speed, drive_speed);
  }

  if (!msg.buttons[0] && !msg.buttons[1] && !msg.buttons[2] && !msg.buttons[3]) motor_stop();

  //motors[3].writeMicroseconds(1500 + (val * 500));    // STEER MOTOR -- 1.45 max
  //motors[7].writeMicroseconds(1500 + (msg.axes[1] * 500 * 0.35));   // DRIVE MOTOR -- 32 max right now

  if (currentMillis - previousMillis > 20) {
    previousMillis = currentMillis;
  }
  
  // FINISH TRIG MATH -----------------------------------------------------------------

  steer_angle *= (180 / 3.141592653);

  temp.data = aCount;

  temp_array.data = resultant;
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
ros::Publisher mag_array_pub("mag_array_data", &mag_array);
//ros::Publisher status_pub("arduino_status", &stats);

void setup() {
  //nh.getHardware()->setBaud(115200); // Default 57600

  initialize();
  nh.initNode();

  nh.subscribe(joy_sub);
  nh.advertise(temp_pub);
  nh.advertise(testing_pub);
  nh.advertise(temp_array_pub);
  nh.advertise(mag_array_pub);
  //nh.advertise(status_pub);
}

void loop() {
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
  /*
    temp_array.layout.dim = (std_msgs::MultiArrayDimension *)
    malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    temp_array.layout.dim[0].size = 4;
    temp_array.layout.dim[0].stride = 1*4;
    temp_array.layout.data_offset = 0;
    temp_array.data = (float *)malloc(sizeof(float)*4);
    temp_array.layout.dim_length = 1;
    temp_array.data_length = 4;
  */

  temp_array.data_length = 4;
  //array_angle.data_length = 4;
  mag_array.data_length = 4;

  rotate_angle[0] -= M_PI / 4;
  rotate_angle[1] += M_PI / 4;
  rotate_angle[2] -= M_PI / 4;
  rotate_angle[3] += M_PI / 4;

  for (int i = 0; i < 8; i++) {
    motors[i].attach(i + 2);
    pinMode(i + 2, OUTPUT);
  }

  pinMode(22, INPUT);
  pinMode(23, INPUT);

  attachInterrupt(22, read_encoder, RISING);
  //attachInterrupt(23, read_encoder, RISING);
}

float vector_sum(float steer_angle, float steer_mag, float rotate_angle, float rotate_mag, int counter) {
  res_x = (steer_mag * cos(steer_angle)) + (rotate_mag * cos(rotate_angle));
  res_y = (steer_mag * sin(steer_angle)) + (rotate_mag * sin(rotate_angle));

  resultant_angle[counter] = atan2(res_y, res_x);
  resultant_angle[counter] *= 180 / M_PI;

  //resultant_angle[counter + 4] = sqrt(res_x * res_x + res_y * res_y);

  return sqrt(res_x * res_x + res_y * res_y);
}

void motor_debugging(int var, float val, float drive) {
  switch (var) {
    case 0:
      motors[0].writeMicroseconds(1500 + (val * 500));    // STEER MOTOR -- 1.45 max
      motors[4].writeMicroseconds(1500 + (drive * 500 * 0.35));   // DRIVE MOTOR -- 32 max right now
      break;
    case 1:
      motors[1].writeMicroseconds(1500 + (val * 500));    // STEER MOTOR -- 1.45 max
      motors[5].writeMicroseconds(1500 + (drive * 500 * 0.35));   // DRIVE MOTOR -- 32 max right now
      break;
    case 2:
      motors[2].writeMicroseconds(1500 + (val * 500));    // STEER MOTOR -- 1.45 max
      motors[6].writeMicroseconds(1500 + (drive * 500 * 0.35));   // DRIVE MOTOR -- 32 max right now
      break;
    case 3:
      motors[3].writeMicroseconds(1500 + (val * 500));    // STEER MOTOR -- 1.45 max
      motors[7].writeMicroseconds(1500 + (drive * 500 * 0.35));   // DRIVE MOTOR -- 32 max right now
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

void read_encoder(){
  aCount++;
  //if (value == 1) bCount++;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

