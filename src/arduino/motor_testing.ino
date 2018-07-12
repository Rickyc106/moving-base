#include <ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
//#include <moving_base/arduino_status.h>
#include <tf/tf.h>

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
std_msgs::Float32MultiArray temp_array;
//std_msgs::Float32MultiArray array_angle;
//moving_base::arduino_status stats;

void xbox_cb(const sensor_msgs::Joy& msg) {
  steer_angle = atan2(msg.axes[1], -msg.axes[0]);

  if (msg.buttons[4] && !msg.buttons[5]) {
    CCW = true;
    no_rotate = false;
  }
  if (msg.buttons[5] && !msg.buttons[4]) {
    CCW = false;
    no_rotate = false;
  }
  if (!msg.buttons[4] && !msg.buttons[5]) no_rotate = true;

  // DO ALL TRIG MATH HERE ------------------------------------------------------------

  x = cos(steer_angle) * -msg.axes[0];
  y = sin(steer_angle) * msg.axes[1];

  steer_mag = sqrt(x * x + y * y);

  for (int i = 0; i < 4; i++) {
    if (CCW && !no_rotate) {
      rotate_angle[i] += M_PI / 180;
      rotate_mag[i] = 1.0;
    }
    else if (!CCW && !no_rotate) {
      rotate_angle[i] -= M_PI / 180;
      rotate_mag[i] = 1.0;
    }

    if (i == 0 || i == 1) rotate_mag[i] = -1.0;

    if (no_rotate) {
      rotate_mag[i] = 0.0;
    }

    if (steer_mag != 0.0 || rotate_mag[i] != 0.0) {
      resultant[i] = vector_sum(steer_angle, steer_mag, rotate_angle[i], rotate_mag[i], i);
    }
  }

  // FINISH TRIG MATH -----------------------------------------------------------------

  steer_angle *= (180 / 3.141592653);

  temp_array.data = resultant_angle;
  //array_angle.data = rotate_angle;
  temp.data = rotate_angle[0] * (180 / M_PI);
  //stats.steer_angle = angle;
}

ros::Subscriber < sensor_msgs::Joy > joy_sub("joy_queued", &xbox_cb);
ros::Publisher temp_pub("temp_data", &temp);
ros::Publisher temp_array_pub("temp_array_data", &temp_array);
//ros::Publisher array_angle_pub("array_angle", &array_angle);
//ros::Publisher status_pub("arduino_status", &stats);

void setup() {
  //nh.getHardware()->setBaud(115200); // Default 57600

  initialize();
  nh.initNode();

  nh.subscribe(joy_sub);
  nh.advertise(temp_pub);
  nh.advertise(temp_array_pub);
  //nh.advertise(array_angle_pub);
  //nh.advertise(status_pub);
}

void loop() {
  temp_pub.publish(&temp);
  temp_array_pub.publish(&temp_array);
  //array_angle_pub.publish(&array_angle);
  //status_pub.publish(&stats);

  nh.spinOnce();
  //delay(100);
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

  rotate_angle[0] -= M_PI / 4;
  rotate_angle[1] += M_PI / 4;
  rotate_angle[2] -= M_PI / 4;
  rotate_angle[3] += M_PI / 4;
}

float vector_sum(float steer_angle, float steer_mag, float rotate_angle, float rotate_mag, int counter) {
  res_x = (steer_mag * cos(steer_angle)) + (rotate_mag * cos(rotate_angle));
  res_y = (steer_mag * sin(steer_angle)) + (rotate_mag * sin(rotate_angle));

  resultant_angle[counter] = atan2(res_y, res_x);
  resultant_angle[counter] *= 180 / M_PI;

  return sqrt(res_x * res_x + res_y * res_y);
}

