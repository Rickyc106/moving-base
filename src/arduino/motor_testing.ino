#include <ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
//#include <moving_base/arduino_status.h>

ros::NodeHandle nh;

double steer_angle;
double x, y;
double steer_mag;

double rotate_angle[4];
double rotate_mag[4];

double res_x, res_y;
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
//moving_base::arduino_status stats;

void xbox_cb(const sensor_msgs::Joy& msg) {
  steer_angle = atan2(msg.axes[1], -msg.axes[0]);

  // DO ALL TRIG MATH HERE ------------------------------------------------------------

  x = cos(steer_angle) * -msg.axes[0];
  y = sin(steer_angle) * msg.axes[1];

  steer_mag = sqrt(x*x + y*y);

  for (int i = 0; i < 4; i++) {
    rotate_mag[i] = 1.0;
    resultant[i] = vector_sum(steer_angle, steer_mag, rotate_angle[i], rotate_mag[i]);
  }

  // FINISH TRIG MATH -----------------------------------------------------------------

  steer_angle *= (180 / 3.141592653);

  temp_array.data = resultant;
  temp.data = resultant[0];
  //stats.steer_angle = angle;

  if (msg.buttons[4] && !msg.buttons[5]) {
    CCW = true;
    no_rotate = false;
  }
  if (msg.buttons[5] && !msg.buttons[4]) {
    CCW = false;
    no_rotate = false;
  }
  if (!msg.buttons[4] && !msg.buttons[5]) no_rotate = true;
}

ros::Subscriber < sensor_msgs::Joy > joy_sub("joy_queued", &xbox_cb);
ros::Publisher temp_pub("temp_data", &temp);
ros::Publisher temp_array_pub("temp_array_data", &temp_array);
//ros::Publisher status_pub("arduino_status", &stats);

void setup() {
  //nh.getHardware()->setBaud(115200); // Default 57600

  initialize();
  nh.initNode();
  
  nh.subscribe(joy_sub);
  nh.advertise(temp_pub);
  nh.advertise(temp_array_pub);
  //nh.advertise(status_pub);
}

void loop() {
  temp_pub.publish(&temp);
  temp_array_pub.publish(&temp_array);
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
  temp_array.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  temp_array.layout.dim[0].size = 4;
  temp_array.layout.dim[0].stride = 1*4;
  temp_array.layout.data_offset = 0;
  temp_array.data = (float *)malloc(sizeof(float)*4);
  temp_array.layout.dim_length = 1;
  temp_array.data_length = 4;
  
  rotate_angle[0] -= 45;
  rotate_angle[1] += 45;
  rotate_angle[2] -= 45;
  rotate_angle[3] += 45;
}

float vector_sum(float steer_angle, float steer_mag, float rotate_angle, float rotate_mag) {
  res_x = (steer_mag * cos(steer_angle)) + (rotate_mag * cos(rotate_angle));
  res_y = (steer_mag * sin(steer_angle)) + (rotate_mag * sin(rotate_angle));

  return sqrt(res_x*res_x + res_y*res_y);
}

