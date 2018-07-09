#include <ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

float* axes;
long int* buttons;

float angle;

float error;
float sum;
float previous;

float output;
float p_gain, i_gain, d_gain;

bool CCW, no_rotate;
bool forwards[4];

std_msgs::Int32 counter;
std_msgs::Float32 left_joy;

std_msgs::Float32MultiArray steer_motor;
std_msgs::Float32MultiArray drive_motor;

//sensor_msgs::Joy joy_msg;

void xbox_cb(const sensor_msgs::Joy& msg) {
  axes = msg.axes;
  buttons = msg.buttons;
  
  angle = atan2(msg.axes[1], -msg.axes[0]);
  angle *= (180 / 3.141592653);
  left_joy.data = angle;

  if (buttons[4] && !buttons[5]) {
    CCW = true;
    no_rotate = false;
  }
  if (buttons[5] && !buttons[4]) {
    CCW = false;
    no_rotate = false;
  }
  if (!buttons[4] && !buttons[5]) no_rotate = true;
}

ros::Subscriber < sensor_msgs::Joy > joy_sub("joy_queued", &xbox_cb);
ros::Publisher counter_pub("counter", &counter);
ros::Publisher steer_pub("steer_angle", &left_joy);
//ros::Publisher joy_pub("joy_arduino", &joy_msg);

void setup() {
  //nh.getHardware()->setBaud(115200); // Default 57600
  
  nh.initNode();
  
  nh.subscribe(joy_sub);
  nh.advertise(counter_pub);
  nh.advertise(steer_pub);
  //nh.advertise(joy_pub);
}

void loop() {
  if (axes[0] != 0) {
    counter.data += 1;  
  }

  //joy_msg.axes = axes;
  //joy_msg.buttons = buttons;

  counter_pub.publish(&counter);
  steer_pub.publish(&left_joy);
  //joy_pub.publish(&joy_msg);
  
  nh.spinOnce();
  //delay(1);
}

void PID(float setpoint, float current_angle) {
  error = setpoint - current_angle;
  sum += error;

  if (sum > 500) sum = 500;
  else if (sum < -500) sum = -500;

  output = p_gain * error + i_gain * sum + d_gain * (previous - error);
  previous = error;
}

void initialize() {
  for (int i = 0; i < 4; i++) {
    PID(0, steer_motor.data[i]);    // Set all steer motors back to zero position 
  }
  
  steer_motor.data[0] -= 45;
  steer_motor.data[1] += 45;
  steer_motor.data[2] -= 45;
  steer_motor.data[3] += 45;

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


