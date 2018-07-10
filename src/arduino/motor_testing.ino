#include <ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
//#include <moving_base/arduino_status.h>

ros::NodeHandle nh;

float angle;
double resultant;

float error;
float sum;
float previous;

float output;
float p_gain, i_gain, d_gain;

bool CCW, no_rotate;
bool forwards[4];

std_msgs::Float32 temp;
//moving_base::arduino_status stats;

void xbox_cb(const sensor_msgs::Joy& msg) {
  angle = atan2(msg.axes[1], -msg.axes[0]);
  angle *= (180 / 3.141592653);

  resultant = sqrt(sq(msg.axes[1]) + sq(msg.axes[0]));
  
  //if (resultant > 1.0) resultant = 1.0;      // Crude solution for now...
  // if (sin(angle) > cos(angle)) resultant -= sin(angle) * (resultant - 1.0); 
  // else if (cos(angle) > sin(angle)) resultant -= cos(angle) * (resultant - 1.0); 
  
  temp.data = resultant;
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
//ros::Publisher status_pub("arduino_status", &stats);

void setup() {
  //nh.getHardware()->setBaud(115200); // Default 57600
  
  nh.initNode();
  
  nh.subscribe(joy_sub);
  nh.advertise(temp_pub);
  //nh.advertise(status_pub);
}

void loop() {
  temp_pub.publish(&temp);
  //status_pub.publish(&stats);
  
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

