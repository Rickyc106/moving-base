#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/tf.h>

ros::NodeHandle nh;
std_msgs::Float32MultiArray array_out;

void command_cb(const std_msgs::Float32MultiArray& msg) {
  for (int i = 0; i < 8; i++) {
    array_out.data[i] = msg.data[i] * 2.0;
  }
}

ros::Subscriber < std_msgs::Float32MultiArray > command_sub("commands", &command_cb);
ros::Publisher encoder_pub("encoder_ticks", &array_out);

void setup() {
  nh.initNode();
  nh.subscribe(command_sub);
  nh.advertise(encoder_pub);

  nh.getHardware()->setBaud(57600);

  array_out.data_length = 8;
}

void loop() {
  encoder_pub.publish(&array_out);
  nh.spinOnce();
}
