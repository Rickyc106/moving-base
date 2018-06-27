/**
This node runs on the MCU. To edit it you need to follow the tutorials at
http://wiki.ros.org/rosserial_tivac/Tutorials/Energia%20Setup.
*/

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <Servo.h>

int lpwm_pin = PD_2,
    rpwm_pin = PD_3,
    lenca_pin= PE_2,
    lencb_pin= PE_3,
    renca_pin= PA_2,
    rencb_pin= PA_3;

Servo left_motor;
Servo right_motor;

ros::NodeHandle nh;

std_msgs::Int32 left_ticks;
std_msgs::Int32 right_ticks;
std_msgs::Bool zero;

//TODO: add watchdog
void left_cb(const std_msgs::Float32& msg){
    left_motor.write(90 + msg.data * 90);
}

void right_cb(const std_msgs::Float32& msg){
    right_motor.write(90 + msg.data * 90);
}

void zero_cb(const std_msgs::Bool& msg){
    if(msg.data){
      left_ticks.data = 0;
      right_ticks.data = 0;
    }
}

ros::Subscriber < std_msgs::Bool > zero_sub("zero", &zero_cb);
ros::Subscriber < std_msgs::Float32 > left_sub("left", &left_cb);
ros::Subscriber < std_msgs::Float32 > right_sub("right", &right_cb);

void left_encoder_int(){
    if(digitalRead(lencb_pin)){
      --left_ticks.data;
    }
    else{
      ++left_ticks.data;
    }
}

void right_encoder_int(){
    if(digitalRead(rencb_pin)){
      --right_ticks.data;
    }
    else{
      ++right_ticks.data;
    }
}

ros::Publisher left_pub("lticks", &left_ticks);
ros::Publisher right_pub("rticks", &right_ticks);

void setup(){
    pinMode(lenca_pin, INPUT);
    pinMode(lencb_pin, INPUT);
    pinMode(renca_pin, INPUT);
    pinMode(rencb_pin, INPUT);

    left_ticks.data = 0;
    right_ticks.data = 0;
    attachInterrupt(lenca_pin, &left_encoder_int, RISING);
    attachInterrupt(renca_pin, &right_encoder_int, RISING);

    left_motor.attach(PD_2);
    right_motor.attach(PD_3);

    nh.initNode();
    nh.subscribe(left_sub);
    nh.subscribe(right_sub);

    nh.advertise(left_pub);
    nh.advertise(right_pub);
}

void loop(){
    left_pub.publish(&left_ticks);
    right_pub.publish(&right_ticks);
    nh.spinOnce();
    delay(10);
}