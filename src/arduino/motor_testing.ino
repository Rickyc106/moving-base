#include <ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/tf.h>
#include <Servo.h>
#include <avr/interrupt.h>
#include <EasyTransfer.h>

ros::NodeHandle nh;

float x, y;
float steer_angle;
float steer_mag;

float rotate_angle[4];
float rotate_mag[4];

float res_x, res_y;
float resultant_angle[4];
float resultant[4];

float output;
float p_gain, i_gain, d_gain;
float p_drive, i_drive, d_drive;

bool CCW, no_rotate;
bool forwards[4];

//std_msgs::Float32 temp;
//std_msgs::Float32 testing;

std_msgs::Float32 DATA_0;
std_msgs::Float32 DATA_1;
std_msgs::Float32 DATA_2;
std_msgs::Float32 DATA_3;

//std_msgs::Float32MultiArray steer_pos;
std_msgs::Float32MultiArray desired_motor_act;

std_msgs::Float32MultiArray temp_array;

//std_msgs::Float32MultiArray mag_array;
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
float drive_speed[4], drive_position[4];
float steer_speed[4], steer_position[4];

long previous_micros[4];

int counter = 0;
float speed_cap = 0.45;

uint8_t comp_period[8];

unsigned short min_speed[2];
float max_PWM_output[2];
bool motor_dir[8];

const byte numBytes = 10;

const float pi = 3.141592653589793;

boolean newData = false;

float PWM_output[8];
float steer_input, drive_input;

float steer_desired_speed[4], drive_desired_speed[4];

double steer_time_step, drive_time_step;
double comp_steer_time_step, comp_drive_time_step;

double steer_desired_position[4], drive_desired_position[4], steer_desired_fpos[4];
double prev_steer_desired_position[4], prev_drive_desired_position[4];

float steer_pos_offset[4], drive_pos_offset[4];

float error[8], sum[8], previous[8];
float steer_output[4], drive_output[4];

float alpha = 0.8;
float comp_steer[4], comp_drive[4];

EasyTransfer ETin_steer, ETout_steer, ETin_drive, ETout_drive;

struct STEER_DATA_STRUCTURE {
  unsigned short filter_period[4];
  float wheel_position[4];
};

struct DRIVE_DATA_STRUCTURE {
  unsigned short filter_period[4];
  float wheel_position[4];
};

STEER_DATA_STRUCTURE rx_steer;
DRIVE_DATA_STRUCTURE rx_drive;

void desiredMotorActCb(const std_msgs::Float32MultiArray& msg) {
  //Set desired steer final angle
  for (int i = 0; i < 4; i++) {
    steer_desired_fpos[i] = msg.data[i];
    steer_desired_speed[i] = msg.data[i+4];
    drive_desired_speed[i] = msg.data[i+8];
  }
}

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

      steer_input = mapf(msg.axes[2], -1.0, 1.0, 1.0, 0);
    }
    else if (!CCW && !no_rotate) {
      rotate_angle[i] -= M_PI / 22.5;
      rotate_mag[i] = 1.0;

      steer_input = mapf(msg.axes[5], -1.0, 1.0, -1.0, 0);
    }

    if (i == 0 || i == 1) rotate_mag[i] = -1.0;

    if (no_rotate) {
      rotate_mag[i] = 0.0;

      steer_input = 0.0;
      //val = 0.0;
    }

    if (steer_mag != 0.0 || rotate_mag[i] != 0.0) {
      resultant[i] = vector_sum(steer_angle, steer_mag, rotate_angle[i], rotate_mag[i], i);
    }
  }

  drive_input = msg.axes[1] * 0.4;

  if (!msg.buttons[0] && !msg.buttons[1] && !msg.buttons[2] && !msg.buttons[3]) motor_stop();

  for (int i = 0; i < 4; i++) {
    if (msg.buttons[i]) var[i] = i;
  }

  // FINISH TRIG MATH -----------------------------------------------------------------

  steer_angle *= (180 / 3.141592653);
  
  temp_array.data = resultant;  // WARNING: DO NOT CHANGE RIGHT NOW
  //testing.data = 90 + msg.axes[1] * 90;
}

//PLAYSTATION CONTROLLER
//ros::Subscriber < sensor_msgs::Joy > joy_sub("joy_queued", &xbox_cb);

//ros::Publisher temp_pub("temp_data", &temp);
//ros::Publisher testing_pub("testing", &testing);

ros::Subscriber < std_msgs::Float32MultiArray > desired_motor_act_sub("MOTOR_ACT", &desiredMotorActCb);
//ros::Publisher steer_pos_pub("STEER_POS", &steer_pos); 

ros::Publisher DATA_0_pub("DATA_0", &DATA_0);
ros::Publisher DATA_1_pub("DATA_1", &DATA_1);
ros::Publisher DATA_2_pub("DATA_2", &DATA_2);
ros::Publisher DATA_3_pub("DATA_3", &DATA_3);

ros::Publisher temp_array_pub("temp_array_data", &temp_array);

void setup() {
  ros_init();   // **Comment out when using Serial monitor for debugging**
  initialize();   // Attach motors, establish serial comms, etc.
}

void loop() {
  //Playstation Controller Input
  //steer_desired_speed = convert_controller_to_speed(steer_input, 1.0);
  //drive_desired_speed = convert_controller_to_speed(drive_input, 1.0);

  for (int i = 0; i < 4; i++) {
    ETin_steer.receiveData();
    steer_speed[i] = rx_steer.filter_period[i];
    steer_position[i] = rx_steer.wheel_position[i] - steer_pos_offset[i];

    ETin_drive.receiveData();
    drive_speed[i] = rx_drive.filter_period[i];
    drive_position[i] = rx_drive.wheel_position[i] - drive_pos_offset[i];
  }

  for (int i = 0; i < 4; i++) {

      //Always drive
      drive_time_step =  (2* pi) * (micros() - previous_micros[i]) / (1000000);
      drive_desired_position[i] = drive_position[i] + drive_desired_speed[i] * drive_time_step;

      comp_drive_time_step = (alpha) * comp_drive_time_step + (1 - alpha) * drive_time_step;
      
      if (abs(drive_desired_position[i] - prev_drive_desired_position[i]) > abs(drive_desired_speed[i] * comp_drive_time_step) * 2) {
        drive_desired_position[i] = prev_drive_desired_position[i];  
      }

      prev_drive_desired_position[i] = drive_desired_position[i];

      // Only increment if steer_desired_fpos is not reached **NOTE: only positive speeds (CCW) when absolute encoders are installed, this needs to be UPDATED
      if(steer_position[i] < steer_desired_fpos[i]) {
        steer_time_step =  (2* pi) * (micros() - previous_micros[i]) / (1000000);
        steer_desired_position[i] = steer_position[i] + steer_desired_speed[i] * steer_time_step;
  
        comp_steer_time_step = (alpha) * comp_steer_time_step + (1 - alpha) * steer_time_step;
        
        if (abs(steer_desired_position[i] - prev_steer_desired_position[i]) > abs(steer_desired_speed[i] * comp_steer_time_step) * 2) {
          steer_desired_position[i] = prev_steer_desired_position[i];  
        }
  
        prev_steer_desired_position[i] = steer_desired_position[i];
      }

      previous_micros[i] = micros();
  }

  steer_input = constrain(steer_input, -0.8, 0.8);
  drive_input = constrain(drive_input, -0.8, 0.8);
  
  for (int i = 0; i < 4; i++) {
    steer_output[i] = PID(i, steer_desired_position[i], steer_position[i], 0.02);     // PID From Setpoint and Raw rotation data
    drive_output[i] = PID(i+4, drive_desired_position[i], drive_position[i], 0.02);

    steer_output[i] = constrain(steer_output[i], -1.0, 1.0);
    drive_output[i] = constrain(drive_output[i], -1.0, 1.0);
    
    comp_steer[i] = alpha * comp_steer[i] + (1 - alpha) * steer_output[i];      // Filter PID output
    comp_drive[i] = alpha * comp_drive[i] + (1 - alpha) * drive_output[i];

    comp_steer[i] = constrain(comp_steer[i], -1.0, 1.0);                        // Constrain to [-1.0, 1.0] bounds
    comp_drive[i] = constrain(comp_drive[i], -1.0, 1.0);
  }
  
  for (int i = 0; i < 4; i++) {
    //motor_debugging(var[i], comp_steer[i], comp_drive[i]); // Drive Speed is PID output speed, drive input is input from controller
    set_motor_speed(i, comp_steer[i], comp_drive[i]);
  }

  //DATA_0.data = steer_desired_position[0];
  //DATA_1.data = steer_position[0];
  //DATA_2.data = steer_output[0];
  //DATA_3.data = comp_steer[0];

  //DATA_0.data = drive_desired_position[0];
  //DATA_1.data = drive_position[0];
  //DATA_2.data = drive_output[0];
  //DATA_3.data = comp_drive[0];

  //DATA_0.data = steer_desired_position[0];
  //DATA_1.data = steer_desired_position[1];
  //DATA_2.data = steer_desired_position[2];
  //DATA_3.data = steer_desired_position[3];
  
  DATA_0.data = drive_position[0];
  DATA_1.data = drive_output[1];
  DATA_2.data = drive_desired_position[1];
  DATA_3.data = drive_desired_speed[1];
  
  DATA_0_pub.publish(&DATA_0);
  DATA_1_pub.publish(&DATA_1);
  DATA_2_pub.publish(&DATA_2);
  DATA_3_pub.publish(&DATA_3);
  //steer_pos_pub.publish(&steer_pos);

  nh.spinOnce();
}

float convert_controller_to_speed(float input, float throttle_cap) {
  float speed_output; 
  
  speed_output = throttle_cap * ((input * speed_cap * 11.2));    // Force through zero
  
  return speed_output; 
}

////////////////////////////////////////////////////////////////////
//  Print Motor Speed to Serial Monitor -- DO NOT use Rosserial   //
////////////////////////////////////////////////////////////////////

void print_speed() {
  Serial.print("Steer 0: ");
  Serial.print(steer_speed[0]);
  Serial.print("\t");
  Serial.print("Steer 1: ");
  Serial.print(steer_speed[1]);
  Serial.print("\t");
  Serial.print("Steer 2: ");
  Serial.print(steer_speed[2]);
  Serial.print("\t");
  Serial.print("Steer 3: ");
  Serial.print(steer_speed[3]);
  Serial.println();

  Serial.print("Drive 0: ");
  Serial.print(drive_speed[4]);
  Serial.print("\t");
  Serial.print("Drive 1: ");
  Serial.print(drive_speed[5]);
  Serial.print("\t");
  Serial.print("Drive 2: ");
  Serial.print(drive_speed[6]);
  Serial.print("\t");
  Serial.print("Drive 3: ");
  Serial.print(drive_speed[7]);
  Serial.println();
}

////////////////////////////////////////////////////////////////////
//        Currently tuning PID gains -- Works but iffy            //
////////////////////////////////////////////////////////////////////

float PID(int motor, float setpoint, float current_value, float deadzone) {
  for (int i = 0; i < 8; i++) {
    if (motor == i) {
      if (motor >= 0 && motor < 4) {
        error[i] = setpoint - current_value;
        //if (abs(error[i]) < deadzone) error[i] = 0;
        
        sum[i] += error[i];
  
        if (sum[i] > 0.5) sum[i] = 0.5;
        else if (sum[i] < -0.5) sum[i] = -0.5;
  
        output = p_gain * error[i] + i_gain * sum[i] + d_gain * (error[i] - previous[i]);
        previous[i] = error[i];  
      }

      if (motor >= 4 && motor < 8) {
        error[i] = setpoint - current_value;
        //if (abs(error[i]) < deadzone) error[i] = 0;
        
        sum[i] += error[i];
  
        if (sum[i] > 0.5) sum[i] = 0.5;
        else if (sum[i] < -0.5) sum[i] = -0.5;
  
        output = p_drive * error[i] + i_drive * sum[i] + d_drive * (error[i] - previous[i]);
        previous[i] = error[i];  
      }
    }
  }

  return output;
}

////////////////////////////////////////////////////////////////////
//          Attach motors, establish serial comms, etc.           //
////////////////////////////////////////////////////////////////////

void initialize() {
  //Serial.begin(115200);   // **Comment Out when using Rosserial -- Only used for easier debugging**
  Serial1.begin(115200);
  Serial2.begin(115200);
  
  ETin_steer.begin(details(rx_steer), &Serial1);
  //ETout_steer.begin(details(tx_steer), &Serial1);

  ETin_drive.begin(details(rx_drive), &Serial2);
  //ETout_drive.begin(details(tx_drive), &Serial2);

  delay(10);

  for (int i = 0; i < 4; i++) {
    ETin_steer.receiveData();
    steer_pos_offset[i] = rx_steer.wheel_position[i];
  
    ETin_drive.receiveData();
    drive_pos_offset[i] = rx_drive.wheel_position[i];
  }  

  rotate_angle[0] -= M_PI / 4;
  rotate_angle[1] += M_PI / 4;
  rotate_angle[2] -= M_PI / 4;
  rotate_angle[3] += M_PI / 4;

  p_gain = 6.5;
  i_gain = 0.35;
  d_gain = 1.0;

  p_drive = 2.5;
  i_drive = 0.2;
  d_drive = 0;

  for (int i = 0; i < 2; i++) {
    motors[i].attach(i + 22);
    motors[i+2].attach(i+48);
    motors[i+4].attach(i+50);
    motors[i+6].attach(i+24);
  }
}

////////////////////////////////////////////////////////////////////
//             Init ROS Node, advertise & subscribe.              //
////////////////////////////////////////////////////////////////////

void ros_init() {
  nh.initNode();  // Create ROS nodehandle

  //nh.subscribe(joy_sub);    // Subscribe to joy controller message

  nh.subscribe(desired_motor_act_sub);
  
  //nh.advertise(temp_pub);       // Debugging purposes -- Float32
  //nh.advertise(testing_pub);    // Debugging purposes -- Float32

  nh.advertise(DATA_0_pub);
  nh.advertise(DATA_1_pub);
  nh.advertise(DATA_2_pub);
  nh.advertise(DATA_3_pub);
  //nh.advertise(steer_pos_pub);
  
  nh.advertise(temp_array_pub); // Debugging purposes -- Float32MultiArray
  
  temp_array.data_length = 4;   // Set length for array message
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

void set_motor_speed(int var, float drive, float steer) {
  switch (var) {
    case 0:
      motors[0].writeMicroseconds(1500 + (drive * 500 * speed_cap));    // DRIVE MOTOR -- 32 max right now
      motors[1].writeMicroseconds(1500 + (steer * 500 * speed_cap));   // STEER MOTOR -- 1.45 max
      break;
    case 1:
      motors[2].writeMicroseconds(1500 + (drive * 500 * speed_cap));    // DRIVE MOTOR -- 32 max right now
      motors[3].writeMicroseconds(1500 + (steer * 500 * speed_cap));   // STEER MOTOR -- 1.45 max
      break;
    case 2:
      motors[4].writeMicroseconds(1500 + (drive * 500 * speed_cap));    // DRIVE MOTOR -- 32 max right now
      motors[5].writeMicroseconds(1500 + (steer * 500 * speed_cap));   // STEER MOTOR -- 1.45 max
      break;
    case 3:
      motors[6].writeMicroseconds(1500 + (drive * 500 * speed_cap));    // DRIVE MOTOR -- 32 max right now
      motors[7].writeMicroseconds(1500 + (steer * 500 * speed_cap));   // STEER MOTOR -- 1.45 max
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
