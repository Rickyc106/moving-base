#include <ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <moving_base/arduino_status.h>
#include <tf/tf.h>

#include <Servo.h>
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

float error[8];
float sum[8];
float previous[8];

float output;
float p_gain, i_gain, d_gain;

bool CCW, no_rotate;
bool forwards[4];

//std_msgs::Float32 temp;
//std_msgs::Float32 testing;

std_msgs::Bool boolean_data;

std_msgs::Float32 DATA_0;
std_msgs::Float32 DATA_1;
std_msgs::Float32 DATA_2;
std_msgs::Float32 DATA_3;

std_msgs::Float32MultiArray temp_array;

//std_msgs::Float32MultiArray mag_array;
//moving_base::arduino_status stats;

// ------------------------------------------------------------------------------------

bool enc_CW = false;
bool enc_CCW = false;
int CW_pulse, CCW_pulse;
bool stop_signal = false;
bool run_all_motors = false;
bool startUp = true;

Servo motors[8];
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

int var[4];
float drive_speed[4];
float steer_speed[4];

float comp_steer[4];
float comp_drive[4];

float alpha = 0.8;
float beta = 0.95;

int counter = 0;
float width = 0.2;
float speed_cap = 0.5; // 0.1 Good cap for testing drive encoders with finger
                        // 0.15 Good cap for testing steer encoders with finger

unsigned short comp_period[8];

unsigned short motor_period[8];
unsigned short min_period[2];
float rotation[8];
float max_PWM_output[2];
float min_PWM_output[2];
float range[2];
bool motor_dir[8];

const byte numBytes = 10;

boolean newData = false;

float PWM_output[8];
float pot_input, steer_input, drive_input;
float steer_rotation, drive_rotation;
boolean motor_stopped[8];

int motor_counter[8];
unsigned short queue[8][10];

float random_float_value;

EasyTransfer ETin_steer, ETout_steer, ETin_drive, ETout_drive;

struct STEER_DATA_STRUCTURE {
  unsigned short filter_period[4];
  //boolean wheel_stopped[4];
  int wheel_counter[4];
};

struct DRIVE_DATA_STRUCTURE {
  unsigned short filter_period[4];
  //boolean wheel_stopped[4];
  int wheel_counter[4];
};

STEER_DATA_STRUCTURE rx_steer;
DRIVE_DATA_STRUCTURE rx_drive;

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

  for (int i =  0; i < 4; i++) {
    if (msg.buttons[i]) var[i] = i;  
  }

  if (msg.buttons[0] && msg.buttons[1] && msg.buttons[2] && msg.buttons[3]) run_all_motors = true;

  drive_input = msg.axes[1];

  if (!msg.buttons[0] && !msg.buttons[1] && !msg.buttons[2] && !msg.buttons[3]) {
    motor_stop();
    run_all_motors = false;
  }

  // FINISH TRIG MATH -----------------------------------------------------------------

  steer_angle *= (180 / 3.141592653);
  
  temp_array.data = resultant;  // WARNING: DO NOT CHANGE RIGHT NOW
  //testing.data = 90 + msg.axes[1] * 90;
}

ros::Subscriber < sensor_msgs::Joy > joy_sub("joy_queued", &xbox_cb);

//ros::Publisher temp_pub("temp_data", &temp);
//ros::Publisher testing_pub("testing", &testing);

ros::Publisher boolean_data_pub("boolean_data", &boolean_data);

ros::Publisher DATA_0_pub("DATA_0", &DATA_0);
ros::Publisher DATA_1_pub("DATA_1", &DATA_1);
ros::Publisher DATA_2_pub("DATA_2", &DATA_2);
ros::Publisher DATA_3_pub("DATA_3", &DATA_3);

ros::Publisher temp_array_pub("temp_array_data", &temp_array);

void setup() {
  initialize();   // Attach motors, establish serial comms, etc.
  ros_init();   // **Comment out when using Serial monitor for debugging**
}

void loop() {

  //steer_rotation = steer_input * 3.856;   // Arbitrary setpoint
  //drive_rotation = drive_input * 3.856;   // Arbitrary setpoint

  steer_rotation = convert_controller_to_speed(steer_input, 1.0);
  drive_rotation = convert_controller_to_speed(drive_input, 1.0);
  
  for (int i = 0; i < 4; i++) {
    ETin_steer.receiveData();
    motor_period[i] = rx_steer.filter_period[i];
    comp_period[i] = beta * comp_period[i] + (1 - beta) * motor_period[i];

    ETin_drive.receiveData();
    motor_period[i+4] = rx_drive.filter_period[i];
    comp_period[i+4] = beta * comp_period[i+4] + (1 - beta) * motor_period[i+4];

    queue[i][0] = motor_period[i];
    queue[i+4][0] = motor_period[i+4];

    for (int j = 9; j > 0; j--) {
      queue[i][j] = queue[i][j-1];
      queue[i+4][j] = queue[i+4][j-1];
    }

    if (queue[i][9] == queue[i][0]) motor_stopped[i] = true;
    if (queue[i+4][9] == queue[i][0]) motor_stopped[i+4] = true;

    //motor_stopped[i] = rx_steer.wheel_stopped[i];
    motor_counter[i] = rx_steer.wheel_counter[i];

    if (steer_rotation != 0) rotation[i] = 0.0022 * (2 * 3.141592653 * 1000000) / (comp_period[i] * 2) + 0.2452;
    //if (steer_rotation != 0 && motor_period[i] > 250 && motor_period[i] < 14000) rotation[i] = 0.0022 * (2 * 3.141592653 * 1000000) / (motor_period[i] * 2) + 0.2452;
    else if (steer_rotation == 0) rotation[i] = 0;

    if (motor_counter[i] < 0 || motor_counter[i] > 3) rotation[i] = 0;

    //motor_stopped[i+4] = rx_drive.wheel_stopped[i];
    motor_counter[i+4] = rx_drive.wheel_counter[i];

    if (drive_rotation != 0 && motor_counter[i+4] >= 0 && motor_counter[i+4] < 3) rotation[i+4] = 0.0022 * (2 * 3.141592653 * 1000000) / (comp_period[i+4] * 2) + 0.2452;  // may have to recalibrate for drive motors
    //if (drive_rotation != 0 && motor_period[i+4] > 250 && motor_period[i+4] < 14000) rotation[i+4] = 0.0022 * (2 * 3.141592653 * 1000000) / (motor_period[i+4] * 2) + 0.2452;  // may have to recalibrate for drive motors
    else if (drive_rotation == 0) rotation[i+4] = 0;

    if (motor_counter[i+4] < 0 || motor_counter[i+4] > 3) rotation[i+4] = 0;

    if (CCW == false) rotation[i] = rotation[i] * -1;
    if (drive_rotation < 0) rotation[i+4] = rotation[i+4] * -1;
  }

  min_period[0] = motor_period[0];
  min_period[1] = motor_period[4];

  for (int i = 1; i < 4; i++) {
    if (motor_period[i] > min_period[0]) min_period[0] = motor_period[i];
    if (motor_period[i+4] > min_period[4]) min_period[1] = motor_period[i+4];
  }

  //steer_rotation = steer_input * speed_cap * 9.052;   // TEST BEFORE RUNNING
  //drive_rotation = drive_input * speed_cap * 9.052;   // TEST BEFORE RUNNING

  for (int i = 0; i < 4; i++) {
    //PWM_output[i] = PID(min_period[0], motor_period[i]);
    //PWM_output[i+4] = PID(min_period[1], motor_period[i+4]);
/*
    if (var[i] == i && steer_input == 0) {
      PWM_output[i] = 0;
    }
    else if (var[i] == i && steer_input != 0) {
      //PWM_output[i] = PID(i, motor_period[0], motor_period[i]);
      PWM_output[i] = PID(i, steer_rotation, rotation[i]);    // Uncomment once previous lines tested
    }
    
    if (var[i] == i && drive_input == 0) {
      PWM_output[i+4] = 0;  
    }
    else if (var[i] == i && drive_input != 0) {
      //PWM_output[i+4] = PID(i+4, motor_period[4], motor_period[i+4]);
      PWM_output[i+4] = PID(i, drive_rotation, rotation[i+4]);    // Uncomment once previous lines tested
    }
*/
    PWM_output[i] = PID(i, steer_rotation, rotation[i]);    // Uncomment once previous lines tested
    PWM_output[i+4] = PID(i+4, drive_rotation, rotation[i+4]);    // Uncomment once previous lines tested
  }

  max_PWM_output[0] = PWM_output[0];
  max_PWM_output[1] = PWM_output[4];

  min_PWM_output[0] = PWM_output[0];
  min_PWM_output[1] = PWM_output[4];

  for (int i = 1; i < 4; i++) {
    if (PWM_output[i] > max_PWM_output[0]) max_PWM_output[0] = PWM_output[i];
    if (PWM_output[i+4] > max_PWM_output[1]) max_PWM_output[1] = PWM_output[i+4];

    if (PWM_output[i] < min_PWM_output[0]) min_PWM_output[0] = PWM_output[i];
    if (PWM_output[i+4] < min_PWM_output[1]) min_PWM_output[1] = PWM_output[i+4];
  }
  
  //pot_input = mapf(analogRead(A8), 0, 1023, -1, 0) * 0.5;

  range[0] = max_PWM_output[0] - min_PWM_output[0];
  range[1] = max_PWM_output[1] - min_PWM_output[1];

  for (int i = 0; i < 4; i++) {
    //steer_speed[i] = constrain(pot_input * PWM_output[i], -1, 0);
    //drive_speed[i] = constrain(pot_input * PWM_output[i+4], -1, 0);
    /*
    if (min_PWM_output[0] >= 0) {
      PWM_output[i] = mapf(PWM_output[i], min_PWM_output[0], max_PWM_output[0], 0, 0.2);  
    }
    else if (min_PWM_output[0] < 0) {
      PWM_output[i] = mapf(PWM_output[i], min_PWM_output[0], max_PWM_output[0], -0.2, 0);  
    }

    if (min_PWM_output[1] >= 0) {
      PWM_output[i+4] = mapf(PWM_output[i+4], min_PWM_output[1], max_PWM_output[1], 0, 0.2);  
    }
    else if (min_PWM_output[1] < 0) {
      PWM_output[i+4] = mapf(PWM_output[i+4], min_PWM_output[1], max_PWM_output[1], -0.2, 0);  
    }
    */
    //steer_speed[i] = steer_input * PWM_output[i];
    //drive_speed[i] = drive_input * PWM_output[i+4];

    //PWM_output[i] = (((PWM_output[i] - min_PWM_output[0]) / abs(range[0])) * (2 * width)) - width;       // Width of 0.2 right now
    //PWM_output[i+4] = (((PWM_output[i+4] - min_PWM_output[1]) / abs(range[1])) * (2 * width)) - width;
    
    //steer_input = constrain(steer_input, -0.8, 0.8);
    //drive_input = constrain(drive_input, -0.8, 0.8);

    steer_speed[i] = steer_speed[i] + PWM_output[i];        // Old integrator here
    drive_speed[i] = drive_speed[i] + PWM_output[i+4];      // Old integrator here

    //steer_speed[i] = (steer_input * throttle_cap) + PWM_output[i];    // Testing new controller here
    //drive_speed[i] = (drive_input * throttle_cap) + PWM_output[i+4];  // Testing new controllre here
    
    steer_speed[i] = constrain(steer_speed[i], -1, 1);
    drive_speed[i] = constrain(drive_speed[i], -1, 1);

    if (steer_rotation == 0) steer_speed[i] = 0;
    if (drive_rotation == 0) drive_speed[i] = 0; 

    //comp_steer[i] = (1 - alpha) * steer_speed[i];
    //random_float_value = comp_steer[i] + alpha * comp_steer[i];

    comp_steer[i] = alpha * (comp_steer[i]) + ((1 - alpha) * steer_speed[i]);
    //comp_drive[i] = alpha * (comp_drive[i]) + ((1 - alpha) * drive_speed[i]);

    //steer_speed[i] = constrain(steer_speed[i], -1, 1);
    //drive_speed[i] = constrain(drive_speed[i], -1, 1);
  }
/*
  DATA_0.data = steer_speed[0];
  DATA_1.data = steer_speed[1];
  DATA_2.data = steer_speed[2];
  DATA_3.data = steer_speed[3];
*/
/*
  DATA_0.data = drive_speed[0];
  DATA_1.data = drive_speed[1];
  DATA_2.data = drive_speed[2];
  DATA_3.data = drive_speed[3];
*/
/*
  DATA_0.data = PWM_output[0];
  DATA_1.data = PWM_output[1];
  DATA_2.data = PWM_output[2];
  DATA_3.data = PWM_output[3];
*/
  
  boolean_data.data = motor_stopped[0];
  
  //DATA_0.data = min_PWM_output[0];
  //DATA_1.data = min_PWM_output[1];
  //DATA_2.data = max_PWM_output[0];
  //DATA_3.data = max_PWM_output[1];

/*
  DATA_0.data = motor_period[0];
  DATA_1.data = motor_period[1];
  DATA_2.data = motor_period[2];
  DATA_3.data = motor_period[3];
*/
/*
  DATA_0.data = rotation[0];
  DATA_1.data = rotation[1];
  DATA_2.data = rotation[2];
  DATA_3.data = rotation[3];
*/
  //DATA_0.data = steer_rotation;
  //DATA_1.data = drive_rotation;
/*
  DATA_0.data = comp_steer[0];
  DATA_1.data = comp_steer[1];
  DATA_2.data = comp_steer[2];
  DATA_3.data = comp_steer[3];
*/
  //DATA_0.data = motor_counter[0];
  //DATA_0.data = steer_rotation;
  //DATA_0.data = motor_period[0];
  //DATA_0.data = rotation[0];
  //DATA_0.data = error[0];
  //DATA_0.data = PWM_output[0];
  //DATA_0.data = steer_speed[0];
  //DATA_0.data = comp_steer[0];

  DATA_0.data = steer_rotation;
  DATA_1.data = rotation[0];
  //DATA_1.data = motor_period[0];
  //DATA_1.data = comp_period[0];
  //DATA_2.data = steer_speed[0];
  DATA_2.data = comp_steer[0];
  //DATA_2.data = PWM_output[0];

  //DATA_0.data = queue[0][0];
  //if (motor_stopped[0]) DATA_0.data = 1;
  //else DATA_0.data = 0;

  if (run_all_motors) {
    for (int i = 0; i < 4; i++) {
      motor_debugging(var[i], comp_steer[i], 0); 
      //motor_debugging(var[i], steer_input, 0); 
    }
  }
  else {
    for (int i = 0; i < 4; i++) {
      //motor_debugging(var[i], steer_input * throttle_cap, 0);
      motor_debugging(var[i], steer_input, 0);
      //motor_debugging(var[i], comp_steer[i], 0);
    }
  }
  
  for (int i = 0; i < 4; i++) {
    //motor_debugging(i, steer_speed[i], drive_speed[i]); // Drive Speed is PID output speed, drive input is input from controller
    //motor_debugging(i,steer_input, drive_input);
    //motor_debugging(var[i],steer_speed[i], drive_speed[i]);
    //motor_debugging(var[i], steer_input, 0);
    //motor_debugging(var[i], steer_speed[i], 0);
  }

  //temp_pub.publish(&temp);
  //testing_pub.publish(&testing);

  //boolean_data_pub.publish(&boolean_data);

  DATA_0_pub.publish(&DATA_0);
  DATA_1_pub.publish(&DATA_1);
  DATA_2_pub.publish(&DATA_2);
  DATA_3_pub.publish(&DATA_3);
  
  //temp_array_pub.publish(&temp_array);

  nh.spinOnce();
}

float convert_controller_to_speed(float input, float throttle_cap) {
  float speed_output; 
  
  if (input == 0) speed_output = 0;
  else if (input != 0) speed_output = throttle_cap * ((input * speed_cap * 11.2) - 0.624);
  
  return speed_output; 
}

////////////////////////////////////////////////////////////////////
//  Print Motor Speed to Serial Monitor -- DO NOT use Rosserial   //
////////////////////////////////////////////////////////////////////

void print_speed() {
  Serial.print("Steer 0: ");
  Serial.print(motor_period[0]);
  Serial.print("\t");
  Serial.print("Steer 1: ");
  Serial.print(motor_period[1]);
  Serial.print("\t");
  Serial.print("Steer 2: ");
  Serial.print(motor_period[2]);
  Serial.print("\t");
  Serial.print("Steer 3: ");
  Serial.print(motor_period[3]);
  Serial.println();

  Serial.print("Drive 0: ");
  Serial.print(motor_period[4]);
  Serial.print("\t");
  Serial.print("Drive 1: ");
  Serial.print(motor_period[5]);
  Serial.print("\t");
  Serial.print("Drive 2: ");
  Serial.print(motor_period[6]);
  Serial.print("\t");
  Serial.print("Drive 3: ");
  Serial.print(motor_period[7]);
  Serial.println();
}

////////////////////////////////////////////////////////////////////
//        Currently tuning PID gains -- Works but iffy            //
////////////////////////////////////////////////////////////////////

float PID(int motor, float setpoint, float current_value) {
  for (int i = 0; i < 8; i++) {
    if (motor == i) {
      error[i] = setpoint - current_value;
      sum[i] += error[i];

      if (sum[i] > 0.5) sum[i] = 0.5;
      else if (sum[i] < -0.5) sum[i] = -0.5;

      output = p_gain * error[i] + i_gain * sum[i] + d_gain * (previous[i] - error[i]);
      previous[i] = error[i];
    }
  }

  return output;
}

////////////////////////////////////////////////////////////////////
//          Attach motors, establish serial comms, etc.           //
////////////////////////////////////////////////////////////////////

void initialize() {
  Serial.begin(115200);   // **Comment Out when using Rosserial -- Only used for easier debugging**
  Serial1.begin(115200);
  Serial2.begin(115200);
  
  ETin_steer.begin(details(rx_steer), &Serial1);
  //ETout_steer.begin(details(tx_steer), &Serial1);

  ETin_drive.begin(details(rx_drive), &Serial2);
  //ETout_drive.begin(details(tx_drive), &Serial2);

  rotate_angle[0] -= M_PI / 4;
  rotate_angle[1] += M_PI / 4;
  rotate_angle[2] -= M_PI / 4;
  rotate_angle[3] += M_PI / 4;

  p_gain = 0.0025; // Previous 2.1
  i_gain = 0.0;// Previous 0.25
  d_gain = 0.0; // Previous 0.1

  for (int i = 0; i < 8; i++) {
    motors[i].attach(i + 32);   // Attach motors -- Pins 32 -> 39
    //pinMode(i + 32, OUTPUT); // 22
  }
}

////////////////////////////////////////////////////////////////////
//             Init ROS Node, advertise & subscribe.              //
////////////////////////////////////////////////////////////////////

void ros_init() {
  nh.initNode();  // Create ROS nodehandle

  nh.subscribe(joy_sub);    // Subscribe to joy controller message
  
  //nh.advertise(temp_pub);       // Debugging purposes -- Float32
  //nh.advertise(testing_pub);    // Debugging purposes -- Float32

  nh.advertise(boolean_data_pub);

  nh.advertise(DATA_0_pub);
  nh.advertise(DATA_1_pub);
  nh.advertise(DATA_2_pub);
  nh.advertise(DATA_3_pub);
  
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

void motor_debugging(int var, float steer, float drive) {
  switch (var) {
    case 0:
      motors[0].writeMicroseconds(1500 + (steer * 500 * speed_cap));    // STEER MOTOR -- 1.45 max
      motors[1].writeMicroseconds(1500 + (drive * 500 * speed_cap));   // DRIVE MOTOR -- 32 max right now
      break;
    case 1:
      motors[2].writeMicroseconds(1500 + (steer * 500 * speed_cap));    // STEER MOTOR -- 1.45 max
      motors[3].writeMicroseconds(1500 + (drive * 500 * speed_cap));   // DRIVE MOTOR -- 32 max right now
      break;
    case 2:
      motors[4].writeMicroseconds(1500 + (steer * 500 * speed_cap));    // STEER MOTOR -- 1.45 max
      motors[5].writeMicroseconds(1500 + (drive * 500 * speed_cap));   // DRIVE MOTOR -- 32 max right now
      break;
    case 3:
      motors[6].writeMicroseconds(1500 + (steer * 500 * speed_cap));    // STEER MOTOR -- 1.45 max
      motors[7].writeMicroseconds(1500 + (drive * 500 * speed_cap));   // DRIVE MOTOR -- 32 max right now
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

