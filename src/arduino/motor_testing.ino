#include <ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <moving_base/arduino_status.h>
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
float drive_speed[4];
float steer_speed[4];

volatile float pulse_count[16];   // Steer: 0-7, Drive 8-15
volatile int previous_pulse[16];   // Stores previous pulse -- One for each pair of quadrature signals

uint8_t data_in[5], data_in_drive[5];
byte data_in_prev[5], data_in_drive_prev[5];

byte data_in_1[2], data_in_2[2], data_in_3[2], data_in_0[2];

int counter = 0;

uint8_t comp_period[8];

unsigned short motor_speed[8];
unsigned short min_speed[2];
bool motor_dir[8];

const byte numBytes = 10;

boolean newData = false;

float PWM_output[8];
float pot_input, steer_input, drive_input;

EasyTransfer ETin_steer, ETout_steer, ETin_drive, ETout_drive;

struct STEER_DATA_STRUCTURE {
  unsigned short filter_period[4];
};

struct DRIVE_DATA_STRUCTURE {
  unsigned short filter_period[4];
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

  drive_input = msg.axes[1] * 0.40;

  for (int i = 0; i < 4; i++) {
    if (msg.buttons[i]) var[i] = i;
  }

  if (!msg.buttons[0] && !msg.buttons[1] && !msg.buttons[2] && !msg.buttons[3]) motor_stop();

  // FINISH TRIG MATH -----------------------------------------------------------------

  steer_angle *= (180 / 3.141592653);

  //temp.data = motor_speed[0];
  //temp.data = motor_speed[1];
  //temp.data = motor_speed[2];
  //temp.data = motor_speed[3];
  
  //testing.data = motor_speed[4];
  //testing.data = motor_speed[5];
  //testing.data = motor_speed[6];
  //testing.data = motor_speed[7];

  //temp.data = data_in[2];
  //testing.data = data_in_drive[2];
  
  temp_array.data = resultant;  // WARNING: DO NOT CHANGE RIGHT NOW
  //testing.data = 90 + msg.axes[1] * 90;
}

ros::Subscriber < sensor_msgs::Joy > joy_sub("joy_queued", &xbox_cb);

ros::Publisher temp_pub("temp_data", &temp);
ros::Publisher testing_pub("testing", &testing);
ros::Publisher temp_array_pub("temp_array_data", &temp_array);

void setup() {
  initialize();   // Attach motors, establish serial comms, etc.
  ros_init();   // **Comment out when using Serial monitor for debugging**
}

void loop() {
  
  
  //receive_data();
  //print_speed();
/*
  if (Serial2.available()) {
    while (Serial2.available() > 0) {
      byte incomingByte = Serial2.read();
      //Serial.println(incomingByte);

      if (incomingByte == '$') {
        for (int i = 0; i < 4; i++) {
          data_in[i] = Serial2.read();

          if (data_in[i] == 255) {
            while (data_in[i] == 255) data_in[i] = Serial2.read();  
          }
        }
        
        switch (data_in[0]) {
        // DRIVE MOTORS ----------------------------------------------
          case 0:
            motor_speed[4] = data_in[1] << 8 | data_in[2];
            motor_dir[4] = data_in[3];
            break;
          case 1:
            motor_speed[5] = data_in[1] << 8 | data_in[2];
            motor_dir[5] = data_in[3];
            break;
          case 2:
            motor_speed[6] = data_in[1] << 8 | data_in[2];
            motor_dir[6] = data_in[3];
            break;
          case 3:
            motor_speed[7] = data_in[1] << 8 | data_in[2];
            motor_dir[7] = data_in[3];
            break;
        }
      }
      
      if (incomingByte == '%') break;
    }
  }
*/

  for (int i = 0; i < 4; i++) {
    ETin_steer.receiveData();
    motor_speed[i] = rx_steer.filter_period[i];
    
    //Serial.print(rx_steer.filter_period[i]);
    //Serial.print(" ");
  }

  //Serial.print(" , ");
  
  for (int i = 0; i < 4; i++) {
    ETin_drive.receiveData();
    motor_speed[i+4] = rx_drive.filter_period[i];
    
    //Serial.print(rx_drive.filter_period[i]);
    //Serial.print(" ");
  }

  min_speed[0] = motor_speed[0];
  min_speed[1] = motor_speed[4];

  for (int i = 1; i < 4; i++) {
    if (motor_speed[i] > min_speed[0]) min_speed[0] = motor_speed[i];
    if (motor_speed[i+4] > min_speed[4]) min_speed[1] = motor_speed[i+4];
  }

  for (int i = 0; i < 4; i++) {
    PWM_output[i] = PID(min_speed[0], motor_speed[i]);
    PWM_output[i+4] = PID(min_speed[1], motor_speed[i+4]);

    //Serial.print(PWM_output[i]);
    //Serial.print(" ");
  }
  
  //Serial.println();

  pot_input = mapf(analogRead(A8), 0, 1023, -1, 0) * 0.5;
  
  for (int i = 0; i < 4; i++) {
    steer_speed[i] = constrain(pot_input * PWM_output[i], -1, 0);
    drive_speed[i] = constrain(pot_input * PWM_output[i+4], -1, 0);
  }

  for (int i = 0; i < 4; i++) {
    var[i] = i;
    motor_debugging(var[i], 0, drive_speed[i]);  
  }

  temp_pub.publish(&temp);
  testing_pub.publish(&testing);
  temp_array_pub.publish(&temp_array);

  nh.spinOnce();
}

////////////////////////////////////////////////////////////////////
//     Receive data when available -> Exits at stop flag          //
////////////////////////////////////////////////////////////////////

void receive_data() {
  static boolean receiveInProgress = false;
  static byte ndx = 0;

  byte startMarker = byte('$');
  byte endMarker = byte('%');
  
  while (Serial1.available() > 0 && newData == false) {
    byte incomingByte = Serial1.read();

    if (incomingByte == 255) {
      while (incomingByte == 255) incomingByte = Serial1.read();  
    }

    if (receiveInProgress == true) {
      if (incomingByte != endMarker) {
        data_in[ndx] = incomingByte;
        ndx++;
      }
      else {
        receiveInProgress = false;
        newData = true;
        ndx = 0;
      }
    }

    else if (incomingByte == startMarker) {
      receiveInProgress = true;  
    }
  }

  bit_convert();
}

////////////////////////////////////////////////////////////////////
//          Convert Two Bytes into Single 16-bit var              //
////////////////////////////////////////////////////////////////////

void bit_convert() {
  switch (data_in[0]) {
  // STEER MOTORS ----------------------------------------------
    case 0:
      motor_speed[0] = data_in[1] << 8 | data_in[2];
      motor_dir[0] = data_in[3];
      //temp.data = data_in[1];
      //testing.data = data_in[2];
      break;
    case 1:
      motor_speed[1] = data_in[1] << 8 | data_in[2];
      motor_dir[1] = data_in[3];
      //temp.data = data_in[1];
      //testing.data = data_in[2];
      break;
    case 2:
      motor_speed[2] = data_in[1] << 8 | data_in[2];
      motor_dir[2] = data_in[3];
      //temp.data = data_in[1];
      //testing.data = data_in[2];
      break;
    case 3:
      motor_speed[3] = data_in[1] << 8 | data_in[2];
      motor_dir[3] = data_in[3];
      //temp.data = data_in[1];
      //testing.data = data_in[2];
      break;
  } 
}

////////////////////////////////////////////////////////////////////
//        Print Motor Speed 0-3 and reset newData flag            //
////////////////////////////////////////////////////////////////////

void print_speed() {
  if (newData == true) {
    
      Serial.print("Steer 0: ");
      Serial.print(motor_speed[0]);
      Serial.print("\t");
      Serial.print("Steer 1: ");
      Serial.print(motor_speed[1]);
      Serial.print("\t");
      Serial.print("Steer 2: ");
      Serial.print(motor_speed[2]);
      Serial.print("\t");
      Serial.print("Steer 3: ");
      Serial.print(motor_speed[3]);
      Serial.println();

      newData = false;
  }
}

////////////////////////////////////////////////////////////////////
//        Currently tuning PID gains -- Works but iffy            //
////////////////////////////////////////////////////////////////////

float PID(float setpoint, float current_value) {
  if (setpoint == 0.0) setpoint = 0.1;
  
  error = current_value / setpoint;
  sum += error;

  if (sum > 0.1) sum = 0.1;
  else if (sum < -0.1) sum = -0.1;

  output = p_gain * error + i_gain * sum + d_gain * (previous - error);
  previous = error;

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

  rotate_angle[0] -= M_PI / 4;
  rotate_angle[1] += M_PI / 4;
  rotate_angle[2] -= M_PI / 4;
  rotate_angle[3] += M_PI / 4;

  p_gain = 2.1;
  i_gain = 0.25;
  d_gain = 0.1;

  for (int i = 0; i < 8; i++) {
    motors[i].attach(i + 32);   // Attach motors -- Pins 32 -> 39
    pinMode(i + 32, OUTPUT); // 22
  }
}

////////////////////////////////////////////////////////////////////
//             Init ROS Node, advertise & subscribe.              //
////////////////////////////////////////////////////////////////////

void ros_init() {
  nh.initNode();  // Create ROS nodehandle

  nh.subscribe(joy_sub);    // Subscribe to joy controller message
  
  nh.advertise(temp_pub);       // Debugging purposes -- Float32
  nh.advertise(testing_pub);    // Debugging purposes -- Float32
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

