#include <Servo.h>
#include <EasyTransfer.h>

Servo motors[8];

float steer_command[4], drive_command[4];
float stop_steer[4], stop_drive[4];

float speed_cap = 0.45;   // Percentage Speed Cap
float incoming_value;     // Serial Comms incoming Value
unsigned char buffer[4];  // Serial Comms buffer

EasyTransfer ETin_steer, ETin_drive;
EasyTransfer ETin_pyserial, ETout_pyserial;


struct STEER_DATA_STRUCTURE {
  float encoder_ticks[4];
  float period[4];
  float wheel_position[4];
};

struct DRIVE_DATA_STRUCTURE {
  float encoder_ticks[4];
  float period[4];
  float wheel_position[4];
};


struct PYSERIAL_RECEIVE_DATA_STRUCTURE {
  float data[8];  
};

struct PYSERIAL_SEND_DATA_STRUCTURE {
  float data[8];
  //float ticks[8];
  //float period[8];
  //float pos[8];
};

STEER_DATA_STRUCTURE rx_steer;
DRIVE_DATA_STRUCTURE rx_drive;

PYSERIAL_RECEIVE_DATA_STRUCTURE rx_serial;
PYSERIAL_SEND_DATA_STRUCTURE tx_serial;

void stop_cb() {
  for (int i = 0; i < 4; i++) {
    //stop_steer[i] = msg.data[i * 2];
    //stop_drive[i] = msg.data[(i * 2) + 1];  
  }  
}

void setup() {
  initialize();   // Attach motors, establish serial comms, etc.
}

void loop() { 
  /*
  for (int i = 0; i < 4; i++) {
    ETin_steer.receiveData();

    tx_serial.ticks[i * 2] = rx_steer.encoder_ticks[i];
    tx_serial.period[i * 2] = rx_steer.period[i];
    tx_serial.pos[i * 2] = rx_steer.wheel_position[i];

    ETin_drive.receiveData();

    tx_serial.ticks[(i * 2) + 1] = rx_drive.encoder_ticks[i];
    tx_serial.period[(i * 2) + 1] = rx_drive.period[i];
    tx_serial.pos[(i * 2) + 1] = rx_drive.wheel_position[i];
  }
  */

  for (int i = 0; i < 8; i++) {
    tx_serial.data[i] = rx_serial.data[i];
  }
  
  ETout_pyserial.sendData();
  delay(10);
  ETin_pyserial.receiveData();

  for (int i = 0; i < 8; i++) {
    if (i % 2 == 0) {
      steer_command[i / 2] = rx_serial.data[i];
      steer_command[i / 2] = constrain(steer_command[i / 2], -1.0, 1.0);
      
      //tx_serial.debug[i / 2] = rx_serial.data[i];
    }
    else if (i % 2 == 1) {
      drive_command[(i - 1) / 2] = rx_serial.data[i];
      drive_command[(i - 1) / 2] = constrain(steer_command[(i - 1) / 2], -1.0, 1.0);
      
      //tx_serial.debug[(i - 1) / 2] = rx_serial.data[i];
    }
  }
  
  for (int i = 0; i < 4; i++) {
    motor_debugging(i, steer_command[i], 0);
  }
}

////////////////////////////////////////////////////////////////////
//        Read all bytes in buffer until flushed properly         //
////////////////////////////////////////////////////////////////////

void flushSerial() {
  for (int i = 0; i < 5; i++) {
    while (Serial.available() > 0) {
      char k = Serial.read();
      delay(1);
    }
   delay(1);
  }
}

////////////////////////////////////////////////////////////////////
//        IEEE-754 Floating-Point Format -- Serial Write          //
////////////////////////////////////////////////////////////////////

void serialWrite(float outgoing_value) {
  Serial.write((const char *)&outgoing_value, sizeof(float)/*4*/);
}

////////////////////////////////////////////////////////////////////
//         IEEE-754 Floating-Point Format -- Serial Read          //
////////////////////////////////////////////////////////////////////

/*
bool serialRead(int len) {
  int rx_len = 0;
  int rx_array_index = 0;

  if (Serial.available() >= 3) {
    while (Serial.read() != 0x06) {
      if (Serial.available() < 3) {
        return false;
      }
    }

    if (Serial.read() == 0x85) {
      rx_len = Serial.read();
      if (rx_len != len) {
        rx_len = 0;
        return false;  
      }
    }
  }

  if (rx_len != 0) {
    while (Serial.available() && rx_array_index <= rx_len) {
      rx_buffer[rx_array_index++] = Serial.read();
    }

    if rx_len == (rx_array_index - 1) {
      calc_CS = rx_len;

      for (int i = 0; i < rx_len; i++) {
        calc_CS ^= int(rx_buffer[i]);
      }
    }

    if (calc_CS == rx_buffer[rx_array_index - 1] {
      continue;
    }
  }

  // If we read enough bytes, unpack it
  if (Serial.readBytes(buffer, sizeof(float)) == sizeof(float)) {
    memcpy(&incoming_value, buffer, sizeof(float));
  }
  else {
    // I/O error - no data, not enough bytes, etc.
    incoming_value = 0;
  }

  return incoming_value;
}
*/

////////////////////////////////////////////////////////////////////
//          Attach motors, establish serial comms, etc.           //
////////////////////////////////////////////////////////////////////

void initialize() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  
  ETin_steer.begin(details(rx_steer), &Serial1);
  ETin_drive.begin(details(rx_drive), &Serial2);

  flushSerial();
  
  ETout_pyserial.begin(details(tx_serial), &Serial);
  ETin_pyserial.begin(details(rx_serial), &Serial);

  for (int i = 0; i < 2; i++) {
    motors[i].attach(i + 22);
    motors[i+2].attach(i+48);
    motors[i+4].attach(i+50);
    motors[i+6].attach(i+24);
  }
}

////////////////////////////////////////////////////////////////////
// Motor drive subroutine -- Range: 1000 ms - 2000 ms pulse width //
//     Variable 'var' corresponds to which motor is driven        //
//      Variabel 'steer' is the speed of PG71 Gearmotor           //
//        Variable 'drive' is the speed of CIM motor              //
////////////////////////////////////////////////////////////////////

void motor_debugging(int var, float drive, float steer) {
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
  }
}
