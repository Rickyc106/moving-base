#include <EasyTransfer.h>

EasyTransfer ETin_pyserial, ETout_pyserial;

struct PYSERIAL_RECEIVE_DATA_STRUCTURE {
  float data[8];
};

struct PYSERIAL_SEND_DATA_STRUCTURE {
  float data[24];
};

PYSERIAL_RECEIVE_DATA_STRUCTURE rx_serial;
PYSERIAL_SEND_DATA_STRUCTURE tx_serial;

void flushSerial() {
  for (int i = 0; i < 5; i++) {
    while (Serial.available() > 0) {
      char k = Serial.read();
      delay(1);
    }
   delay(1);
  }
}

void setup() {
  Serial.begin(115200);

  flushSerial();
  ETout_pyserial.begin(details(tx_serial), &Serial);
  ETin_pyserial.begin(details(rx_serial), &Serial);
}

void loop() {
  ETin_pyserial.receiveData();

  for (int i = 0; i < 8; i++) {
    tx_serial.data[i] = rx_serial.data[i] + i;
    tx_serial.data[i+8] = rx_serial.data[i] + i*2;
    tx_serial.data[i+16] = rx_serial.data[i] + i*3;
  }

  ETout_pyserial.sendData();
}
