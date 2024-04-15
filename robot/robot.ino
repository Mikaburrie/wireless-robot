// Mika Burmester

// Import Robot struct
#include "robotUtil.h"

// Import receiver
#include "txrx_settings.h"
#include "rx_analog.h"

void setup() {
//  Serial.begin(9600);

  // Initialize receiver on pin A0
  rxInit(0);
  
  // Initialize motors
  Robot.init();
  Robot.calibrate();
}

#define SYNC_TIMEOUT 500000
unsigned long lastSynced = 0;

void loop() {
  // Update robot and motors
  Robot.update();

  uint8_t x;
  uint8_t y;

  // Drive if receiving data
  if (rxRecv(&y, &x) == RX_SYNC) {
    lastSynced = micros();
    float power = 1 - y/128.0;
    float direction = x/128.0 - 1;

    if (abs(power) < 0.2) power = 0;
    if (abs(direction) < 0.2) direction = 0;

  //  if (abs(power) > abs(direction)) Robot.setPower(power, power);
  //  else Robot.setPower(direction, -direction);
    Robot.setPower(power + direction, power - direction);

  } else if (micros() - lastSynced > SYNC_TIMEOUT) {
    Robot.setPower(0, 0);
  }

  // Serial.print(x);
  // Serial.print(" ");
  // Serial.println(y);

  delay(10);
  
}
