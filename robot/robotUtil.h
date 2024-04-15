// Mika Burmester

// Distances in micrometers
#define WHEELBASE 150000
#define WHEEL_DIAMETER 62000
#define TICKS_PER_ROTATION 20
#define PI 3.14159
#define WHEEL_CIRCUMFERENCE PI*WHEEL_DIAMETER
#define DISTANCE_PER_TICK (WHEEL_CIRCUMFERENCE/TICKS_PER_ROTATION)

// Time in microseconds
#define MINIMUM_ENCODER_INTERVAL 5000UL
#define MAXIMUM_ENCODER_INTERVAL 200000UL

#define ENCODER_P 1.5
#define VELOCITY_BALANCE 2


// Definitions for Motor
struct Motor;
void _motor_init(Motor*, uint8_t, uint8_t, uint8_t, uint8_t, void (*)());
void _motor_calibrate(Motor*);
void _motor_update(Motor*, float);
void _motor_update_encoder(Motor*);
void _motor_set_velocity(Motor*, float);

struct Motor {
  uint8_t pinPower;
  uint8_t pinForward;
  uint8_t pinBackward;
  uint8_t pinEncoder;

  int16_t encoderTick = 0;
  unsigned long encoderTime = 0;

  float velocityTarget = 0;
  float velocityOutput = 0;
  float velocityMeasured = 0;
  float speedMax = 0;

  uint8_t powerMin = 0;
  uint8_t powerStart = 0;

  void (*init)(Motor*, uint8_t, uint8_t, uint8_t, uint8_t, void (*)()) = &_motor_init;
  void (*calibrate)(Motor*) = &_motor_calibrate;
  void (*update)(Motor*, float) = &_motor_update;
  void (*setVelocity)(Motor*, float) = &_motor_set_velocity;
  void (*updateEncoder)(Motor*) = &_motor_update_encoder;
};

// Configures motor pins and encoder interrupt
void _motor_init(Motor* m, uint8_t pinPower, uint8_t pinForward, uint8_t pinBackward, uint8_t pinEncoder, void (*onEncoderTick)()) {
  // Store pins
  m->pinPower = pinPower;
  m->pinForward = pinForward;
  m->pinBackward = pinBackward;
  m->pinEncoder = pinEncoder;

  // Configure motor pins
  pinMode(m->pinPower, OUTPUT);
  pinMode(m->pinForward, OUTPUT);
  pinMode(m->pinBackward, OUTPUT);

  // Configure interrupts for encoders
  pinMode(m->pinEncoder, INPUT);
  attachInterrupt(digitalPinToInterrupt(m->pinEncoder), onEncoderTick, RISING);

  // Zero speed
  m->setVelocity(m, 0);
}

// Calculates powerStart, powerMin, and speedMax for a motor
void _motor_calibrate(Motor* m) {
  // Set forward and reset encoder
  digitalWrite(m->pinForward, HIGH);
  digitalWrite(m->pinBackward, LOW);
  m->velocityMeasured = 0;

  // Increase power until motor moves and store result
  for (uint8_t power = 0; power < 255; power++) {
    if (m->velocityMeasured != 0) {
      m->powerStart = min(power + 10, 255);
      break;
    }
    analogWrite(m->pinPower, power);
    delay(16);
  }

  // Drive at full power and measure speed
  analogWrite(m->pinPower, 255);
  delay(1000);

  float speed = 0;
  for (uint8_t i = 0; i < 64; i++) {
    speed += m->velocityMeasured;
    delay(10);
  }

  m->speedMax = speed/64;

  // Decrease power until motor stops and store result
  for (uint8_t power = m->powerStart; power > 0; power--) {
    if ((micros() - m->encoderTime) > MAXIMUM_ENCODER_INTERVAL) {
      m->powerMin = power + 10;
      break;
    }
    analogWrite(m->pinPower, --power);
    delay(50);
  }

  m->velocityTarget = 0;
}

// Determines motor speed and adjusts output to match target
void _motor_update(Motor* m, float delta) {
  // Assume motor stopped if no encoder tick occurs in a certain interval
  if ((micros() - m->encoderTime) > MAXIMUM_ENCODER_INTERVAL) m->velocityMeasured = 0;
  
  // Update PID loop (really just P)
  float error = m->velocityTarget - m->velocityMeasured;
  if (m->velocityTarget == 0) m->velocityOutput = 0;
  else m->velocityOutput += ENCODER_P*error*delta;

  // Limit speed range
  m->velocityOutput = max(-m->speedMax, min(m->velocityOutput, m->speedMax));

  // Calculate power output
  uint16_t power = (uint16_t) 255*abs(m->velocityOutput)/m->speedMax; // Speed -> power
  if (m->powerMin < power && power < m->powerStart && abs(m->velocityMeasured) < 0.01) power = m->powerStart; // Start push
  power = min(power, 255)*(power > m->powerMin); // Range

  // Set motor power
  digitalWrite(m->pinForward, m->velocityOutput > 0 ? HIGH : LOW);
  digitalWrite(m->pinBackward, m->velocityOutput <= 0 ? HIGH : LOW);
  analogWrite(m->pinPower, power);
}

// Determines speed of motor
void _motor_update_encoder(Motor* m) {
  unsigned long now = micros();
  unsigned long delta = now - m->encoderTime;

  // Ignore tick if too fast
  if (delta < MINIMUM_ENCODER_INTERVAL) return;

  // Handle tick
  int8_t direction = (m->velocityOutput < 0 ? -1 : 1);
  m->velocityMeasured = DISTANCE_PER_TICK/delta*direction;
  m->encoderTick += direction;
  m->encoderTime = now;
}

// Sets the motor speed
void _motor_set_velocity(Motor* m, float velocity) {
  float speed = abs(velocity);

  // Zero power if below minimum
  if (speed < (m->speedMax*m->powerMin/255)) {
    m->velocityTarget = 0;
    return;
  }

  // Limit to max speed and set
  if (speed > m->speedMax) velocity = m->speedMax*(velocity < 0 ? -1 : 1);
  m->velocityTarget = velocity;
}



// Definitions for Robot
void _robot_init();
void _robot_calibrate();
void _robot_update();
void _robot_set_power(float, float);
int _robot_get_ultrasonic_distance(int);
void _robot_delay(int);
void _robot_left_encoder_tick();
void _robot_right_encoder_tick();

struct _Robot {
  struct Motor motorLeft;
  struct Motor motorRight;

  unsigned long lastUpdate = 0;
  float minSpeed = 0;
  float maxSpeed = 0;
  float minPower = 0;

  uint8_t pinTrigger = 12;
  uint8_t pinEcho = 13;
  
  // Functions
  void (*init)() = &_robot_init;
  void (*calibrate)() = &_robot_calibrate;
  void (*update)() = &_robot_update;
  void (*setPower)(float, float) = &_robot_set_power;
  int (*getUltrasonicDistance)(int) = &_robot_get_ultrasonic_distance;
  void (*delay)(int) = &_robot_delay;
} Robot;

void _robot_init() {
  Robot.motorLeft.init(&Robot.motorLeft, 9, 8, 4, 2, &_robot_left_encoder_tick);
  Robot.motorRight.init(&Robot.motorRight, 6, 5, 7, 3, &_robot_right_encoder_tick);
}

void _robot_calibrate() {
  Robot.motorLeft.calibrate(&Robot.motorLeft);
  Robot.motorRight.calibrate(&Robot.motorRight);
  Robot.maxSpeed = min(Robot.motorLeft.speedMax, Robot.motorRight.speedMax);
  Robot.minPower = max(Robot.motorLeft.powerMin, Robot.motorRight.powerMin)/255.0;
  Robot.minSpeed = Robot.maxSpeed*Robot.minPower;
}

void _robot_update() {
  unsigned long now = micros();
  float delta = (now - Robot.lastUpdate)/1000000.0;
  Robot.lastUpdate = now;

  // Match motor speed ratios
  if (Robot.motorLeft.velocityTarget != 0 && Robot.motorRight.velocityTarget != 0) {
    float ratio = Robot.motorLeft.velocityTarget/Robot.motorRight.velocityTarget;
    float leftSpeedError = Robot.motorRight.velocityMeasured*ratio - Robot.motorLeft.velocityMeasured;
    float rightSpeedError = Robot.motorLeft.velocityMeasured/ratio - Robot.motorRight.velocityMeasured;
    Robot.motorLeft.velocityOutput += leftSpeedError*VELOCITY_BALANCE*delta;
    Robot.motorRight.velocityOutput += rightSpeedError*VELOCITY_BALANCE*delta;
  }

  Robot.motorLeft.update(&Robot.motorLeft, delta);
  Robot.motorRight.update(&Robot.motorRight, delta);
}

void _robot_set_power(float left, float right) {
  Robot.motorLeft.setVelocity(&Robot.motorLeft, left*Robot.maxSpeed);
  Robot.motorRight.setVelocity(&Robot.motorRight, right*Robot.maxSpeed);
}

int _robot_get_ultrasonic_distance(int samples) {
  unsigned long totalTime = 0;
  
  for (int sample = 0; sample < samples; sample++) {
    // Send pulse
    digitalWrite(Robot.pinTrigger, LOW);
    delayMicroseconds(2);
    digitalWrite(Robot.pinTrigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(Robot.pinTrigger, LOW);

    // Wait for echo
    unsigned long delay = pulseIn(Robot.pinEcho, HIGH);
    totalTime += delay;
    delayMicroseconds(10000 - delay);
  }

  unsigned long averageTime = totalTime/samples;
  return (int) min(max(0, averageTime/55 - 2), 255);
}

void _robot_delay(int milliseconds) {
  unsigned long start = millis();
  unsigned long now = start;
  do {
    delay(min(10, start + milliseconds - now));
    Robot.update();
    now = millis();
  } while (now - start < milliseconds);
}

void _robot_left_encoder_tick() {
  Robot.motorLeft.updateEncoder(&Robot.motorLeft);
}

void _robot_right_encoder_tick() {
  Robot.motorRight.updateEncoder(&Robot.motorRight);
}
