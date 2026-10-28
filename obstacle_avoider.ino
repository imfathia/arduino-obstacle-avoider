
#include <Servo.h>

// =====================================================
// Arduino Uno - 4WD Obstacle Avoider
// Hardware:
// - Arduino Uno
// - Arduino Motor Shield Rev3 (L298P)
// - HC-SR04 ultrasonic sensor
// - SG90 microservo
// - 4 DC motors: 2 left + 2 right
// =====================================================

// ---------------- SENSOR ----------------
constexpr uint8_t TRIG_PIN = 2;
constexpr uint8_t ECHO_PIN = 4;

// ---------------- SERVO -----------------
constexpr uint8_t SERVO_PIN = 10;

constexpr uint8_t SERVO_CENTER = 90;
constexpr uint8_t SERVO_LEFT   = 150;
constexpr uint8_t SERVO_RIGHT  = 30;

// ---------------- MOTOR SHIELD -----------
// Channel A
constexpr uint8_t MOTOR_A_DIR   = 12;
constexpr uint8_t MOTOR_A_PWM   = 3;
constexpr uint8_t MOTOR_A_BRAKE = 9;

// Channel B
constexpr uint8_t MOTOR_B_DIR   = 13;
constexpr uint8_t MOTOR_B_PWM   = 11;
constexpr uint8_t MOTOR_B_BRAKE = 8;

// Set true if the corresponding side runs backward
// when the robot is commanded to move forward.
constexpr bool INVERT_LEFT_MOTOR  = false;
constexpr bool INVERT_RIGHT_MOTOR = false;

// ---------------- BEHAVIOUR --------------
constexpr uint8_t FORWARD_SPEED = 155;
constexpr uint8_t TURN_SPEED    = 165;
constexpr uint8_t REVERSE_SPEED = 140;

constexpr uint16_t SAFE_DISTANCE_CM = 25;
constexpr uint16_t SENSOR_TIMEOUT_US = 25000;

constexpr uint16_t REVERSE_TIME_MS = 350;
constexpr uint16_t TURN_TIME_MS    = 400;
constexpr uint16_t SCAN_SETTLE_MS  = 250;

constexpr uint16_t LOOP_DELAY_MS = 40;

// Distances above this are treated as clear space.
constexpr uint16_t MAX_VALID_DISTANCE_CM = 400;

// ---------------- OBJECTS ----------------
Servo scanner;

// ---------------- STATE ------------------
uint32_t lastDecisionTime = 0;

// =====================================================
// MOTOR CONTROL
// =====================================================

void setupMotorShield() {
  pinMode(MOTOR_A_DIR, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_A_BRAKE, OUTPUT);

  pinMode(MOTOR_B_DIR, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_B_BRAKE, OUTPUT);

  // Start safely with both channels stopped.
  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);

  digitalWrite(MOTOR_A_BRAKE, HIGH);
  digitalWrite(MOTOR_B_BRAKE, HIGH);
}

void setMotor(
  uint8_t dirPin,
  uint8_t pwmPin,
  uint8_t brakePin,
  int speed,
  bool inverted
) {
  speed = constrain(speed, -255, 255);

  if (speed == 0) {
    analogWrite(pwmPin, 0);
    digitalWrite(brakePin, HIGH);
    return;
  }

  bool forward = (speed > 0);

  if (inverted) {
    forward = !forward;
  }

  digitalWrite(brakePin, LOW);
  digitalWrite(dirPin, forward ? HIGH : LOW);

  analogWrite(pwmPin, abs(speed));
}

void drive(int leftSpeed, int rightSpeed) {
  setMotor(
    MOTOR_A_DIR,
    MOTOR_A_PWM,
    MOTOR_A_BRAKE,
    leftSpeed,
    INVERT_LEFT_MOTOR
  );

  setMotor(
    MOTOR_B_DIR,
    MOTOR_B_PWM,
    MOTOR_B_BRAKE,
    rightSpeed,
    INVERT_RIGHT_MOTOR
  );
}

void stopMotors() {
  drive(0, 0);
}

void moveForward() {
  drive(FORWARD_SPEED, FORWARD_SPEED);
}

void moveBackward() {
  drive(-REVERSE_SPEED, -REVERSE_SPEED);
}

void turnLeft() {
  // Differential drive: left side backward,
  // right side forward.
  drive(-TURN_SPEED, TURN_SPEED);
}

void turnRight() {
  drive(TURN_SPEED, -TURN_SPEED);
}

// =====================================================
// ULTRASONIC SENSOR
// =====================================================

uint16_t readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(
    ECHO_PIN,
    HIGH,
    SENSOR_TIMEOUT_US
  );

  // No echo: do not interpret this as confirmed
  // clear space.
  if (duration == 0) {
    return 0;
  }

  uint16_t distance = duration / 58UL;

  if (distance == 0 ||
      distance > MAX_VALID_DISTANCE_CM) {
    return 0;
  }

  return distance;
}

uint16_t readStableDistanceCm() {
  // Take three readings and use the median
  // to reduce occasional measurement noise.
  uint16_t values[3];

  for (uint8_t i = 0; i < 3; i++) {
    values[i] = readDistanceCm();

    if (i < 2) {
      delay(25);
    }
  }

  // Sort three values in ascending order.
  for (uint8_t i = 0; i < 2; i++) {
    for (uint8_t j = i + 1; j < 3; j++) {
      if (values[j] < values[i]) {
        uint16_t temp = values[i];
        values[i] = values[j];
        values[j] = temp;
      }
    }
  }

  // A zero reading means a failed measurement.
  // If the median is zero, report an invalid reading.
  return values[1];
}

// =====================================================
// SERVO SCANNING
// =====================================================

uint16_t scanDirection(uint8_t angle) {
  stopMotors();

  scanner.write(angle);
  delay(SCAN_SETTLE_MS);

  return readStableDistanceCm();
}

// =====================================================
// OBSTACLE AVOIDANCE
// =====================================================

void avoidObstacle() {
  stopMotors();
  delay(120);

  // First create space between the robot and obstacle.
  moveBackward();
  delay(REVERSE_TIME_MS);
  stopMotors();
  delay(120);

  // Check the left side.
  uint16_t leftDistance = scanDirection(SERVO_LEFT);

  // Check the right side.
  uint16_t rightDistance = scanDirection(SERVO_RIGHT);

  // Return the sensor to the forward position.
  scanner.write(SERVO_CENTER);
  delay(SCAN_SETTLE_MS);

  Serial.print(F("Left: "));
  Serial.print(leftDistance);
  Serial.print(F(" cm | Right: "));
  Serial.print(rightDistance);
  Serial.println(F(" cm"));

  // A zero reading is considered unknown,
  // not proof that a path is clear.
  bool leftClear =
    leftDistance > SAFE_DISTANCE_CM;

  bool rightClear =
    rightDistance > SAFE_DISTANCE_CM;

  if (leftClear && rightClear) {
    // Prefer the side with more measured space.
    if (leftDistance > rightDistance) {
      turnLeft();
    } else {
      turnRight();
    }

    delay(TURN_TIME_MS);
  } else if (leftClear) {
    turnLeft();
    delay(TURN_TIME_MS);
  } else if (rightClear) {
    turnRight();
    delay(TURN_TIME_MS);
  } else {
    // Both sides appear blocked or measurements
    // are invalid. Stop rather than blindly advancing.
    stopMotors();
    Serial.println(F("No confirmed clear path."));
    delay(250);
  }

  stopMotors();
  scanner.write(SERVO_CENTER);
  delay(100);
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  setupMotorShield();

  scanner.attach(SERVO_PIN);
  scanner.write(SERVO_CENTER);

  delay(500);

  Serial.println(F("4WD Obstacle Avoider"));
  Serial.println(F("System initialized."));
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  uint16_t distance = readStableDistanceCm();

  Serial.print(F("Front distance: "));
  Serial.print(distance);
  Serial.println(F(" cm"));

  // Invalid measurements trigger a safe stop.
  if (distance == 0) {
    stopMotors();
    delay(100);
    return;
  }

  if (distance <= SAFE_DISTANCE_CM) {
    avoidObstacle();
  } else {
    moveForward();
  }

  delay(LOOP_DELAY_MS);
}
