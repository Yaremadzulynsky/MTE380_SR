#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

// Set to 1 for serial debug prints.
#ifndef ENABLE_SERIAL_DEBUG
#define ENABLE_SERIAL_DEBUG 1
#endif

constexpr uint8_t I2C_ADDRESS = 0x08;
constexpr uint8_t PWM_CAP = 160;                 // Max motor PWM (0..255)
constexpr int8_t VECTOR_DEADBAND = 8;            // Ignore small vector noise
constexpr unsigned long COMMAND_TIMEOUT_MS = 500;

// DRV8871 pin plan on Mega 2560
constexpr uint8_t M1_IN1_PIN = 5;   // PWM-capable
constexpr uint8_t M1_IN2_PIN = 9;   // PWM-capable (rewire from 22 -> 9)
constexpr uint8_t M2_IN1_PIN = 6;   // PWM-capable
constexpr uint8_t M2_IN2_PIN = 10;  // PWM-capable (rewire from 23 -> 10)
constexpr uint8_t STATUS_LED_PIN = LED_BUILTIN;
constexpr uint8_t SERVO_PIN = 4;
constexpr uint8_t SERVO_MIN_ANGLE = 0;
constexpr uint8_t SERVO_MAX_ANGLE = 90;

enum PacketType : uint8_t {
  PACKET_NONE = 0,
  PACKET_VECTOR_XY = 1
};

volatile PacketType g_pendingPacketType = PACKET_NONE;
volatile int8_t g_pendingX = 0;
volatile int8_t g_pendingY = 0;
volatile uint8_t g_pendingServoAngle = SERVO_MIN_ANGLE;
volatile bool g_packetReady = false;
volatile bool g_invalidPacket = false;

unsigned long g_lastVectorCommandMs = 0;
bool g_haveVectorCommand = false;
bool g_motorsStopped = true;
Servo g_servo;
uint8_t g_servoAngle = SERVO_MIN_ANGLE;

static uint8_t computePwmFromVector(int8_t x, int8_t y) {
  const int xi = static_cast<int>(x);
  const int yi = static_cast<int>(y);
  const float magnitude = sqrtf(static_cast<float>(xi * xi + yi * yi));

  // Max vector length when x/y are int8 in [-127, 127].
  constexpr float kMaxMagnitude = 179.61f;
  const float clampedMagnitude = magnitude > kMaxMagnitude ? kMaxMagnitude : magnitude;
  return static_cast<uint8_t>(roundf((clampedMagnitude / kMaxMagnitude) * PWM_CAP));
}

static void stopMotor(uint8_t in1Pin, uint8_t in2Pin) {
  analogWrite(in1Pin, 0);
  analogWrite(in2Pin, 0);
}

static void setMotorForward(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwm) {
  analogWrite(in1Pin, pwm);
  analogWrite(in2Pin, 0);
}

static void setMotorReverse(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwm) {
  analogWrite(in1Pin, 0);
  analogWrite(in2Pin, pwm);
}

static void stopAllMotors() {
  stopMotor(M1_IN1_PIN, M1_IN2_PIN);
  stopMotor(M2_IN1_PIN, M2_IN2_PIN);
  g_motorsStopped = true;
}

static void applyDriveFromVector(int8_t x, int8_t y) {
  const int absX = abs(static_cast<int>(x));
  const int absY = abs(static_cast<int>(y));

  if (absX <= VECTOR_DEADBAND && absY <= VECTOR_DEADBAND) {
    stopAllMotors();
#if ENABLE_SERIAL_DEBUG
    Serial.print("STOP (deadband), x=");
    Serial.print(x);
    Serial.print(", y=");
    Serial.println(y);
#endif
    return;
  }

  bool reverse = false;
  if (y > VECTOR_DEADBAND) {
    reverse = false;
  } else if (y < -VECTOR_DEADBAND) {
    reverse = true;
  } else {
    stopAllMotors();
#if ENABLE_SERIAL_DEBUG
    Serial.print("STOP (no drive direction), x=");
    Serial.print(x);
    Serial.print(", y=");
    Serial.println(y);
#endif
    return;
  }

  const uint8_t basePwm = computePwmFromVector(x, y);
  const float turnRatio = (absX == 0) ? 0.0f : (static_cast<float>(absX) / static_cast<float>(absX + absY));
  const uint8_t innerPwm = static_cast<uint8_t>(roundf(basePwm * (1.0f - turnRatio)));

  // Motor 1 is LEFT, Motor 2 is RIGHT.
  uint8_t leftPwm = basePwm;
  uint8_t rightPwm = basePwm;
  if (x < -VECTOR_DEADBAND) {
    leftPwm = innerPwm;
    rightPwm = basePwm;
  } else if (x > VECTOR_DEADBAND) {
    leftPwm = basePwm;
    rightPwm = innerPwm;
  }

  if (reverse) {
    setMotorReverse(M1_IN1_PIN, M1_IN2_PIN, leftPwm);
    setMotorReverse(M2_IN1_PIN, M2_IN2_PIN, rightPwm);
  } else {
    setMotorForward(M1_IN1_PIN, M1_IN2_PIN, leftPwm);
    setMotorForward(M2_IN1_PIN, M2_IN2_PIN, rightPwm);
  }
  g_motorsStopped = (leftPwm == 0 && rightPwm == 0);

#if ENABLE_SERIAL_DEBUG
  Serial.print(reverse ? "REVERSE x=" : "DRIVE x=");
  Serial.print(x);
  Serial.print(", y=");
  Serial.print(y);
  Serial.print(", base=");
  Serial.print(basePwm);
  Serial.print(", left=");
  Serial.print(leftPwm);
  Serial.print(", right=");
  Serial.println(rightPwm);
#endif
}

void onI2cReceive(int howMany) {
  if (howMany <= 0) {
    return;
  }

  uint8_t bytesRead = 0;
  uint8_t buffer[8];
  while (Wire.available() && bytesRead < sizeof(buffer)) {
    buffer[bytesRead++] = Wire.read();
  }
  while (Wire.available()) {
    (void)Wire.read();
  }

  if (bytesRead == 3) {
    g_pendingX = static_cast<int8_t>(buffer[0]);
    g_pendingY = static_cast<int8_t>(buffer[1]);
    g_pendingServoAngle = buffer[2];
    g_pendingPacketType = PACKET_VECTOR_XY;
    g_packetReady = true;
    return;
  }

  g_invalidPacket = true;
}

void setup() {
#if ENABLE_SERIAL_DEBUG
  Serial.begin(115200);
#endif

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  pinMode(M1_IN1_PIN, OUTPUT);
  pinMode(M1_IN2_PIN, OUTPUT);
  pinMode(M2_IN1_PIN, OUTPUT);
  pinMode(M2_IN2_PIN, OUTPUT);
  stopAllMotors();

  g_servo.attach(SERVO_PIN);
  g_servo.write(g_servoAngle);

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onI2cReceive);

#if ENABLE_SERIAL_DEBUG
  Serial.print("I2C slave ready at 0x");
  Serial.println(I2C_ADDRESS, HEX);
  Serial.println("Packet format: [int8 x][int8 y][uint8 angle_0_to_90]");
#endif
}

void loop() {
  bool hadPacket = false;
  bool hadInvalidPacket = false;
  PacketType packetType = PACKET_NONE;
  int8_t x = 0;
  int8_t y = 0;
  uint8_t servoAngle = SERVO_MIN_ANGLE;

  noInterrupts();
  if (g_packetReady) {
    packetType = g_pendingPacketType;
    x = g_pendingX;
    y = g_pendingY;
    servoAngle = g_pendingServoAngle;
    g_pendingPacketType = PACKET_NONE;
    g_packetReady = false;
    hadPacket = true;
  }
  if (g_invalidPacket) {
    g_invalidPacket = false;
    hadInvalidPacket = true;
  }
  interrupts();

  if (hadInvalidPacket) {
    stopAllMotors();
    digitalWrite(STATUS_LED_PIN, LOW);
#if ENABLE_SERIAL_DEBUG
    Serial.println("Invalid I2C packet -> motors stopped");
#endif
  }

  if (hadPacket) {
    if (packetType == PACKET_VECTOR_XY) {
      g_lastVectorCommandMs = millis();
      g_haveVectorCommand = true;
      applyDriveFromVector(x, y);
      if (servoAngle > SERVO_MAX_ANGLE) {
        servoAngle = SERVO_MAX_ANGLE;
      }
      g_servoAngle = servoAngle;
      g_servo.write(g_servoAngle);
      digitalWrite(STATUS_LED_PIN, g_motorsStopped ? LOW : HIGH);
#if ENABLE_SERIAL_DEBUG
      Serial.print("Packet: x=");
      Serial.print(x);
      Serial.print(", y=");
      Serial.print(y);
      Serial.print(", servo=");
      Serial.println(g_servoAngle);
#endif
    }
  }

  if (g_haveVectorCommand && (millis() - g_lastVectorCommandMs > COMMAND_TIMEOUT_MS)) {
    if (!g_motorsStopped) {
      stopAllMotors();
      digitalWrite(STATUS_LED_PIN, LOW);
#if ENABLE_SERIAL_DEBUG
      Serial.println("Vector command timeout -> motors stopped");
#endif
    }
  }

  delay(5);
}
