/*
  =========================================================================
  STM32 Black Pill Rover Firmware
  Board      : STM32F401CC / STM32F411 (Black Pill)
  Motor Driver: L298N
  Bluetooth   : HC-06 (Serial1, 9600 baud)
  USB Debug   : Serial (115200 baud)

  Commands:
    F  Forward
    B  Backward
    L  Left
    R  Right
    G  Forward Left
    I  Forward Right
    H  Back Left
    J  Back Right
    S  Stop
    V0-V10  Speed level (mapped to PWM 0-255)

  Non-blocking, byte-by-byte parser. No String, no delay(), no
  readStringUntil().
  =========================================================================
*/

#include <Arduino.h>

// ============================================================
// Pin Definitions (L298N)
// ============================================================
#define ENA   PB6   // Left motor PWM  (TIM1 CH1)
#define IN1   PA0   // Left motor dir 1
#define IN2   PA1  // Left motor dir 2

#define ENB   PA8   // Right motor PWM (TIM3 CH3)
#define IN3   PA3   // Right motor dir 1
#define IN4   PA2   // Right motor dir 2

// ============================================================
// Bluetooth Serial (HC-06)
// ============================================================
#define BT_SERIAL     Serial1
#define BT_BAUD       9600
#define USB_BAUD      115200

// ============================================================
// Globals
// ============================================================
uint8_t currentPWM = 255;          // Default PWM (speed level 5 approx.)
const uint8_t defaultPWM = 255;

// Non-blocking speed parser state
bool     speedParseActive = false;
uint8_t  speedDigitBuffer  = 0;    // accumulated numeric value
uint8_t  speedDigitCount   = 0;    // number of digits accumulated
uint32_t lastSpeedByteTime = 0;
const uint32_t SPEED_PARSE_TIMEOUT_MS = 40;

// ============================================================
// Function Prototypes
// ============================================================
void readUSB();
void readBluetooth();
void processCommand(char cmd, const char* source);
void handleSpeedByte(char c, const char* source);
void finalizeSpeed(const char* source);
void setSpeed(uint8_t level);
void forward();
void backward();
void left();
void right();
void forwardLeft();
void forwardRight();
void backwardLeft();
void backwardRight();
void stopMotor();
void logMessage(const char* msg);
void logCommand(char cmd, const char* source);
void applyMotorPWM();
void motorsInit();

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(USB_BAUD);
  BT_SERIAL.begin(BT_BAUD);

  motorsInit();
  stopMotor();  // Motor immediately stops during startup

  logMessage("====================");
  logMessage("STM32 Rover Ready");
  logMessage("USB Connected");
  logMessage("Bluetooth Ready");

  char buf[24];
  snprintf(buf, sizeof(buf), "Default PWM: %d", defaultPWM);
  logMessage(buf);

  logMessage("====================");
}

// ============================================================
// Main Loop
// ============================================================
void loop() {
  readUSB();
  readBluetooth();

  // Timeout-based finalization for single-digit speed commands
  if (speedParseActive && speedDigitCount > 0) {
    if ((millis() - lastSpeedByteTime) >= SPEED_PARSE_TIMEOUT_MS) {
      finalizeSpeed("TIMEOUT");
    }
  }
}

// ============================================================
// Input Readers (Non-blocking)
// ============================================================
void readUSB() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') continue; // ignore stray newlines
    processCommand(c, "USB");
  }
}

void readBluetooth() {
  while (BT_SERIAL.available() > 0) {
    char c = (char)BT_SERIAL.read();
    if (c == '\r' || c == '\n') continue; // ignore stray newlines
    processCommand(c, "BT");
  }
}

// ============================================================
// Command Processing
// ============================================================
void processCommand(char cmd, const char* source) {
  // If we are currently parsing a speed value, route digits there
  if (speedParseActive) {
    if (cmd >= '0' && cmd <= '9') {
      handleSpeedByte(cmd, source);
      return;
    } else {
      // Non-digit received while parsing speed: finalize speed first
      finalizeSpeed(source);
      // fall through to process this new character normally
    }
  }

  // Start of a speed command
  if (cmd == 'V' || cmd == 'v') {
    logCommand(cmd, source);
    speedParseActive   = true;
    speedDigitBuffer    = 0;
    speedDigitCount     = 0;
    lastSpeedByteTime   = millis();
    return;
  }

  // Log the received command
  logCommand(cmd, source);

  switch (cmd) {
    case 'F': forward();        break;
    case 'B': backward();       break;
    case 'L': left();           break;
    case 'R': right();          break;
    case 'G': forwardLeft();    break;
    case 'I': forwardRight();   break;
    case 'H': backwardLeft();   break;
    case 'J': backwardRight();  break;
    case 'S': stopMotor();      break;
    default: {
      char buf[32];
      snprintf(buf, sizeof(buf), "UNKNOWN COMMAND: %c", cmd);
      logMessage(buf);
      break;
    }
  }
}

// ============================================================
// Speed Parsing Helpers
// ============================================================
void handleSpeedByte(char c, const char* source) {
  (void)source;
  uint8_t digit = (uint8_t)(c - '0');

  // Max speed level is 10 -> max 2 digits
  if (speedDigitCount < 2) {
    speedDigitBuffer = (speedDigitBuffer * 10) + digit;
    speedDigitCount++;
    lastSpeedByteTime = millis();
  }

  // If value already reached or exceeded 10, or 2 digits captured,
  // finalize immediately (covers "V10" fully typed).
  if (speedDigitBuffer >= 10 || speedDigitCount >= 2) {
    finalizeSpeed(source);
  }
}

void finalizeSpeed(const char* source) {
  (void)source;
  if (speedDigitCount == 0) {
    // 'V' received with no digits - ignore silently
    speedParseActive = false;
    return;
  }

  uint8_t level = speedDigitBuffer;
  if (level > 10) level = 10;

  setSpeed(level);

  speedParseActive  = false;
  speedDigitBuffer  = 0;
  speedDigitCount   = 0;
}

// ============================================================
// Speed Control
// ============================================================
void setSpeed(uint8_t level) {
  if (level > 10) level = 10;

  currentPWM = (uint8_t)((uint16_t)level * 255 / 10);
  applyMotorPWM();

  char buf[16];
  logMessage("ACTION : SPEED");

  snprintf(buf, sizeof(buf), "LEVEL : %d", level);
  logMessage(buf);

  snprintf(buf, sizeof(buf), "PWM : %d", currentPWM);
  logMessage(buf);
}

void applyMotorPWM() {
  analogWrite(ENA, currentPWM);
  analogWrite(ENB, currentPWM);
}

// ============================================================
// Motor Primitive Control
// ============================================================
void motorsInit() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  applyMotorPWM();
  logMessage("ACTION : FORWARD");
}

void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  applyMotorPWM();
  logMessage("ACTION : BACKWARD");
}

void left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  applyMotorPWM();
  logMessage("ACTION : LEFT");
}

void right() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  applyMotorPWM();
  logMessage("ACTION : RIGHT");
}

void forwardLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, currentPWM / 2);
  analogWrite(ENB, currentPWM);
  logMessage("ACTION : FORWARD LEFT");
}

void forwardRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, currentPWM);
  analogWrite(ENB, currentPWM / 2);
  logMessage("ACTION : FORWARD RIGHT");
}

void backwardLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, currentPWM / 2);
  analogWrite(ENB, currentPWM);
  logMessage("ACTION : BACKWARD LEFT");
}

void backwardRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, currentPWM);
  analogWrite(ENB, currentPWM / 2);
  logMessage("ACTION : BACKWARD RIGHT");
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  logMessage("ACTION : STOP");
}

// ============================================================
// Logging
// ============================================================
void logMessage(const char* msg) {
  Serial.println(msg);
  BT_SERIAL.println(msg);
}

void logCommand(char cmd, const char* source) {
  char buf[32];
  snprintf(buf, sizeof(buf), "[%s] RX: %c", source, cmd);
  Serial.println(buf);
  BT_SERIAL.println(buf);
}