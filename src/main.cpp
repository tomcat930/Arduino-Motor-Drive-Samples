#include <Arduino.h>

const int IN1_pin = 9;   // 9番ピンを出力ピンIN1と定義
const int IN2_pin = 8;   // 8番ピンを出力ピンIN2と定義
const int ENA_pin = 10;  // 10番ピンを出力ピンENAと定義

const int lmSW_L_pin = 12;  // 11番ピンをLMスイッチ1と定義
const int lmSW_R_pin = 11;  // 12番ピンをLMスイッチ2と定義

const int mainSW_pin = 7;

bool lmSW_L_mode = false;
bool lmSW_R_mode = false;
bool mainSW_mode = false;
bool mainSW_status = false;

int mode = 0;
int old_mode;
int drive_ready = 0;
int motor_speed = 255;

const int stop_mode = 0;
const int forward_mode = 1;
const int back_mode = 2;

unsigned long startTime;
unsigned long currentTime;
unsigned long operationTime;
const unsigned long period = 5000;  // 動作継続時間(秒)

void setup() {
  Serial.begin(115200);
  Serial.println("Ready...");

  pinMode(IN1_pin, OUTPUT);
  pinMode(IN2_pin, OUTPUT);
  pinMode(ENA_pin, OUTPUT);
  pinMode(lmSW_L_pin, INPUT_PULLUP);
  pinMode(lmSW_R_pin, INPUT_PULLUP);
  pinMode(mainSW_pin, INPUT_PULLUP);

  delay(3000);
}

void motorStop() {
  digitalWrite(IN1_pin, LOW);
  digitalWrite(IN2_pin, LOW);
}

void forwardDrive() {
  digitalWrite(IN1_pin, HIGH);
  digitalWrite(IN2_pin, LOW);
  analogWrite(ENA_pin, motor_speed);
}

void backDrive() {
  digitalWrite(IN1_pin, LOW);
  digitalWrite(IN2_pin, HIGH);
  analogWrite(ENA_pin, motor_speed);
}

void safetyTimerStop() {
  if (startTime == 0) {
    startTime = currentTime;
  }
  if (currentTime >= startTime + period) {
    Serial.println("Safety stopped.");
    drive_ready = 0;
    startTime = 0;
    mode = 0;
  }
}

void motorDrive(int IN1, int IN2) {
  safetyTimerStop();
  digitalWrite(IN1_pin, IN1);
  digitalWrite(IN2_pin, IN2);
  analogWrite(ENA_pin, motor_speed);
}

void loop() {
  currentTime = millis();
  lmSW_L_mode = !digitalRead(lmSW_L_pin);
  lmSW_R_mode = !digitalRead(lmSW_R_pin);
  mainSW_mode = !digitalRead(mainSW_pin);

  if (mainSW_mode == true && mainSW_status == false) {
    drive_ready = 1 - drive_ready;
    if (drive_ready == 0) {
      mode = stop_mode;
    }
  }
  mainSW_status = mainSW_mode;

  if (drive_ready) {
    if (lmSW_L_mode == false && lmSW_R_mode == true) {
      mode = forward_mode;
    } else if (lmSW_L_mode == true && lmSW_R_mode == false) {
      mode = back_mode;
    }

    if (mode != old_mode) {
      startTime = 0;
    }

    switch (mode) {
      case stop_mode:
        motorStop();
        Serial.println("standby mode");
        break;

      case forward_mode:
        motorDrive(HIGH, LOW);
        Serial.println("foward drive");
        break;

      case back_mode:
        motorDrive(LOW, HIGH);
        Serial.println("back drive");
        break;

      default:
        break;
    }
  } else {
    motorStop();
    Serial.println("Stop mode...");
  }
  old_mode = mode;
  delay(20);
}