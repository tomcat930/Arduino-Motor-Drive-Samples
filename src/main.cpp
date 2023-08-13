#include <Arduino.h>

const int IN1_pin = 9;      // 9番ピンを出力ピンIN1と定義
const int IN2_pin = 8;      // 8番ピンを出力ピンIN2と定義
const int ENA_pin = 10;     // 10番ピンを出力ピンENAと定義
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
int motor_speed = 255;  // モーター速度

const int stop_mode = 0;
const int forward_mode = 1;
const int back_mode = 2;

unsigned long start_time;                         // 動作開始時間
unsigned long current_time;                       // 現在時間
unsigned long operation_time;                     // 動作時間
const unsigned long operation_limit_time = 5000;  // 動作継続時間(秒)

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

/**
 * @brief モーターを停止させる
 * 
 */
void motorStop() {
  digitalWrite(IN1_pin, LOW);
  digitalWrite(IN2_pin, LOW);
}

/**
 * @brief 動作継続時間を超えた場合停止する
 *
 */
void safetyTimerStop() {
  if (start_time == 0) {
    start_time = current_time;
  }
  if (current_time >= start_time + operation_limit_time) {
    Serial.println("Safety stopped.");
    drive_ready = 0;
    start_time = 0;
    mode = stop_mode;
  }
}

/**
 * @brief モーターを動作させる
 *
 * @param IN1 モーターのIN1
 * @param IN2 モーターのIN2
 */
void motorDrive(int IN1, int IN2) {
  safetyTimerStop();
  digitalWrite(IN1_pin, IN1);
  digitalWrite(IN2_pin, IN2);
  analogWrite(ENA_pin, motor_speed);
}

void loop() {
  current_time = millis();
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
      start_time = 0;
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