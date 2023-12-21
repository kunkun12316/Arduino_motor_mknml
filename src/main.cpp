#include <Arduino.h>
#include <A4988.h>
#include <BasicStepperDriver.h>

// 一般步进电机的步距角是1.8度。一转需要200个脉冲
#define MOTOR_STEPS 200
#define RPM 60
// 细分数16.这样一来200*16=3200个脉冲电机才能转一圈。注意拨码开关也要拨到想通档位
#define MICROSTEPS 16

#define L1_motor_STRP 2
#define L1_motor_DIR 3
#define R1_motor_STRP 4
#define R1_motor_DIR 5
#define L2_motor_STRP 6
#define L2_motor_DIR 7
#define R2_motor_STRP 8
#define R2_motor_DIR 9

#define MS1_PIN 22
#define MS2_PIN 24
#define MS3_PIN 26

enum Mode
{
  CONSTANT_SPEED,
  LINEAR_SPEED
};

A4988 L1_motor(MOTOR_STEPS,L1_motor_STRP,L1_motor_DIR,MS1_PIN,MS2_PIN,MS3_PIN);
A4988 R1_motor(MOTOR_STEPS,R1_motor_STRP,R1_motor_DIR,MS1_PIN,MS2_PIN,MS3_PIN);
A4988 L2_motor(MOTOR_STEPS,L2_motor_STRP,L2_motor_DIR,MS1_PIN,MS2_PIN,MS3_PIN);
A4988 R2_motor(MOTOR_STEPS,R2_motor_STRP,R2_motor_DIR,MS1_PIN,MS2_PIN,MS3_PIN);

int motor_regulation_status = LINEAR_SPEED; 
short motor_accelerate = 2000;
short motor_slow_down = 2000;

void setup() {
  Serial.begin(115200);
  
  // 设置 A4988 驱动器的微步数和转速
  L1_motor.begin(RPM, MICROSTEPS);  // 60 RPM，8 微步
  R1_motor.begin(RPM, MICROSTEPS);
  L2_motor.begin(RPM, MICROSTEPS);
  R2_motor.begin(RPM, MICROSTEPS);
  L1_motor.setSpeedProfile(static_cast<BasicStepperDriver::Mode>(motor_regulation_status), motor_accelerate, motor_slow_down); //配置加减速曲线
  R1_motor.setSpeedProfile(static_cast<BasicStepperDriver::Mode>(motor_regulation_status), motor_accelerate, motor_slow_down);
  L2_motor.setSpeedProfile(static_cast<BasicStepperDriver::Mode>(motor_regulation_status), motor_accelerate, motor_slow_down);
  R2_motor.setSpeedProfile(static_cast<BasicStepperDriver::Mode>(motor_regulation_status), motor_accelerate, motor_slow_down);
}

void loop() {
  // put your main code here, to run repeatedly:
    // 以指定速度和方向运行步进电机
  L1_motor.setSpeed(100);  // 100 步/秒
  L1_motor.runSpeed();

  delay(1000);  // 等待 1 秒
}

