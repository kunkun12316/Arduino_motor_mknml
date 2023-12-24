#include <Arduino.h>
#include <BasicStepperDriver.h> //StepperDriver库

#define X0 15 // PLC输入脚，注意是共阳极输入
#define X1 14 // PLC输入脚，注意是共阳极输入
#define X2 7  // PLC输入脚，注意是共阳极输入，看原理图
#define X3 4  // PLC输入脚，注意是共阳极输入，看原理图
#define X4 3  // PLC输入脚，注意是共阳极输入，可触发中断
#define X5 2  // PLC输入脚，注意是共阳极输入，可触发中断
#define Y0 5  // PLC输出脚，注意是开集电极输出
#define Y1 6  // PLC输出脚，注意是开集电极输出
#define Y2 8  // PLC输出脚，注意是开集电极输出
#define Y3 9  // PLC输出脚，注意是开集电极输出

#define TOTAL_LENGTH 78000 // 轿厢移动最大行程
#define DOOR_LENGTH 35000  // 小门移动最大行程
#define Z_PIN X4           // Z轴限位,轿厢回零中断用
#define D_PIN X5           // 门回零中断用

// 一般步进电机的步距角是1.8度。一转需要200个脉冲
#define MOTOR_STEPS 200
#define RPM1 120
#define RPM2 120
#define RPM3 800
// 细分数16.这样一来200*16=3200个脉冲电机才能转一圈。注意拨码开关也要拨到想通档位
#define MICROSTEPS1 16
#define MICROSTEPS2 16
#define MICROSTEPS3 1

#define DIR1 13
#define STEP1 12

#define DIR2 18
#define STEP2 10

#define DIR3 19
#define STEP3 11

enum Mode
{
  CONSTANT_SPEED,
  LINEAR_SPEED
};
enum State
{
  STOPPED,
  ACCELERATING,
  CRUISING,
  DECELERATING
};
// 声明两个电机对象，分别对应Z轴的两个步进电机
BasicStepperDriver stepper1(MOTOR_STEPS, DIR1, STEP1);
BasicStepperDriver stepper2(MOTOR_STEPS, DIR2, STEP2);
BasicStepperDriver stepper3(MOTOR_STEPS, DIR3, STEP3);

// 定义状态枚举类型
enum State_f
{
  state,                // 空闲状态
  theSmallDoorIsClosed, // 小门关闭状态
  theGondolaDescends,   // 吊箱下降状态
  thePartitionOpens,    // 隔板打开状态
  thePartitionIsClosed, // 隔板关闭状态
  theGondolaRises       // 吊箱上升状态
};

State_f currentState = state;     // 当前状态，默认为state
unsigned long now;                // 当前时间
unsigned long stateStartTime = 0; // 状态开始时间
unsigned long stateDuration = 0;  // 状态持续时间

char receivedCommand;          // 接收到的命令的头一个字母
long receivedPosition = 0;     // 接收到新位置命令
long receivedD = 0;            // 测试D用的
int receivedDevNum = 0;        // 1表示门2表示隔板
long zPosition = TOTAL_LENGTH; // 基位置。以步数计。在回零后归零。
long doorPosition = DOOR_LENGTH;
int state_3 = STOPPED;
int mode_3 = LINEAR_SPEED;
short down = 0; // 吊箱最低点位置

int state_s1 = 0;      // 状态机名称
int satte_prev_s1 = 0; // 记录前一个状态的变量
int val_s1 = 0;        // 存储按钮开关的状态
short KGdata = 1;
short n_data = 1;
unsigned long t_s1 = 0;            // 定时时间
unsigned long t_0_s1 = 0;          // 记录开始时间
unsigned long bounce_delay_s1 = 5; // 用来确定开关不再抖动的时间
unsigned long lastdata = 0;        // 记录电机上一次运行位置

#define SHELF_ON_PIN Y2
#define SHELF_OFF_PIN Y0
bool shelfOFF = true;    // 置物隔板的状态。true表示搁板打开，off表示隔板关闭
bool lift_data = false;  // 只输出一次隔板碰撞时的位置
bool time_frist = true;  // 只读取一次开始时间
bool time_state = false; // 当读取到字符“b”时，可以在loop函数中每次都运行提交函数
bool time_data = true;   // 当电机要动作时，只发送一次要动作的数据，不需要一直发，会阻塞

void initialize();  // 初始化
void process();     // 提交后的动作
void backToZero();  // 如果出现错误，可以通过此函数回零
void checkSerial(); // 串口接收
void limitZ();      // 内置中断引脚，控制吊箱限位
void limitD();      // 内置中断引脚，控制小门限位
void pinDetect();   // 隔板上接近开关检测函数

void setup()
{
  Serial.begin(115200);

  // 初始化轿厢回零中断函数
  pinMode(Z_PIN, INPUT_PULLUP); // Z_PIN管脚内部弱上拉，这样遇到GND电平就会触发下降沿中断
  attachInterrupt(digitalPinToInterrupt(Z_PIN), limitZ, FALLING);

  // 初始化小门回零中断函数
  pinMode(D_PIN, INPUT_PULLUP); // D_PIN管脚内部弱上拉，这样遇到GND电平就会触发下降沿中断
  attachInterrupt(digitalPinToInterrupt(D_PIN), limitD, FALLING);

  pinMode(SHELF_ON_PIN, OUTPUT);    // 搁板打开管脚
  digitalWrite(SHELF_ON_PIN, LOW);  //
  pinMode(SHELF_OFF_PIN, OUTPUT);   // 搁板关闭管脚
  digitalWrite(SHELF_OFF_PIN, LOW); //

  // 初始化步进电机，说明最高转速，细分数
  stepper1.begin(RPM1, MICROSTEPS1);
  stepper2.begin(RPM2, MICROSTEPS2);
  stepper3.begin(RPM3, MICROSTEPS3);
  stepper3.setSpeedProfile(static_cast<BasicStepperDriver::Mode>(mode_3), 2000, 2000);
  // 使能步进电机驱动器的ENABLE脚，让电机得电。
  //     stepper1.enable();
  //     stepper2.enable();

  Serial.println("Start,Pleae Home it first");

  // set the motor to move continuously for a reasonable time to hit the stopper
  // let's say 100 complete revolutions (arbitrary number)
  // stepper.startMove(100 * MOTOR_STEPS * MICROSTEPS);     // in microsteps
  // stepper.startRotate(100 * 360);                     // or in degrees
}

void loop()
{
  now = millis();

  // put your main code here, to run repeatedly:
  pinDetect();

  // 重点难点，非阻塞地改变管脚电平发步进脉冲，并告知下一个脉冲需要等待多少时间才能到来
  unsigned wait_time_micros1 = stepper1.nextAction();
  unsigned wait_time_micros2 = stepper2.nextAction();
  unsigned wait_time_micros3 = stepper3.nextAction();
  // 下一个脉冲到来的时间<=0 就表示脉冲永远不会到来了，也就是说命令已经完成。
  if (wait_time_micros1 <= 0)
  {
    // stepper1.disable();       // comment out to keep motor powered
    checkSerial();
  }

  // 下一个脉冲到来的时间>100ms这给了我们充裕地时间在电机运行的过程中"并行"处理一些事情例如读写串口
  if (wait_time_micros1 > 100)
  {
    // other code here
    checkSerial();
  }
  else
  {
    // 如果需要等待的时间在(0,100]ms之间那么这就是说很快就要发脉冲了，我们最好什么都不要做。免得干扰人家发脉冲
  }

  if (time_state)
  {
    process();
  }
}

// 串口读取函数
void checkSerial()
{
  if (Serial.available() > 0) // if something comes
  {
    receivedCommand = Serial.read();
    n_data = 1;
    if (receivedCommand == 'a')
    {
      initialize();
    }
    if (receivedCommand == 'b')
    {
      time_state = true;
    }
    if (receivedCommand == 'c')
    {
      backToZero(); // 如果出现错误，可以通过此函数回零
    }
  }
}

// 内置吊箱中断
void limitZ()
{
  stepper1.startMove(0); // 这里是为了stepper1清零内部的step_count计数。见BasicStepperDriver.cpp
  stepper2.startMove(0);
  stepper1.stop();
  stepper2.stop();
  zPosition = TOTAL_LENGTH;
  Serial.println("Z Limited Switch ON");
}

// 内置小门中断
void limitD()
{
  stepper3.startMove(0); // 这里是为了stepper3清零内部的step_count计数。见BasicStepperDriver.cpp
  stepper3.stop();
  doorPosition = DOOR_LENGTH;
  Serial.println("Door Limited Switch ON");
}

// 隔板接近开关检测引脚
void pinDetect()
{
  satte_prev_s1 = state_s1; // 存储上一个状态

  switch (state_s1)
  {
  case 0: // RESET
    state_s1 = 1;
    break;
  case 1: // START
    val_s1 = digitalRead(X3);
    if (val_s1 == LOW)
    {
      state_s1 = 2;
    }
    break;
  case 2: // GO!
    t_0_s1 = millis();
    state_s1 = 3;
    break;
  case 3: // WAIT
    val_s1 = digitalRead(X3);
    t_s1 = millis();

    if (val_s1 == HIGH)
    {
      state_s1 = 0;
    }
    if (t_0_s1 - t_s1 > bounce_delay_s1)
    {
      state_s1 = 5;
    }
    break;
  case 4: // TRIGGERED
    state_s1 = 0;
    break;
  case 5:
    val_s1 = digitalRead(X3);
    if (val_s1 == HIGH)
    {
      state_s1 = 4;
    }
    if (val_s1 == LOW)
    {
      digitalWrite(SHELF_ON_PIN, LOW);
      digitalWrite(SHELF_OFF_PIN, LOW);
      if (n_data > 0)
      {
        n_data--;

        // 得到新的位置命令，要考虑上一个位置命令可能还在执行
        stepper1.stop(); // 先停下来
        stepper2.stop();
        zPosition = zPosition - stepper1.getDirection() * stepper1.getStepsCompleted(); // 计算出基础位置
        //    receivedPosition = //<------------------------;
        // if(receivedPosition>TOTAL_LENGTH) receivedPosition = TOTAL_LENGTH;//如果输入位置超过总长，则按总长算
        stepper1.startMove(-10000); // 根据命令要求位置与当前位置计算需要走的步数
        stepper2.startMove(-10000);

        // 当在提交状态下，隔板中断，需要将所有状态重置
        currentState = state;
        now = 0;            // 当前时间
        stateStartTime = 0; // 状态开始时间
        stateDuration = 0;  // 状态持续时间
        time_frist = true;  // 只读取开始运行时的时间
        // time_state = false;
        receivedCommand = '\0';
      }
      state_s1 = 3;
    }
    break;
  }
}

void backToZero() // 如果出现错误，可以通过此函数回零
{
  time_state = false;
  if (digitalRead(Z_PIN) == HIGH)
  {
    stepper1.startMove(-2 * TOTAL_LENGTH);
    stepper2.startMove(-2 * TOTAL_LENGTH);
  }

  if (digitalRead(D_PIN) == HIGH)
  {
    stepper3.startMove(-2 * DOOR_LENGTH);
  }
  digitalWrite(SHELF_ON_PIN, LOW);   // 如果两个管脚同时设置为LOW那就要炸了。
  digitalWrite(SHELF_OFF_PIN, HIGH); //
}

void initialize()
{
  // 得到新的位置命令，要考虑上一个位置命令可能还在执行
  stepper3.stop();                                                                      // 先停下来
  doorPosition = doorPosition - stepper3.getDirection() * stepper3.getStepsCompleted(); // 计算出基础位置
  receivedD = 0;                                                                        // 相当于 d 0
  stepper3.startMove(-(receivedD - doorPosition));
  time_state = false;
}

void process()
{
  switch (currentState) // 初始化状态
  {
  case state: // 空闲状态
    currentState = theSmallDoorIsClosed;
    break;

  case theSmallDoorIsClosed: // 小门关闭状态

    // 得到新的位置命令，要考虑上一个位置命令可能还在执行
    doorPosition = doorPosition - stepper3.getDirection() * stepper3.getStepsCompleted(); // 计算出基础位置
    if (time_data)
    {
      Serial.println("little door close!");
      // stepper3.stop();                                                                      // 先停下来
      receivedD = DOOR_LENGTH;
      stepper3.startMove(-(receivedD - doorPosition));
      time_data = false;
    }

    if (digitalRead(D_PIN) == LOW) //
    {                              // 等待后进入电机开启状态
      currentState = theGondolaDescends;
      time_data = true;
      Serial.println("diaoxiang down!");
    }
    break;

  case theGondolaDescends: // 吊箱下降

    zPosition = zPosition - stepper1.getDirection() * stepper1.getStepsCompleted(); // 计算出基础位置
    if (time_data)
    {
      // stepper1.stop(); // 先停下来
      // stepper2.stop();
      receivedPosition = 0;
      stepper1.startMove(-(receivedPosition - zPosition)); // 根据命令要求位置与当前位置计算需要走的步数
      stepper2.startMove(-(receivedPosition - zPosition));
      time_data = false;
    }

    if (stepper1.getStepsCompleted() >= 78000)
    { // 等待后进入电机开启状态
      currentState = thePartitionOpens;
      time_data = true;
      Serial.println("diaoxiang down!");
    }
    break;

  case thePartitionOpens: // 隔板打开状态
  {
    if (time_frist)
    {
      stateStartTime = millis();
      time_frist = false;
    }

    digitalWrite(SHELF_OFF_PIN, LOW); //
    digitalWrite(SHELF_ON_PIN, HIGH); // 如果两个管脚同时设置为LOW那就要炸了。

    stateDuration = now - stateStartTime;
    if (stateDuration >= 7000) // 7000
    {
      digitalWrite(SHELF_OFF_PIN, LOW); // 推杆停止工作
      digitalWrite(SHELF_ON_PIN, LOW);  //

      currentState = thePartitionIsClosed;
      time_frist = true;
      Serial.println("geban close!");
    }
    break;
  }

  case thePartitionIsClosed: // 隔板关闭状态
  {
    if (time_frist)
    {
      stateStartTime = millis();
      time_frist = false;
    }
    digitalWrite(SHELF_ON_PIN, LOW); // 如果两个管脚同时设置为LOW那就要炸了。
    digitalWrite(SHELF_OFF_PIN, HIGH);

    stateDuration = now - stateStartTime;
    if (stateDuration >= 7000) // 7000
    {
      digitalWrite(SHELF_OFF_PIN, LOW); //
      digitalWrite(SHELF_ON_PIN, LOW);  //

      currentState = theGondolaRises;
      time_frist = true;
      Serial.println("diaoxiang up!");
    }
    break;
  }

  case theGondolaRises: // 吊箱上升状态
  {
    zPosition = zPosition - stepper1.getDirection() * stepper1.getStepsCompleted(); // 计算出基础位置
    if (time_data)
    {
      stepper1.startMove(-2 * TOTAL_LENGTH);
      stepper2.startMove(-2 * TOTAL_LENGTH);
      time_data = false;
    }

    if (digitalRead(Z_PIN) == LOW) //
    {

      currentState = state;
      now = 0;            // 当前时间
      stateStartTime = 0; // 状态开始时间
      stateDuration = 0;  // 状态持续时间
      receivedCommand = '\0';
      time_frist = true;
      time_state = false;
      time_data = true;
      Serial.println("game over!");
    }
    break;
  }
  }
}