#include <Arduino.h>

#include <Arduino_FreeRTOS.h>
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

A4988 L1_motor(MOTOR_STEPS, L1_motor_STRP, L1_motor_DIR, MS1_PIN, MS2_PIN, MS3_PIN);
A4988 R1_motor(MOTOR_STEPS, R1_motor_STRP, R1_motor_DIR, MS1_PIN, MS2_PIN, MS3_PIN);
A4988 L2_motor(MOTOR_STEPS, L2_motor_STRP, L2_motor_DIR, MS1_PIN, MS2_PIN, MS3_PIN);
A4988 R2_motor(MOTOR_STEPS, R2_motor_STRP, R2_motor_DIR, MS1_PIN, MS2_PIN, MS3_PIN);

//初始化传感器(漫反射参数)
long diffuse_reflection_Position = 0;

//加减速曲线规划
int motor_regulation_status = LINEAR_SPEED;
short motor_accelerate = 2000;
short motor_slow_down = 2000;

// 在启用蓝牙时，数据储存的变量
char BT_SerialData[64];
volatile double result = 0;

long receivedPosition = 0; // 接收到新位置命令

// 调试用位置变量
long debug_receivedPosition = 10000; // 移动五千步

// 存储运动的脉冲步数
struct motor_Position_status
{
  long first_forward = 50000;
  long first_lift = 50000;
  long second_forward = 50000;
  long sercond_lift = 50000;
  long third_forward = 50000;
  long third_lift = 50000;
  long fourth_forward = 50000;
  long fourth_lift = 50000;
  long fifth_forward = 50000;
  long fifth_lift = 50000;
};

// 函数初始化
void motor_move_Task(void *pvParameters);
void BTserver_Task(void *pvParameters);

void setup()
{
  Serial.begin(115200);

  xTaskCreate(motor_move_Task, "电机运动进程", 1000, NULL, 1, NULL);
  xTaskCreate(BTserver_Task, "通过蓝牙连接Arduino,实现对小车运动状态的更改,以及对PID参数的更改", 1000, NULL, 2, NULL);

  // 设置 A4988 驱动器的微步数和转速
  L1_motor.begin(RPM, MICROSTEPS); // 60 RPM，16 微步
  R1_motor.begin(RPM, MICROSTEPS);
  L2_motor.begin(RPM, MICROSTEPS);
  R2_motor.begin(RPM, MICROSTEPS);
  L1_motor.setSpeedProfile(static_cast<BasicStepperDriver::Mode>(motor_regulation_status), motor_accelerate, motor_slow_down); // 配置加减速曲线
  R1_motor.setSpeedProfile(static_cast<BasicStepperDriver::Mode>(motor_regulation_status), motor_accelerate, motor_slow_down);
  L2_motor.setSpeedProfile(static_cast<BasicStepperDriver::Mode>(motor_regulation_status), motor_accelerate, motor_slow_down);
  R2_motor.setSpeedProfile(static_cast<BasicStepperDriver::Mode>(motor_regulation_status), motor_accelerate, motor_slow_down);
}

void loop()
{
}

void motor_move_Task(void *pvParameters)
{
  while (1)
  {
    // // 重点难点，非阻塞地改变管脚电平发步进脉冲，并告知下一个脉冲需要等待多少时间才能到来
    // unsigned L1_wait_time_micros = L1_motor.nextAction();
    // unsigned R2_wait_time_micros = R1_motor.nextAction();
    // unsigned L2_wait_time_micros = L2_motor.nextAction();
    // unsigned R2_wait_time_micros = R2_motor.nextAction();

    // // 下一个脉冲到来的时间<=0 就表示脉冲永远不会到来了，也就是说命令已经完成。
    // if (L1_wait_time_micros <= 0)
    // {
    //   // stepper1.disable();       // comment out to keep motor powered
    //   checkSerial();
    // }
    // 下一个脉冲到来的时间>100ms这给了我们充裕地时间在电机运行的过程中"并行"处理一些事情例如读写串口
    // if (L1_wait_time_micros > 100)
    // {
    //   // other code here
    //   checkSerial();
    // }
    // else
    // {
    //   // 如果需要等待的时间在(0,100]ms之间那么这就是说很快就要发脉冲了，我们最好什么都不要做。免得干扰人家发脉冲
    // }

    L1_motor.startMove(debug_receivedPosition);
    R1_motor.startMove(debug_receivedPosition);
    L2_motor.startMove(debug_receivedPosition);
    R2_motor.startMove(debug_receivedPosition);
    vTaskDelay(pdMS_TO_TICKS(200));
    L1_motor.startMove(-(debug_receivedPosition));
    R1_motor.startMove(debug_receivedPosition);
    L2_motor.startMove(-(debug_receivedPosition));
    R2_motor.startMove(debug_receivedPosition);
    vTaskDelay(pdMS_TO_TICKS(200));
    L1_motor.startMove(debug_receivedPosition);
    R1_motor.startMove(debug_receivedPosition);
    L2_motor.startMove(debug_receivedPosition);
    R2_motor.startMove(debug_receivedPosition);
    vTaskDelay(pdMS_TO_TICKS(200));
    L1_motor.startMove(-(debug_receivedPosition));
    R1_motor.startMove(debug_receivedPosition);
    L2_motor.startMove(-(debug_receivedPosition));
    R2_motor.startMove(debug_receivedPosition);
    vTaskDelay(pdMS_TO_TICKS(200));
    L1_motor.startMove(debug_receivedPosition);
    R1_motor.startMove(debug_receivedPosition);
    L2_motor.startMove(debug_receivedPosition);
    R2_motor.startMove(debug_receivedPosition);
    vTaskDelay(pdMS_TO_TICKS(200));
    L1_motor.startMove(-(debug_receivedPosition));
    R1_motor.startMove(debug_receivedPosition);
    L2_motor.startMove(-(debug_receivedPosition));
    R2_motor.startMove(debug_receivedPosition);
    vTaskDelay(pdMS_TO_TICKS(200));
    L1_motor.startMove(debug_receivedPosition);
    R1_motor.startMove(debug_receivedPosition);
    L2_motor.startMove(debug_receivedPosition);
    R2_motor.startMove(debug_receivedPosition);
    vTaskDelay(pdMS_TO_TICKS(200));
    L1_motor.startMove(-(debug_receivedPosition));
    R1_motor.startMove(debug_receivedPosition);
    L2_motor.startMove(-(debug_receivedPosition));
    R2_motor.startMove(debug_receivedPosition);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void BTserver_Task(void *pvParameters)
{
  while (1)
  {
    // 将用户通过串口监视器输入的数据发送给HC-06
    if (Serial2.available() > 0)
    { // 如果硬件串口缓存中有等待传输的数据
      // 将传入数据读取到字符数组中
      int i = 0;
      // 清空BT_data数组
      memset(BT_SerialData, 0, sizeof(BT_SerialData));

      // 清空串口缓冲区
      while (Serial2.available())
      {
        char c = Serial2.read();
        BT_SerialData[i] = c;
        i++;
      }

      // 查找第一个数字的位置
      char *ptr = BT_SerialData;
      while (*ptr && !isdigit(*ptr))
      {
        ptr++;
      }

      // 将字符串的数字部分转换为浮点数
      result = atof(ptr);

      // 打印接收到的字符串
      Serial2.print("Received data: ");
      Serial2.println(BT_SerialData);

      // 输出转换后的浮点数
      Serial2.print("转换后的浮点数: ");
      Serial2.println(result, 2); // 保留2位小数

      BTserver_move(BT_SerialData[0], result);
    }

    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

void BTserver_move(char data, double num)
{
  vTaskDelay(pdMS_TO_TICKS(100));
  switch (data)
  {
  case 'w':
    Serial2.println("前进"); // 编号:w
    allstop_Status = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    L1_motor.startMove(debug_receivedPosition);
    R1_motor.startMove(debug_receivedPosition);
    L2_motor.startMove(debug_receivedPosition);
    R2_motor.startMove(debug_receivedPosition);
    break;

  case 'x':
    Serial2.println("后退"); // 编号:x
    allstop_Status = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    L1_motor.startMove(-(debug_receivedPosition));
    R1_motor.startMove(-(debug_receivedPosition));
    L2_motor.startMove(-(debug_receivedPosition));
    R2_motor.startMove(-(debug_receivedPosition));
    break;

  case 'a':
    Serial2.println("左边平移");
    allstop_Status = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    L1_motor.startMove(-(debug_receivedPosition));
    R1_motor.startMove(debug_receivedPosition);
    L2_motor.startMove(debug_receivedPosition);
    R2_motor.startMove(-(debug_receivedPosition));
    break;

  case 'f':
    Serial2.println("右边平移");
    allstop_Status = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    L1_motor.startMove(debug_receivedPosition);
    R1_motor.startMove(-(debug_receivedPosition));
    L2_motor.startMove(-(debug_receivedPosition));
    R2_motor.startMove(debug_receivedPosition);
    break;

  case 'q':
    Serial2.println("斜向左上方");
    allstop_Status = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    R1_motor.startMove(debug_receivedPosition);
    L2_motor.startMove(debug_receivedPosition);
    break;

  case 'e':
    Serial2.println("斜向右上方");
    allstop_Status = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    L1_motor.startMove(debug_receivedPosition);
    R2_motor.startMove(debug_receivedPosition);
    break;

  case 'z':
    Serial2.println("斜向左下方");
    allstop_Status = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    L1_motor.startMove(-(debug_receivedPosition));
    R2_motor.startMove(-(debug_receivedPosition));
    break;

  case 'c':
    Serial2.println("斜向右下方");
    allstop_Status = false;
    vTaskDelay(pdMS_TO_TICKS(200));
    R1_motor.startMove(-(debug_receivedPosition));
    L2_motor.startMove(-(debug_receivedPosition));
    break;

  case 'v':
    Serial2.println("顺时针原地旋转");
    allstop_Status = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    L1_motor.startMove(debug_receivedPosition);
    R1_motor.startMove(-(debug_receivedPosition));
    L2_motor.startMove(debug_receivedPosition);
    R2_motor.startMove(-(debug_receivedPosition));
    break;

  case 'b':
    Serial2.println("逆时针原地旋转");
    allstop_Status = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    L1_motor.startMove(-(debug_receivedPosition));
    R1_motor.startMove(debug_receivedPosition);
    L2_motor.startMove(-(debug_receivedPosition));
    R2_motor.startMove(debug_receivedPosition);
    break;

  default:
    break;
  }

  switch (data)
  {
  case 'r':
    BT_move_Status = true;
    Serial2.println("自动模式");
    break;
  case 't':
    BT_move_Status = false;
    Serial2.println("蓝牙调试模式");
    break;
  case 's':
    BT_move_Status = false;
    Serial2.println("电机停止");
    break;
  case 'p':
    PID_data.kp = num;
    Serial2.print("P参数更改为：");
    Serial2.println(PID_data.kp);
    break;
  case 'i':
    PID_data.ki = num;
    Serial2.print("i参数更改为：");
    Serial2.println(PID_data.kp);
    break;

  case 'd':
    PID_data.kd = num;
    Serial2.print("d参数更改为：");
    Serial2.println(PID_data.kp);
    break;
  default:
    break;
  }
}
