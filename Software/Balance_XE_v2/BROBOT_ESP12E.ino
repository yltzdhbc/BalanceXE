
/*****************************************************************************************
使用说明,因为需要获得脉冲数,ESP8266的库文件中没有相关函数,因此需要修改底层库文件
修改方式: 文件路径 需要注意这些路径在两个系统下都是隐藏的 需要打开显示隐藏文件功能
WINDOWS下: /Users/你的用户名/AppData/Local/Arduino15/packages/esp8266/hardware/esp8266/2.6.3/cores/esp8266
LINUX下: /home/你的用户名/.arduino15/packages/esp8266/hardware/esp8266/2.6.3/cores/esp8266
打开文件夹中的 core_esp8266_waveform.cpp 文件 在 void timer1Interrupt() 函数前加上 外部声明
extern int8_t dir_M1, dir_M2;
extern volatile int32_t steps1, steps2;
在函数里面 if (waveformState & mask) 判断中 加上
else if (i == 12)
{
  GPOS = (1 << 12);
  dir_M1 > 0 ? steps1-- : steps1++;
}
else if (i == 14)
{
  GPOS = (1 << 14);
  dir_M2 > 0 ? steps2-- : steps2++;
}
*****************************************************************************************/

/*********************include********************/
#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>
#include "Ticker.h"
#include "config.h"
#include "Plotter.h"

/*********************引脚定义********************/
//这里也是LED的端口2

#define pinServo1 0
#define pinServo2 2

#define pinMotoPluse1 12
#define pinMotoDir1 13
#define pinMotoPluse2 14
#define pinMotoDir2 15
#define pinMotoEnable 16

/* 0 不打印调试信息
** 1 微分时间，陀螺仪的角度
** 2 滤波后的估计速度
* 
** 3 目标角度，电池信息
** 4 两电机输出值 */
#define DEBUG 0

/*********************实例继承********************/
Servo myservo1;
Servo myservo2;
Ticker TestTicker;                  // 测试定时器
IPAddress local_IP(192, 168, 4, 1); // UDP本机IP
IPAddress gateway(192, 168, 4, 1);  // UDP网关,不设置
IPAddress subnet(255, 255, 255, 0); // UDP子网掩码
uint16_t localUdpPort = 2222;       // UDP本地监听端口
WiFiUDP Udp;

double x;  // global variables
Plotter p; // create plotter

IPAddress sta_staticIP(192, 168, 1, 201); //固定IP地址
IPAddress sta_gateway(192, 168, 1, 1);    //网关地址
IPAddress sta_subnet(255, 255, 255, 0);   //子网掩码地址

void setup()
{
  /*********************初始化IO********************/
  pinMode(LED_BUILTIN, OUTPUT); // ESP8266 板载 LED
  pinMode(pinServo1, OUTPUT);        // GPIO0 (NODEMCU-D3) 舵机1PWM
  pinMode(pinServo2, OUTPUT);        // GPIO2 (NODEMCU-D4) 舵机2PWM
  pinMode(pinMotoPluse1, OUTPUT);    // GPIO12 (NODEMCU-D6) A4988 1 脉冲
  pinMode(pinMotoDir1, OUTPUT);      // GPIO13 (NODEMCU-D7) A4988 1 方向
  pinMode(pinMotoPluse2, OUTPUT);    // GPIO14 (NODEMCU-D5) A4988 2 脉冲
  pinMode(pinMotoDir2, OUTPUT);      // GPIO15 (NODEMCU-D8) A4988 2 方向
  pinMode(pinMotoEnable, OUTPUT);    // GPIO16 (NODEMCU-D0) A4988   使能
  digitalWrite(pinMotoEnable, HIGH); // 失能电机
  digitalWrite(LED_BUILTIN, LOW);    // 关LED

  /*********************通信初始化********************/
  Serial.begin(115200); // 调试串口
  Wire.begin();       // Wire 使用的引脚GPIO4 (NODEMCU-D2) SDA | GPIO5 (NODEMCU-D1) SCL
  OSC_init();         // OSC协议初始化

  // p.Begin();                                                       // start plotter
  // p.AddTimeGraph("Some title of a graph", 1500, "label for x", x); // add any graphs you want

  /*********************MPU6050初始化********************/
  Serial.println("B-BOT");
  delay(200);
  Serial.println("Don't move for 10 sec...");
  MPU6050_setup(); // 初始化MPU6050
  delay(500);
  MPU6050_calibrate(); // 校准MPU6050

  /*********************WIFI初始化********************/

#ifdef EXTERNAL_WIFI
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.config(sta_staticIP, sta_gateway, sta_subnet);
  Udp.begin(localUdpPort); // 启动Udp监听服务
// Serial.printf("UDP server on port %d\n", localUdpPort);
#else
  Serial.println("WIFI init");
  WiFi.softAPmacAddress(macAddr);               // 获取本机的MAC地址
  Serial.printf("macAddr %d\n", macAddr);
  WiFi.softAPConfig(local_IP, gateway, subnet); // 配置AP信息
  char *cmd = "ARYA_XXXX";                        // XX替换成MAC地址,用于区分不同的机器人
  sprintf(macAddrChar, "%d", macAddr);
  cmd[5] = macAddrChar[0];
  cmd[6] = macAddrChar[1];
  cmd[7] = macAddrChar[2];
  cmd[8] = macAddrChar[3];
  WiFi.softAP(cmd, "88888888"); // 启动AP模式，并设置账号和密码
  Udp.begin(localUdpPort);      // 启动Udp监听服务
  Serial.printf("UDP server on port %d\n", localUdpPort);
#endif
  /*********************测试定时器初始化********************/
  // TestTicker.attach_ms(1000, testFunc);

  /*********************舵机初始化********************/
  Serial.println("Servo init");
  myservo1.attach(0);
  myservo1.write(SERVO_AUX_NEUTRO);
  myservo2.attach(2);
  myservo2.write(SERVO_AUX_NEUTRO);

  /*********************执行初始启动动作********************/
  Serial.println("Stepers init");
  for (uint8_t k = 0; k < 5; k++)
  {
    ROBOT_moveServo1(SERVO_AUX_NEUTRO + 30);
    ROBOT_moveServo2(SERVO2_NEUTRO + 30);
    delay(200);
    ROBOT_moveServo1(SERVO_AUX_NEUTRO - 30);
    ROBOT_moveServo2(SERVO2_NEUTRO - 30);
    delay(200);
  }
  ROBOT_moveServo1(SERVO_AUX_NEUTRO);
  ROBOT_moveServo2(SERVO2_NEUTRO);

  Serial.println("Start...");
  //digitalWrite(LED_BUILTIN, LOW); // 打开 LED 指示

  timer_old = micros();
}

uint16_t ledCounter = 0;
uint16_t udpCounter = 0;
void loop()
{
  if (udpCounter == 0)
  {
    packetSize = Udp.parsePacket(); // 读取一帧 OSC 消息
    udpCounter = packetSize;        // 将数据包的大小传入udpCounter
  }
  else
  {
    OSC_MsgRead(); // 解析一位数据
    OSC_Parse();   // OSC数据转换，得到每个通道的数值
    udpCounter--;
  }

  timer_value = micros(); // 更新时间

  if (MPU6050_newData()) // IMU是否准备好
  {
    MPU6050_read_3axis();
    loop_counter++;
    slow_loop_counter++;
    dt = (timer_value - timer_old) * 0.000001; // dt (s)
    timer_old = timer_value;

    angle_adjusted_Old = angle_adjusted;
    // 更新角度数据 (MPU6050)
    float MPU_sensor_angle = MPU6050_getAngle(dt);
    angle_adjusted = MPU_sensor_angle + angle_offset;
    if ((MPU_sensor_angle > -15) && (MPU_sensor_angle < 15))
      angle_adjusted_filtered = angle_adjusted_filtered * 0.99 + MPU_sensor_angle * 0.01;

#if DEBUG == 1
    Serial.print(dt);
    Serial.print(" ");
    Serial.print(angle_offset);
    Serial.print(" ");
    Serial.print(angle_adjusted);
    Serial.print(",");
    Serial.println(angle_adjusted_filtered);
#endif

    // 因为步进电机是开环系统在这里我们估算机器人的速度
    // 估算的机器人速度=电机反馈速度 -imu计算出的机器人角速度
    actual_robot_speed = (speed_M1 + speed_M2) / 2; // 由机器人电机反馈计算的机器人速度

    //角速度=（当前角度-上一个周期的角度）/时间 大概是40ms
    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0;
    int16_t estimated_speed = -actual_robot_speed + angular_velocity;
    estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float)estimated_speed * 0.1; // 低通滤波

#if DEBUG == 2
    Serial.print(angle_adjusted);
    Serial.print(" ");
    Serial.println(estimated_speed_filtered);
#endif

    if (positionControlMode) //位置控制模式
    {
      // 电机脉冲控制器. input: 每个电机的目标脉冲数. output: 电机速度
      motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
      motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

      // 将电机脉冲控制的输出量转换为油门和转向数据
      throttle = (motor1_control + motor2_control) / 2;
      throttle = constrain(throttle, -190, 190);
      steering = motor2_control - motor1_control;
      steering = constrain(steering, -50, 50);
    }

    // 机器人速度控制器 PI
    // input:设定的速度数据(油门,转向), variable: 估算的反馈速度, output: 目标机器人倾角(改变倾角获得期望速度)
    target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
    target_angle = constrain(target_angle, -max_target_angle, max_target_angle);

#if DEBUG == 3
    Serial.print(angle_adjusted);
    Serial.print(" ");
    Serial.print(estimated_speed_filtered);
    Serial.print(" ");
    Serial.println(target_angle);
    Serial.print(" ");
    Serial.println(BatteryValue);
#endif

    // 机器人稳定控制器 (100Hz loop) PD
    // input: 机器人的目标角度(由速度控制器传入), variable: 机器人反馈回来的真实角度, output: 电机速度
    // 这里由于对量做了一个积分，实际的输出为加速度而不是速度
    control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
    control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

    motor1 = control_output + steering;
    motor2 = control_output - steering;

#if DEBUG == 4
    Serial.print(motor1);
    Serial.print(" ");
    Serial.println(motor2);
#endif

    // 限制最大输出
    motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
    motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

    int angle_ready;
    if (OSCpush[0]) // 舵机控制部分
      angle_ready = 82;
    else
      angle_ready = 74;

    // 机器人是否在站立的角度范围内
    if ((angle_adjusted < angle_ready) && (angle_adjusted > -angle_ready))
    {
      digitalWrite(pinMotoEnable, LOW); // 电机使能
      setMotorSpeedM1(motor1);
      setMotorSpeedM2(motor2);
    }
    else //角度过大 判定机器人为倒地状态 关闭电机
    {
      digitalWrite(pinMotoEnable, HIGH); // 电机失能
      setMotorSpeedM1(0);
      setMotorSpeedM2(0);
      PID_errorSum = 0; // Reset PID I term
      Kp = KP_RAISEUP;  // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
      steps1 = 0;
      steps2 = 0;
      positionControlMode = false;
      OSCmove_mode = false;
      throttle = 0;
      steering = 0;
    }

    // 控制舵机机械手
    if (OSCpush[0])
    {
      if (angle_adjusted > -40)
        ROBOT_moveServo1(SERVO_MIN_PULSEWIDTH);
      else
        ROBOT_moveServo1(SERVO_MAX_PULSEWIDTH);
    }
    else
      ROBOT_moveServo1(SERVO_AUX_NEUTRO);

    // 舵机2
    ROBOT_moveServo2(SERVO2_NEUTRO + (OSCfader[2] - 0.5) * SERVO2_RANGE);

    // 增益调整 判断是否需要进行站起 赋予不同的参数
    if ((angle_adjusted < 56) && (angle_adjusted > -56))
    {
      Kp = Kp_user;
      Kd = Kd_user;
      Kp_thr = Kp_thr_user;
      Ki_thr = Ki_thr_user;
    }
    else // 赋予控制器不同的参数 进入站起程序
    {
      Kp = KP_RAISEUP;
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
    }

  } // End IMU

  /******Medium loop 7.5Hz******/
  if (loop_counter >= 50)
  {
    loop_counter = 0;
    // Telemetry here?
#if TELEMETRY_ANGLE == 1
    char auxS[25];
    int ang_out = constrain(int(angle_adjusted * 10), -900, 900);
    sprintf(auxS, "$tA,%+04d", ang_out);

    //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    Udp.beginPacket("192.168.4.100", 2223);
    Udp.write(auxS);
    Udp.endPacket();
#endif
#if TELEMETRY_DEBUG == 1
    char auxS[50];
    sprintf(auxS, "$tD,%d,%d,%ld", int(angle_adjusted * 10), int(estimated_speed_filtered), steps1);
    Serial1.println(auxS);
#endif
  }

  /*********slow loop 1Hz*********/
  if (slow_loop_counter >= 50)
  {
    slow_loop_counter = 0;
#if TELEMETRY_BATTERY == 1
    BatteryValue = (BatteryValue + ROBOT_readBattery(false)) / 2;
    sendBattery_counter++;
    if (sendBattery_counter >= 1)
    { //Every 3 seconds we send a message
      sendBattery_counter = 0;
      // Serial.print("B");
      // Serial.println(BatteryValue);
      char auxS[25];
      sprintf(auxS, "$tB,%04d", BatteryValue);
      Serial.println(auxS);
      Udp.beginPacket("192.168.4.2", 2223);
      // Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(auxS);
      Udp.endPacket();
    }
#endif
  } // End of slow loop
}

void testFunc()
{

  (ledCounter == 0) ? (1) : (0);

  if (ledCounter == 0)
    digitalWrite(2, LOW);
  else
    digitalWrite(2, HIGH);
}
