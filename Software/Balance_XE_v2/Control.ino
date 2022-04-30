
// 平衡PD控制器(比例，微分). DT (s)
float stabilityPDControl(float DT, float input, float setPoint, float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  // The biggest one using only the input (sensor) part not the SetPoint input-input(t-1).
  // And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  float Kd_setPoint = constrain((setPoint - setPointOld), -8, 8); // We limit the input part...
  output = Kp * error + (Kd * Kd_setPoint - Kd * (input - PID_errorOld)) / DT;
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  //PID_errorOld2 = PID_errorOld;
  PID_errorOld = input; // error for Kd is only the input component
  setPointOld = setPoint;
  return (output);
}

// 速度环PI控制器(比例，积分). DT (s)
float speedPIControl(float DT, int16_t input, int16_t setPoint, float Kp, float Ki)
{
  int16_t error;
  float output;

  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  //Serial.println(PID_errorSum);

  output = Kp * error + Ki * PID_errorSum * DT;
  return (output);
}

// 位置环PD控制器(比例，微分)
float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM)
{
  float output;
  float P;

  P = constrain(Kpp * float(setPointPos - actualPos), -10000, 10000);//-115,115
  output = P + Kdp * float(speedM);
  return (output);
}

void setMotorSpeedM1(int16_t tspeed)
{
  int16_t speed;

  if ((speed_M1 - tspeed) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed;

#if MICROSTEPPING == 16
  speed = speed_M1 * 50; //0-500 转换到 0-25000 0-40,000
#else
  speed = speed_M1 * 25;
#endif

  if (speed == 0)
  {
    Pluse_Stop(12);
    dir_M1 = 0;
  }
  else if (speed > 0)
  {
    Pluse_Generate(12, speed); //脉冲频率范围 100 - 25000
    dir_M1 = 1;
    GPOS = (1 << 13); //寄存器操作 SET GPIO13
  }
  else
  {
    Pluse_Generate(12, -speed); //脉冲频率范围 100 - 25000
    dir_M1 = -1;
    GPOC = (1 << 13); //寄存器操作 CLEAR GPIO13
  }
}

void setMotorSpeedM2(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  if ((speed_M2 - tspeed) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed;

#if MICROSTEPPING == 16
  speed = speed_M2 * 50;
#else
  speed = speed_M2 * 25;
#endif

  if (speed == 0)
  {
    Pluse_Stop(14);
    dir_M2 = 0;
  }
  else if (speed > 0)
  {
    Pluse_Generate(14, speed); //脉冲频率范围 100 - 25000
    dir_M2 = 1;
    GPOC = (1 << 15); //寄存器操作 SET GPIO15 (前)
  }
  else
  {
    Pluse_Generate(14, -speed); //脉冲频率范围 100 - 25000
    dir_M2 = -1;
    GPOS = (1 << 15); //寄存器操作 CLEAR GPIO13 (后)
  }
}

// int step_old = 0;
// int new_frequency = 100;
// void motoCtrl1()
// {
//   if (new_frequency > 40000)
//     new_frequency = 100;

//   Pluse_Generate(12, new_frequency);

//   Pluse_Generate(14, new_frequency);

//   Serial.print("steps1:");
//   Serial.print(steps1);
//   Serial.print(",");

//   Serial.print("steps2:");
//   Serial.print(steps2);
//   Serial.print(",");

//   Serial.print("diff:");
//   Serial.print(steps1 - step_old);
//   step_old = steps1;
//   Serial.print(",");

//   Serial.print("frequency:");
//   Serial.println(new_frequency);

//   new_frequency += 100;
// }