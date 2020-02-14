#include "PID.h"

PidCtrl_type Left_Motor,Right_Motor,Speed_w;

void PID_Init(void)
{
  Left_Motor.PID.Kp = 100;
  Left_Motor.PID.Ki = 0.2;
  Left_Motor.PID.Kd = 0;
 
  Left_Motor.Output = 0;
  Left_Motor.Realspeed = 0;
  Left_Motor.Tarspeed = 0;
  
  Right_Motor.PID.Kp = 100;
  Right_Motor.PID.Ki = 0.2;
  Right_Motor.PID.Kd = 0;
 
  Right_Motor.Output = 0;
  Right_Motor.Realspeed = 0;
  Right_Motor.Tarspeed = 0;
  
  Speed_w.PID.Kp = 2.9;//50;
  Speed_w.PID.Ki = 0;
  Speed_w.PID.Kd = 8;//20;
 
  Speed_w.Output = 0;
  Speed_w.Realspeed = 0;
  Speed_w.Tarspeed = 0;
}

void PID_Ctrl(PidCtrl_type* Ctrl)
{
  Ctrl->PID.Pre_Error = Ctrl->PID.Cur_Error;
  Ctrl->PID.Cur_Error = Ctrl->Tarspeed - Ctrl->Realspeed;

  if(Ctrl != &Speed_w) // 旋转速度不加积分
  {
 //   if(Ctrl->PID.Cur_Error > 3 || Ctrl->PID.Cur_Error < -3)
      Ctrl->PID.Sum_Error += Ctrl->PID.Cur_Error;
 //   else
 //     Ctrl->PID.Sum_Error = 0;
    //积分上限
    Ctrl->PID.Sum_Error = Ctrl->PID.Sum_Error > 15000 ? 15000 : Ctrl->PID.Sum_Error;
    Ctrl->PID.Sum_Error = Ctrl->PID.Sum_Error < -15000 ? -15000 : Ctrl->PID.Sum_Error;
  }
  else // 若旋转速度计算 则积分系数清零
  {
    Ctrl->PID.Ki = 0;
  }

  int32_t temp = (int32_t)(Ctrl->PID.Kp * Ctrl->PID.Cur_Error + Ctrl->PID.Ki * Ctrl->PID.Sum_Error + Ctrl->PID.Kd * (Ctrl->PID.Cur_Error - Ctrl->PID.Pre_Error));
  //限制输出电流
  Ctrl->Output = (temp > MAXOUT) ? MAXOUT : (int16_t)temp;
  Ctrl->Output = (temp < -MAXOUT) ? -MAXOUT : (int16_t)temp;
}

