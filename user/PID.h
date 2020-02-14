#ifndef PID_H_
#define PID_H_

#define MAXOUT 7000
#define PIDCTRL

#include "chlib_k.h"

typedef struct                                   //PID调节结构体
{
	float Kp, Ki, Kd;
  float Cur_Error, Pre_Error, Sum_Error;

}PID_type;

typedef struct
{
  int16_t Realspeed;
  int16_t Tarspeed;
  int16_t Output;
  PID_type PID;
}PidCtrl_type;

extern PidCtrl_type Left_Motor,Right_Motor,Speed_w;

void PID_Init(void);
void PID_Ctrl(PidCtrl_type* Motor);
#endif /* PID_H_ */
