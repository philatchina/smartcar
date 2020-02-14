
//速度
/*设想能否通过人工智能适应轨道
*/
#include "math.h"
#include "chlib_k.h"
#include "varieble.h"
#include "img_processing.h"
#include "oled.h"
#include "init.h"
#include "ftm.h"
#include "adc.h"
#include "gpio.h"
#include "common.h"
#include "uart.h"
#include "gui.h"
#include "flash.h"
#include "upper_monitor.h"
#include "PID.h"
   
int iscir=0;

void PIT0_CallBack(void);
void Control(void);
void PWM_Ctrl(void);
void EnterCircle();


uint8_t L_dir,R_dir;

int16_t counter_left = 0;
uint32_t counter_right = 0;

int16_t left_speed = 0;
int16_t right_speed = 0;
int16_t maxspeed = 85;
int16_t minspeed = 65;
int16_t base_speed = 80;

uint8_t L_flag = 0,R_flag = 0;
int Enterflag=0;
//int Changeflag= 0, Blankflag= 0;     //changeflag1为识别拐角标志，chageflag2为识别圆环中间黑块标志
//转弯主从轮速度改变系数 （主轮是外侧）
float master_c = 0.4,slaver_c = 0.6;
bool keychange = 1;
int menu_t = 1;
uint8_t pointx,pointy;

bool stopflag = 0;
int stopcounter = 0;

 int main()
{
	/*
	 * 各个初始化模块请自己核对引脚
	 * 应该在init.c中
	 * 各个库可以自行阅读一下，写好的菜单在gui.c中。可以进行调用 前提是需购买OLED
	 * 上位机的使用在upper_monitor.c中，用的是山外的上位机
	 */


	DisableInterrupts;
	init();       //完成对各模块引脚的初始化（串口，摄像头，OLED，SCCB引脚）
          //PWM初始化
        FTM_PWM_QuickInit(FTM0_CH4_PD04,kPWM_EdgeAligned,1000,0);
        FTM_PWM_QuickInit(FTM0_CH5_PD05,kPWM_EdgeAligned,1000,0);
        FTM_PWM_QuickInit(FTM0_CH6_PD06,kPWM_EdgeAligned,1000,0);
        FTM_PWM_QuickInit(FTM0_CH7_PD07,kPWM_EdgeAligned,1000,0);
             //编码器初始化
        
        FTM_QD_QuickInit(FTM1_QD_PHA_PB00_PHB_PB01,kFTM_QD_InvertedPolarity,kQD_CountDirectionEncoding);
        FTM_QD_ClearCount(HW_FTM1);
        
        LPTMR_PC_QuickInit(LPTMR_ALT2_PC05);
        LPTMR_ClearCounter();
        
        GPIO_QuickInit(HW_GPIOC,4,kGPIO_Mode_IFT);
        //占空比 = pwmDuty/10000*100%
        //理论中值占空比=1.5/20=7.5%,实际需根据机械结构和舵机本身进行调整
        /*
        //舵机初始化
        FTM_PWM_QuickInit(FTM2_CH0_PB18,kPWM_EdgeAligned,50,800);
        FTM_PWM_InvertPolarity(HW_FTM2,HW_FTM_CH0,kFTM_PWM_LowTrue);
        */
        //IO初始化
        /*
        UART_QuickInit(UART3_RX_PC16_TX_PC17, 115200);      //串口
        UART_CallbackRxInstall(HW_UART3,UART3_recv);
        UART_ITDMAConfig(HW_UART3, kUART_IT_Rx, true);
        */
        
        GPIO_QuickInit(HW_GPIOB, 20, kGPIO_Mode_IPD);//1
        GPIO_QuickInit(HW_GPIOB, 21, kGPIO_Mode_IPD);//2
        GPIO_QuickInit(HW_GPIOB, 22, kGPIO_Mode_IPD);//3
        GPIO_QuickInit(HW_GPIOB, 23, kGPIO_Mode_IPD);//4
        GPIO_QuickInit(HW_GPIOB, 11, kGPIO_Mode_OPP);//led1
        GPIO_QuickInit(HW_GPIOB, 17, kGPIO_Mode_OPP);//led2
        GPIO_QuickInit(HW_GPIOB, 9, kGPIO_Mode_OPP);//BEEP
       //计算速度中断
        PIT_QuickInit(HW_PIT_CH0, 5000);//开启PIT中断 0.005s
        PIT_CallbackInstall(HW_PIT_CH0, PIT0_CallBack);//注册回调函数
        PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,DISABLE);
        PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF,ENABLE);//使能PIT中断
      
        EnableInterrupts;//使能中断

        PID_Init();
 
        while(1)
        {
          searchline_OV7620();
         if(GPIO_ReadBit(HW_GPIOB,21))
          {
            OLED_Clear();
            OLED_ShowString_1206(2,2,"WhiteRoad:",1);
            OLED_ShowNum_1206(60,2,whiteRoad,1);
            OLED_ShowString_1206(2,12,"Maxspeed:",1);
            OLED_ShowNum_1206(60,12,maxspeed,1);
            OLED_ShowString_1206(2,22,"Minspeed:",1);
            OLED_ShowNum_1206(60,22,minspeed,1);
            
            int key = keyCheck();
            switch(key)
            {
            case 1:
              menu_t--;
              while(menu_t <= 0)
                menu_t+=3;
              break;
            case 2:
              menu_t++;
              while(menu_t > 3)
                menu_t-=3;
              break;
            case 3:
              if(menu_t == 1)
              {
                whiteRoad-=5;
              }
              else if(menu_t == 2)
              {
                maxspeed--;
               }
              else if(menu_t == 3)
              {
                minspeed--;
              }
              break;
            case 4:
              if(menu_t == 1)
              {
                whiteRoad+=5;
              }
              else if(menu_t == 2)
              {
                maxspeed++;
               }
              else if(menu_t == 3)
              {
                minspeed++;
              }
              break;
            case 5:
              break;
              
            }
            switch(menu_t)
            {
            case 1:
                OLED_ShowString_1206(2,80,"***",1);
                break;
            case 2:
                OLED_ShowString_1206(12,80,"***",1);
                break;
            case 3:
                OLED_ShowString_1206(22,80,"***",1);
                break;
            }
            OLED_Refresh_Gram();
          }
          else
            dispimage1();
          //获取中线位置

          GetOffset();
           
          IsRowAngle();
          Control();
          if(stopflag)
          {
            FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH4,0);
            FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH5,0);
            FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH6,0);
            FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH7,0);
          }
        }

}

void PIT0_CallBack(void)
{
  FTM_QD_GetData(HW_FTM1, &counter_left, &L_dir);//左编码器ab相正交编码
  
  //速度计算
  left_speed = counter_left / 5;
  
  FTM_QD_ClearCount(HW_FTM1);//清零
  
  
  counter_right = LPTMR_PC_ReadCounter();//右编码器lptmr脉冲计数
  
  R_dir = GPIO_ReadBit(HW_GPIOC,4);
  if(R_dir)
    right_speed = counter_right / 5;
  else
    right_speed = -(counter_right / 5);
  LPTMR_ClearCounter();//清零

}

//offset左正
void Control(void)
{
  
  //偏转速度计算
  Speed_w.Tarspeed = Offset;
  Speed_w.Realspeed = 0;
  PID_Ctrl(&Speed_w);
  //左右轮目标速度改变
  
  if(Enterflag == 0)
  {
    if(Speed_w.Output > 0) //左转
    {
      if(Speed_w.Output < 40)
      {
        base_speed += 4;
        if(base_speed > maxspeed)
          base_speed = maxspeed;
        Left_Motor.Tarspeed = base_speed - slaver_c * Speed_w.Output;
        Right_Motor.Tarspeed = base_speed + master_c * Speed_w.Output;
      }
      else
      {
        base_speed = minspeed;
        Left_Motor.Tarspeed = base_speed - slaver_c * Speed_w.Output;
        Right_Motor.Tarspeed = base_speed + master_c * Speed_w.Output;
      }
    }
    else //右转
    {
      if(Speed_w.Output > -40)
      {
        base_speed += 4;
        if(base_speed > maxspeed)
          base_speed = maxspeed;
        Left_Motor.Tarspeed = base_speed - master_c * Speed_w.Output;
        Right_Motor.Tarspeed = base_speed + slaver_c * Speed_w.Output;
      }
      else
      {
        base_speed = minspeed;
        Left_Motor.Tarspeed = base_speed - master_c * Speed_w.Output;
        Right_Motor.Tarspeed = base_speed + slaver_c * Speed_w.Output;
      }
    }
  }
  else if(Enterflag == 1)
  {
    if(L_flag)
    {
 
      Left_Motor.Tarspeed = base_speed -20;
      Right_Motor.Tarspeed = base_speed;
    }
    else if(R_flag)
    {

      Left_Motor.Tarspeed = base_speed;
      Right_Motor.Tarspeed = base_speed - 20;
    }
  }
  
  if(IsEmpty())
  {
    Left_Motor.Tarspeed = 0;
    Right_Motor.Tarspeed = 0;
  }
  
    stopcounter++;
  
  if(IsImLine() && stopcounter > 20)
  {
    stopflag = 1;
    Left_Motor.Tarspeed = 0;
    Right_Motor.Tarspeed = 0;
  }
  
  //获取当前速度
  Left_Motor.Realspeed = left_speed;
  Right_Motor.Realspeed = right_speed;
  if(stopflag)
  {
      FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH4,0);
      FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH5,0);
      FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH6,0);
      FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH7,0);
  }
  else
  {
  //PID计算
    PID_Ctrl(&Left_Motor);
    PID_Ctrl(&Right_Motor);
    
    PWM_Ctrl();
  }
}

void PWM_Ctrl(void)
{
  //左
  if(Left_Motor.Output > 0)
  {
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH4,Left_Motor.Output);
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH5,0);
  }
  else
  {    
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH4,0);
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH5,(-Left_Motor.Output));
  }
  
  //右
  if(Right_Motor.Output > 0)
  {
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH6,Right_Motor.Output);
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH7,0);
  }
  else
  {    
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH6,0);
    FTM_PWM_ChangeDuty(HW_FTM0,HW_FTM_CH7,(-Right_Motor.Output));
  }
}



