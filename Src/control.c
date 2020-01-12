#include "control.h"
#include "tim.h"
#include "icm20600.h"
#include "easy_angle.h"
#include "uart_ext.h"
#include "math.h"
#include "pid.h"
#include "adc.h"

float Pitch_Stable=90.0f;
float Roll_Stable=90.0f;

#define L_Servor_Polarity 1
#define R_Servor_Poloarity -1

#define Servor_Angle_Limit 45.0f  // 定义舵机单边最大运动角度

int L_Servor_Middle=1400;
int R_Servor_Middle=1650;

int Servor_Width_Limit=0; // 舵机脉宽限制 

#define MIN(a,b) a<b?a:b

// pitch 往前低头为正 
// Roll 右倾为正
float Pitch,Roll;   

PID_S Roll_PID={0,0,0};
PID_S Pitch_PID={0,0,0};

enum Modes Aircraft_Mode=Mode_Wait;

uint8_t MS_5_Flag=0; // 5ms flag

float BAT_Voltage=0;

float Get_Voltage(uint16_t R1,uint16_t R2){
  uint16_t adc_value=0;
  float voltage=0;
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1,10);
  adc_value=HAL_ADC_GetValue(&hadc1);
  
  voltage=(float)(R1+R2)/R2*adc_value/4096.0f*3300;
  return voltage;
}

void Servors_Init(){
  // 因为两个舵机的安装位置都不是1500处位于中位，因此需要做一些限制，保证舵机在两边的运动范围一样
  // 需要注意的，这里取的仅是一个相对值，如Servor_Angle_Limit为45，Servor_Width_Limit为350
  // 并不是说当脉宽为中值+350时，舵机就转动45°，仅仅是一个相对的量
  int L_Min=0;
  int R_Min=0;
  L_Min=MIN(L_Servor_Middle-1000,2000-L_Servor_Middle);
  R_Min=MIN(R_Servor_Middle-1000,2000-R_Servor_Middle);
  
  Servor_Width_Limit=MIN(L_Min,R_Min);
}

void ESC_Init(){
  TIM3->CCR4=Caculate_Cnt(&htim3,64,1000);
  TIM3->CCR3=Caculate_Cnt(&htim3,64,1000);
}

void Set_Servor(enum Servors Servor,float theta){
  // theta 取值 -45 到 45 因为舵机的运动范围并没有到180°
  if(theta>Servor_Angle_Limit){
    theta=Servor_Angle_Limit;
  }else if(theta<-Servor_Angle_Limit){
    theta=-Servor_Angle_Limit;
  }
  int sub=(int)(theta/Servor_Angle_Limit*Servor_Width_Limit);
  switch(Servor){
  case L_Servor:
    TIM2->CCR2=Caculate_Cnt(&htim2,64,L_Servor_Middle+sub*L_Servor_Polarity);
    break;
  case R_Servor:
    TIM2->CCR1=Caculate_Cnt(&htim2,64,R_Servor_Middle+sub*R_Servor_Poloarity);
    break;
  }
}

void Set_ESC(enum ESCs ESC,float duty){
  if(duty>100){
    duty=100;
  }
  switch(ESC){
  case L_ESC:
    TIM3->CCR4=Caculate_Cnt(&htim3,64,(int)(1000+duty*10));
    break;
  case R_ESC:
    TIM3->CCR3=Caculate_Cnt(&htim3,64,(int)(1000+duty*10));
    break;
  }
}

float Transform_N_to_Duty(float F){
  // 1N->40%
  return F/1*40;
}
float Transform_Rad_to_Dgree(float rad){
  return rad/3.14f*360.0f;
}

void Control_Loop(){
  static int16_t ac[3],gy[3];
  static float angle_speed[3],ac_angle[3],angle[3];
  float Roll_Out=0;
  float result[4]={0};
  static int loop_cnt=0;

  if(!MS_5_Flag){
    return ;
  }
  MPU_Read6500(&MPU9250,ac,gy);
  Gyroraw_to_Angle_Speed(&MPU9250,gy,angle_speed);
  get_angle(ac,angle_speed,angle,ac_angle);
  
  Pitch=angle[1];
  Roll=angle[0];
  
  Roll_Out=PID_Control(&Roll_PID,Roll_Stable,Roll);
  
  Control_Caculate(1,Roll_Out,0,0,result);
  if (Aircraft_Mode==Mode_Arm){
    Set_Servor(L_Servor,0);
    Set_Servor(R_Servor,0);
    Set_ESC(L_ESC,50);
    Set_ESC(R_ESC,50);
  }else if (Aircraft_Mode==Mode_Takeoff){
    Set_ESC(L_ESC, Transform_N_to_Duty(result[0]));
    Set_ESC(R_ESC,Transform_N_to_Duty(result[1]));
    Set_Servor(L_Servor,Transform_Rad_to_Dgree(result[2]));
    Set_Servor(R_Servor,Transform_Rad_to_Dgree(result[3]));
  }else if(Aircraft_Mode==Mode_Stop){
    Set_Servor(L_Servor,0);
    Set_Servor(R_Servor,0);
    Set_ESC(L_ESC,0);
    Set_ESC(R_ESC,0); 
  }
  
  loop_cnt++;
  if(loop_cnt>10){
    BAT_Voltage=Get_Voltage(5100,2200);
    if(BAT_Voltage<7400){
       HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }
    loop_cnt=0;
  }
  send_wave(angle[0],angle[1],BAT_Voltage/100,0);

  MS_5_Flag=0;
}


void Control_Caculate(float Fz,float Mx,float My,float Mz,float *result){
  /*顺序为 F1 F2 theta1 theta2

/               #2              \
|             -----             |
|             2 d r             |
|                               |
|               #1              |
|             -----             |
|             2 d r             |
|                               |
|      / Mx r - #2 + Fz d r \   |
| -atan| ------------------ | 2 |
|      \     My d + Mz r    /   |
|                               |
|      / Mx r + #1 - Fz d r \   |
|  atan| ------------------ | 2 |
\      \     My d - Mz r    /   /

where

                2  2  2              2     2  2     2  2                   2  2
   #1 == sqrt(Fz  d  r  - 2 Fz Mx d r  + Mx  r  + My  d  - 2 My Mz d r + Mz  r )

                2  2  2              2     2  2     2  2                   2  2
   #2 == sqrt(Fz  d  r  + 2 Fz Mx d r  + Mx  r  + My  d  + 2 My Mz d r + Mz  r )
*/
  
  float M1, M2;
  
  float F1,F2,theta1,theta2;
  float d,r;
  
  r=0.04f; // 单位 m
  d=0.1f;
  
  float temp1=Fz*Fz*d*d*r*r+Mx*Mx*r*r+My*My*d*d+Mz*Mz*r*r;
  float temp2=2*Fz*Mx*d*r*r+2*My*Mz*d*r;
  
  M1=sqrtf(temp1-temp2);
  M2=sqrtf(temp1+temp2);
  
  F1=M2/(2*d*r);
  F2=M1/(2*d*r);
  
  theta1=-2*atan2f(Mx*r-M2+Fz*d*r,(My*d+Mz*r+0.000000001f));
  theta2=2*atan2f(Mx*r+M1-Fz*d*r,(My*d-Mz*r+0.000000001f));
  
  result[0]=F1;
  result[1]=F2;
  result[2]=theta1;
  result[3]=theta2;
}

void NRF_Receive_Callback(uint8_t * data,int len){
  
  uint16_t state=0;
  if(len==10){
   memcpy(&state,data+8,2);
   state&=0x3FFF;
   if(state==1){
     Aircraft_Mode=Mode_Arm;
   }else if(state==8){
     Aircraft_Mode=Mode_Stop;
   }
  }
}

void Ms_IRQ_Handler(){
  static int Ms_Cnt=0;
  if(Ms_Cnt==5){
    MS_5_Flag=1;
    Ms_Cnt=0;
  }

  Ms_Cnt++;
}