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

int16_t Thrust_Value;

// pitch 往前低头为正 
// Roll 右倾为正
float Pitch,Roll;   

PID_S Roll_PID={1.5,0,0,0,0,0,0,0.005f};
PID_S Pitch_PID={-2,0,-0.0005f,0,0,0,0,0.005f};
PID_S Theta_PID={0,0,0};

PID_S Pitch_Angle_Speed_PID={0.5,0,0,0,0,0,0,0.005f};
PID_S Roll_Angle_Speed_PID={0.75f,0,0.001f,0,0,0,0,0.005f};

PID_S Theta_Angle_Speed_PID={-1,0,-0.005f,0,0,0,0,0.005f};

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
  // theat 为正值，舵机向前
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
  if(duty>95){
    duty=95;
  }else if(duty<0){
    duty=0;
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

float Caculate_theta(float pitch,float roll);

#define PI 3.1415926f
void Control_Loop(){
  static int16_t ac[3],gy[3];
  static float angle_speed[3],ac_angle[3],angle[3];
  float Roll_Out=0;
  float Pitch_Out=0;
  float Pitch_Angle_Speed_Out,Roll_Angle_Speed_Out;
  float Theta_Angle_Speed_Out;
  float result[4]={0};
  static int loop_cnt=0;

  float base_duty=0;

  if(!MS_5_Flag){
    return ;
  }
  MPU_Read6500(&MPU9250,ac,gy);
  Gyroraw_to_Angle_Speed(&MPU9250,gy,angle_speed);
  get_angle(ac,angle_speed,angle,ac_angle);
  
  Pitch=angle[1];
  Roll=angle[0];
  
  //  Roll_Out=PID_Control(&Roll_PID,Roll_Stable,Roll);

  // 前倾 角度为正  角速度为负
  // 左倾 角度为负 角速度为负    最终输出 左+右-，所以ROLL_Angle_PID 为正
  Pitch_Out=PID_Control(&Pitch_PID,Pitch_Stable,Pitch);
  Roll_Out=PID_Control(&Roll_PID,Roll_Stable,Roll);

  Pitch_Angle_Speed_Out=PID_Control(&Pitch_Angle_Speed_PID,Pitch_Out,angle_speed[0]);
  Roll_Angle_Speed_Out=PID_Control(&Roll_Angle_Speed_PID,Roll_Out,angle_speed[1]);

  Theta_Angle_Speed_Out=PID_Control(&Theta_Angle_Speed_PID,Pitch_Out,angle_speed[0]);

  Pitch_Angle_Speed_Out=fabs(Pitch_Angle_Speed_Out);

  base_duty=Thrust_Value/128.0f*50.0f;
  if(base_duty<0){
    base_duty=0;
  }
  
  if (Aircraft_Mode==Mode_Arm){
    Set_Servor(L_Servor,Theta_Angle_Speed_Out);
    Set_Servor(R_Servor,Theta_Angle_Speed_Out);
    Set_ESC(L_ESC,Pitch_Angle_Speed_Out+Roll_Angle_Speed_Out+base_duty);
    Set_ESC(R_ESC,Pitch_Angle_Speed_Out-Roll_Angle_Speed_Out+base_duty);
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
  send_wave(angle[0],angle[1],angle_speed[0],angle_speed[1]);

  MS_5_Flag=0;
}

void Control_Caculate(float Fz,float Mx,float My,float Mz,float *result){
  ;
}



void NRF_Receive_Callback(uint8_t * data,int len){
  
  uint16_t state=0;
  if(len==10){
   memcpy(&state,data+8,2);
   memcpy(&Thrust_Value,data+4,2);
   state&=0x3FFF;
   if(state==1){
     Aircraft_Mode=Mode_Arm;
   }else if(state==8){
     Aircraft_Mode=Mode_Stop;
   }
  }
}

extern uint8_t Init_OK;
void Ms_IRQ_Handler(){
  static int Ms_Cnt=0;
  if(!Init_OK){
    return ;
  }
  if(Ms_Cnt==5){
    MS_5_Flag=1;
    Ms_Cnt=0;
  }

  Ms_Cnt++;
}