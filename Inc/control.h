#ifndef __CONTROL_H
#define __CONTROL_H

enum Servors{
  L_Servor,
  R_Servor
};

enum ESCs{
  L_ESC,
  R_ESC
};

enum Modes{
  Mode_Wait,
  Mode_Arm,
  Mode_Takeoff,
  Mode_Stop
};



void Control_Caculate(float Fz,float Mx,float My,float Mz,float *result);
void Set_ESC(enum ESCs ESC,float duty);
void Set_Servor(enum Servors Servor,float theta);
void Servors_Init();
void ESC_Init();
void Control_Loop();
void Ms_IRQ_Handler();
#endif