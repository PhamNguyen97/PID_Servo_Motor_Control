#include "PID.h"




void PID_Init(PID* PIDx, float Ts, float db)
{
		PIDx->e = 0;
		PIDx->e_1 = 0;
		PIDx->e_2 = 0;
		PIDx->u = 0;
		PIDx->u_1 = 0;
		PIDx->Ts = Ts;
		PIDx->deadband = db;
}
void PID_cal(int sp, int position,PID* PIDx)
{
	int e_new;
	//int value;
	e_new = sp-position;

	PIDx->e_2 = PIDx->e_1;
	PIDx->e_1 = PIDx->e;
	PIDx->e = e_new;
	
	PIDx->u_1 = PIDx->u;
	
	PIDx->u = PIDx->u_1 + PIDx->Kp*PIDx->e + PIDx->Ki*PIDx->Ts*(PIDx->e + PIDx->e_1) + PIDx->Kd/PIDx->Ts*(PIDx->e - 2*PIDx->e_1 + PIDx->e_2);	
	
	if (PIDx->u>4199-PIDx->deadband) PIDx->u = 4199-PIDx->deadband;
	else if (PIDx->u<-4199+PIDx->deadband) PIDx->u = -4199+PIDx->deadband;
	
	if (PIDx->u>0)
	{
		PIDx->uB = 0;
		PIDx->uA  = (int)PIDx->u+PIDx->deadband;
	}
	else if (PIDx->u<0)
	{
		PIDx->uA = 0;
		PIDx->uB =(int)-PIDx->u+PIDx->deadband;
	}
	else
	{
		PIDx->uA = 0;
		PIDx->uB = 0;
	}

}

void PID_update(PID* PIDx,float Kp,float Ki,float Kd,float Ts,float deadband)
{
		PIDx->Kp = Kp;
		PIDx->Ki = Ki;
		PIDx->Kd = Kd;
		PIDx->Ts = Ts;
	PIDx->deadband = deadband;
}

