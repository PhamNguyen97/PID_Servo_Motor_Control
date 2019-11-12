typedef struct
{  

  float e; 
	float e_1; 
	float e_2;                                
  float Kp;                               
  float Ki;      
  float Kd;
	float deadband;
  float u; 
	float u_1; 
	float Ts;  
	float uA;
	float uB;
} PID;


extern void PID_Init(PID* PIDs);
extern void PID_cal(int sp, int x,PID* PIDs);
extern void PID_update(PID* PIDs,float Kp,float Ki,float Kd);
