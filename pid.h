typedef struct {
	int Kp;  	//precision: .01
	int Ki;  	//precision: .001
	int Kd;		//precision: .1
	int prevError;
	int totalError;
	int Ki_Limit;
	int completion_threshold;
	char loop_done;
} DT_PID;

extern DT_PID arm;
extern DT_PID wrist;
extern DT_PID Mr_Roboto;
extern DT_PID robot_dist;

unsigned char pid_control(DT_PID* pid_data, int error);
void init_pid(DT_PID* pid_data, int P, int I, int D, int iRange, int ct);
void pid_set_Kp(DT_PID* pid_data, int value);
char pid_isDone(DT_PID* pid_data);

#define pid_incomplete	0
#define pid_complete	1
#define pid_inRange		2

