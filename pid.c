#include "pid.h"
#include <stdio.h>
#include "user_routines.h"

//initializes the PID controller in a safe, simple way.
void init_pid(DT_PID* pid_data, int P, int I, int D, int iRange, int ct) {
	pid_data->Kp = P;
	pid_data->Ki = I;
	pid_data->Kd = D;
	pid_data->Ki_Limit = iRange;
	pid_data->prevError = 0;
	pid_data->totalError = 0;
	pid_data->loop_done = 0;
	pid_data->completion_threshold = ct;
}

void pid_set_Kp(DT_PID* pid_data, int value) {
	pid_data->Kp = value;
}

char pid_isDone(DT_PID* pid_data) {
	return pid_data->loop_done;
}

//Update the control and return the PWM value
unsigned char pid_control(DT_PID* pid_data, int error) {
	int P, I, D, diff;
	P = ((long)error * pid_data->Kp)/100;
	I = ((long)pid_data->totalError * pid_data->Ki)/1000;
	D = ((long)(pid_data->prevError - error) * pid_data->Kd)/10;

	diff = pid_data->prevError - error;
	
	pid_data->prevError = error;
	pid_data->totalError += error;

	if (pid_data->totalError > pid_data->Ki_Limit) {
		pid_data->totalError = pid_data->Ki_Limit;
	}else if (pid_data->totalError < -pid_data->Ki_Limit) {
		pid_data->totalError = -pid_data->Ki_Limit;
	}

	if (error <= pid_data->completion_threshold && error >= -pid_data->completion_threshold) {  //determines if the loop is finished
		if (diff == 0) { 
			pid_data->loop_done = 1;
		}else{
			pid_data->loop_done = 2;
		}
	}else{
		pid_data->loop_done = 0;
	}

	//printf("\r\nerror: %d | P: %d | I: %d | D: %d | Tot: %d", error, P, I, diff, pid_data->totalError);
	return Limit_Mix(2000 + 127 + P + I - D);
}
