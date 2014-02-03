 /*******************************************************************************
* FILE NAME: user_routines.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the default mappings of inputs  
*  (like switches, joysticks, and buttons) to outputs on the RC.  
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
*******************************************************************************/

#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "camera.h"	
#include "tracking.h"
#include "encoder.h"
#include "pid.h"
#include "adc.h"
#include "gyro.h"

extern unsigned char aBreakerWasTripped;

//Define the PID controllers
DT_PID arm;
DT_PID wrist;
DT_PID Mr_Roboto;
DT_PID robot_dist;
DT_PID gyro_c;

int encoder_1_count = 0, encoder_2_count = 0; 
long int pan_gyro_angle = 0, desired_robot_angle = 0;
int where_i_want_to_be = 0, desired_wrist_pos = 0;
char extender = 0;
char ext_but_prev = 1;
long int zach_var = 245;

char score_pos = score_low;

char tar_prev = 0;

#define able_to_correct ( p2_sw_top || p2_sw_aux1 || p2_sw_aux2 || p2_sw_trig)
#define track_a_light (p1_sw_top || p1_sw_aux1 || p1_sw_aux2)


/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Max
* PURPOSE:       Sets a PWM value to neutral (127) if it exceeds 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Max(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value > 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Min
* PURPOSE:       Sets a PWM value to neutral (127) if it's less than 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Min(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value < 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Mix
* PURPOSE:       Limits the mixed value for one joystick drive.
* CALLED FROM:   Default_Routine, this file
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     intermediate_value    int    I    
* RETURNS:       unsigned char
*******************************************************************************/
unsigned char Limit_Mix (int intermediate_value)
{
  static int limited_value;
  
  if (intermediate_value < 2000)
  {
    limited_value = 2000;
  }
  else if (intermediate_value > 2254)
  {
    limited_value = 2254;
  }
  else
  {
    limited_value = intermediate_value;
  }
  return (unsigned char) (limited_value - 2000);
}

/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.  
*                You may modify and add to this function.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{
  Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

/* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_02 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;
  digital_io_18 = INPUT;  /* Used for pneumatic pressure switch. */
    /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

/* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */
  digital_io_17 = INPUT;    /* Example - Not used in Default Code. */

/* THIRD: Initialize the values on the digital outputs. */
//  rc_dig_out17 = 0;

/* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;

/* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16.
  /*   Choose from these parameters for PWM 13-16 respectively:               */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...) */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.           */
  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);

  /* 
     Example: The following would generate a 40KHz PWM with a 50% duty cycle on the CCP2 pin:

         CCP2CON = 0x3C;
         PR2 = 0xF9;
         CCPR2L = 0x7F;
         T2CON = 0;
         T2CONbits.TMR2ON = 1;

         Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

  /* Add any other initialization code here. */
 
  Init_Serial_Port_One();
  Init_Serial_Port_Two();

#ifdef TERMINAL_SERIAL_PORT_1    
  stdout_serial_port = SERIAL_PORT_ONE;
#endif

#ifdef TERMINAL_SERIAL_PORT_2    
  stdout_serial_port = SERIAL_PORT_TWO;
#endif

	Initialize_Encoders();
	//I EXPECT INITIALIZATION VALUES
	Reset_Encoder_1_Count(-153);
	Reset_Encoder_2_Count(-415);

	//start comment
	Initialize_Gyro();
    Initialize_ADC();
	//end comment

	init_pid(&arm, 100, 0, 0, 120, 35);  // 275 0 0
	init_pid(&wrist, 33, 0, 0, 20, 80); //45 30 0
	init_pid(&Mr_Roboto, 55, 0 , 0, 100, 25);
	init_pid(&robot_dist, 95, 0, 0, 100, 8);
	init_pid(&gyro_c, 20, 0, 0, 100, 8);

  Putdata(&txdata);            /* DO NOT CHANGE! */

//  ***  IFI Code Starts Here  ***
//
//  Serial_Driver_Initialize();

  printf("IFI 2006 User Processor Initialized ...\r");  /* Optional - Print initialization message. */

  User_Proc_Is_Ready();         /* DO NOT CHANGE! - last line of User_Initialization */
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 26.2ms when it gets new data from the master 
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP(void)
{
	static unsigned int j = 0;

	Getdata(&rxdata);
	
	Camera_Handler();
	Servo_Track();
	//PAN_SERVO = 127;
	//TILT_SERVO = 127;
	Default_Routine();

	j++;
	j++;

	if(j == 1)	{
		printf("\rCalculating Gyro Bias...");
	}
	if(j == 6)	{
		Start_Gyro_Bias_Calc();
	}
	if(j == 200)	{
		Stop_Gyro_Bias_Calc();
		Reset_Gyro_Angle();
		printf("Done\r");
	}

	Putdata(&txdata);
}

//PWM_LIMIT - limits a PWM value x to y
unsigned char pwm_limit ( int pwmval, char range) {
	if (pwmval > 127 + range) {
		pwmval = 127 + range;
	}else if (pwmval < 127 - range) {
		pwmval = 127 - range;
	}
	return (unsigned char)pwmval;
}

unsigned char flip_axis(unsigned char valueToFlip, unsigned char axisFlipOver) {
	int returnValue;
	returnValue = (-1)*(valueToFlip - axisFlipOver);
	returnValue += axisFlipOver;
	return returnValue;
}

unsigned char motor_tone(unsigned char value, char numer, char denom) {
	return ( ( (int)value - 127 ) * numer ) / denom     +    127;
}

/*******************************************************************************
* FUNCTION NAME: Default_Routine
* PURPOSE:       Performs the default mappings of inputs to outputs for the
*                Robot Controller.
* CALLED FROM:   this file, Process_Data_From_Master_uP routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Default_Routine(void)
{   
	static int temp_angle = 0;
	//%d  = decimal
	//%i  = integer
	//%li = long integer
	encoder_1_count = (int)Get_Encoder_1_Count();
	encoder_2_count = (int)Get_Encoder_2_Count();
	pan_gyro_angle 	= Get_Gyro_Angle();
	//debug
	printf("ARM: %i | WRIST: %i | cam tilt %i | cam_pan : %i | M: %li %i\r\n", encoder_1_count, encoder_2_count, PAN_SERVO, TILT_SERVO, Get_Gyro_Angle(), Get_ADC_Result(2));
	//printf("%i %i %i %i", auto_switch_1, auto_switch_2, auto_switch_3, auto_switch_4)
	//printf("\r\nauto_switch_1: %i | auto_switch_2: %i | auto_switch_3: %i | auto_switch_4: %i", auto_switch_1, auto_switch_2, auto_switch_3, auto_switch_4);
	//DRIVETRAIN CONTROL (arcade drive)
	if (!p4_sw_aux2 && !p4_sw_top) {
		drive_R1 = drive_R2 = Limit_Mix(2000 + (p3_x) + p3_y - 127);
		drive_L1 = drive_L2 = flip_axis(Limit_Mix(2000 + (p3_x) - p3_y + 127), 127);	
		tar_prev = 0;
		desired_robot_angle = 0;
	}else if (p4_sw_top) {
		if (tar_prev == 0) {
			tar_prev = 1;
			desired_robot_angle = pan_gyro_angle;
			//printf("\nEntered drive mode\n");
		}
	
		temp_angle = pid_control(&gyro_c, pan_gyro_angle - desired_robot_angle);

		drive_R1 = drive_R2 = Limit_Mix(2000 + (temp_angle) + p3_y - 127);
		drive_L1 = drive_L2 = flip_axis(Limit_Mix(2000 + (temp_angle) - p3_y + 127), 127);
	}else if (p4_sw_aux2) {
		drive_R1 = drive_R2 = Limit_Mix(2000 + (p3_x) + flip_axis(p3_y, 127) - 127);
		drive_L1 = drive_L2 = flip_axis(Limit_Mix(2000 + (p3_x) - flip_axis(p3_y, 127) + 127), 127);
	}
	/*else{
		drive_R1 = drive_L1 = pid_control(&Mr_Roboto, pan_gyro_angle - desired_robot_angle);
	}*/
	//printf("gy: %i | %i  | %i\r\n", drive_R1, drive_L1, tar_prev);

	//if (tar_prev && !p4_sw_aux2) {
	//	tar_prev = 0;
	//}
	//printf("pid, dones (arm, wrist, dist, angle) (%i)", score_pos);


	////**V*V*V*V*V*V*V*V*V*V*V*V*V*V*V*V*V*V*V*V*V*V*V*V*V*V
	//SET THE ARM POS
	if (p2_sw_aux2 == 1) {  //UPPER
		set_arm_pos(ARM_TOP, WRIST_TOP);
		score_pos = score_top;
	}else if(p2_sw_aux1 == 1) {  //MIDDLE
		set_arm_pos(ARM_MID, WRIST_MID);
		score_pos = score_mid;
	}else if(p2_sw_top == 1) {  //LOWER
		set_arm_pos(ARM_LOW, WRIST_LOW);
		score_pos = score_low;
	}else if (p2_sw_trig == 1) { //HOME
		set_arm_pos(ARM_HOME, WRIST_HOME);
	}

	//Zach's Absolute Arm Positions
	//set_arm_pos(p3_y, WRIST_HOME);
	
	

	//where_i_want_to_be -= (((int)p1_y - 127)/30);  //PRECISION ARM
	//desired_wrist_pos  += (((int)p2_y - 127)/20);  //PRECISION WRIST

	if (where_i_want_to_be > ARM_MAX) {  //ARM LIMIT
		where_i_want_to_be = ARM_MAX;
	}else if (where_i_want_to_be < ARM_MIN) {
		where_i_want_to_be = ARM_MIN;
	}
	//END SET ARM POS
	////**^*^*^*^*^*^*^*^*^*^^*^*^*^*^*^*^*^*^*^*^*^*^*^*^*^*^*^*^*^*

	if (p1_sw_aux1) {	//Set pickup position
			zach_var = encoder_2_count;
	}

	if (p1_sw_trig > 0) {  //SAFETY TRIGGER
		arm_l_motor = arm_r_motor = p1_y;
		wrist_motor = flip_axis(p2_y, 127);
		where_i_want_to_be = encoder_1_count;
		desired_wrist_pos = encoder_2_count;
	}else if (able_to_correct) {  //CONSTANT CONTROL
		arm_l_motor = arm_r_motor = pid_control(&arm, where_i_want_to_be -  encoder_1_count);
		wrist_motor = pid_control(&wrist, desired_wrist_pos - encoder_2_count);
	}else if (p1_sw_top == 1) {
		arm_l_motor = arm_r_motor = p1_y;
		if (p4_sw_trig) {
			desired_wrist_pos = (3 * encoder_1_count)/4 + 276 - ((int)p2_y - 127); //insert formula here.
		}else{
			desired_wrist_pos = zach_var;	//ZACH_AND_ELLEN_RULE	//245
		}
		wrist_motor = pid_control(&wrist, desired_wrist_pos - encoder_2_count);
	}else{
		arm_l_motor = arm_r_motor = wrist_motor = 127;
	}
    

	ramp_up = p4_sw_aux1;
	ramp_down = !p4_sw_aux1;

	//JAW CONTROL (player 1 top);
	grabber =  p3_sw_trig | p1_sw_aux2;

	//EXTENDER ARM
	if (p3_sw_top == 1 && ext_but_prev == 0) {
		extender = (extender == 1) ? 0 : 1;
		ext_but_prev = 1;
	}else{
		if (!p3_sw_top) {
			ext_but_prev = 0;
		}
	}

	extension = extender;
	/*
	if (track_a_light) {
		if (Targets.num_of_lights == 1) {
			desired_robot_angle = desired_robot_angle = ((((int)TILT_SERVO - 127)*65)/127  +  Targets.c_light_angle) * 10   +   pan_gyro_angle;
		}else{
			switch (p1_sw_top | (p1_sw_aux1 << 2) | (p1_sw_aux2 << 3)) {
				case 1:
					desired_robot_angle = desired_robot_angle = ((((int)TILT_SERVO - 127)*65)/127  +  Targets.l_light_angle) * 10   +   pan_gyro_angle;
				break;
				case 4:
					desired_robot_angle = desired_robot_angle = ((((int)TILT_SERVO - 127)*65)/127  +  Targets.c_light_angle) * 10   +   pan_gyro_angle;
				break;
				case 8:
					desired_robot_angle = desired_robot_angle = ((((int)TILT_SERVO - 127)*65)/127  +  Targets.r_light_angle) * 10   +   pan_gyro_angle;
				break;
			}
		}
	}*/

	
	
	//COMPRESSOR CONTROL
	//compressor = !pressure_switch;

} /* END Default_Routine(); */


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
//
