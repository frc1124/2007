/*******************************************************************************
* FILE NAME: user_routines_fast.c <FRC VERSION>
*
* DESCRIPTION:
*  This file is where the user can add their custom code within the framework
*  of the routines below. 
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
* OPTIONS:  Interrupts are disabled and not used by default.
*
*******************************************************************************/
#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "encoder.h"
#include "adc.h"
#include "gyro.h"
#include "pid.h"
#include "camera.h"
#include "tracking.h"

/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/


/*******************************************************************************
* FUNCTION NAME: InterruptVectorLow
* PURPOSE:       Low priority interrupt vector
* CALLED FROM:   nowhere by default
* ARGUMENTS:     none
* RETURNS:       void
* DO NOT MODIFY OR DELETE THIS FUNCTION 
*******************************************************************************/
#pragma code InterruptVectorLow = LOW_INT_VECTOR
void InterruptVectorLow (void)
{
  _asm
    goto InterruptHandlerLow  /*jump to interrupt routine*/
  _endasm
}


/*******************************************************************************
* FUNCTION NAME: InterruptHandlerLow
* PURPOSE:       Low priority interrupt handler
* If you want to use these external low priority interrupts or any of the
* peripheral interrupts then you must enable them in your initialization
* routine.  Innovation First, Inc. will not provide support for using these
* interrupts, so be careful.  There is great potential for glitchy code if good
* interrupt programming practices are not followed.  Especially read p. 28 of
* the "MPLAB(R) C18 C Compiler User's Guide" for information on context saving.
* CALLED FROM:   this file, InterruptVectorLow routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
#pragma code
#pragma interruptlow InterruptHandlerLow save=PROD,section(".tmpdata")
//#pragma interruptlow InterruptHandlerLow save=PROD
void InterruptHandlerLow ()     
{
	unsigned char Port_B;
	unsigned char Port_B_Delta;

	if (PIR1bits.RC1IF && PIE1bits.RC1IE) // rx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_RX
		Rx_1_Int_Handler(); // call the rx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.RC2IF && PIE3bits.RC2IE) // rx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_RX
		Rx_2_Int_Handler(); // call the rx2 interrupt handler (in serial_ports.c)
		#endif
	} 
	else if (PIR1bits.TX1IF && PIE1bits.TX1IE) // tx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_TX
		Tx_1_Int_Handler(); // call the tx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.TX2IF && PIE3bits.TX2IE) // tx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_TX
		Tx_2_Int_Handler(); // call the tx2 interrupt handler (in serial_ports.c)
		#endif
	}
	else if(PIR1bits.TMR2IF && PIE1bits.TMR2IE) // timer 2 interrupt?
	{
		PIR1bits.TMR2IF = 0; // clear the timer 2 interrupt flag [92]
		Timer_2_Int_Handler(); // call the timer 2 interrupt handler (in adc.c)
	}                     
	else if(PIR1bits.ADIF && PIE1bits.ADIE) // ADC interrupt
	{
		PIR1bits.ADIF = 0; // clear the ADC interrupt flag
		ADC_Int_Handler(); // call the ADC interrupt handler (in adc.c)
	}
	else if (INTCON3bits.INT2IF && INTCON3bits.INT2IE) // encoder 1 interrupt?
	{ 
		INTCON3bits.INT2IF = 0; // clear the interrupt flag
		#ifdef ENABLE_ENCODER_1
		Encoder_1_Int_Handler(); // call the left encoder interrupt handler (in encoder.c)
		#endif
	}
	else if (INTCON3bits.INT3IF && INTCON3bits.INT3IE) // encoder 2 interrupt?
	{
		INTCON3bits.INT3IF = 0; // clear the interrupt flag
		#ifdef ENABLE_ENCODER_2
		Encoder_2_Int_Handler(); // call right encoder interrupt handler (in encoder.c)
		#endif
	}


	//end comment
}


/*******************************************************************************
* FUNCTION NAME: User_Autonomous_Code
* PURPOSE:       Execute user's code during autonomous robot operation.
* You should modify this routine by adding code which you wish to run in
* autonomous mode.  It will be executed every program loop, and not
* wait for or use any data from the Operator Interface.
* CALLED FROM:   main.c file, main() routine when in Autonomous mode
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Autonomous_Code(void)
{
	/****AUTO MODE SWITCH******
	Switches in the order which they are on the auto-switch box: top to bottom.
	red is generally on, while green is off (???)
	switch 4:
		red: Do the defense mode
		green: Do NOT do the defense mode
	switch 3:
		red: Score on the middle goal
		green: Score on the low goal
	switch 2:
		red: Drive counter-clockwise in the defense mode (RIGHT)
		green: Drive clockwise in the defense mode (LEFT)
	switch 1:
		red: Drive to the other side of the field before executing the auto mode
		green: Normal driver mode
	*/



	//on-the-fly distance formula variable
	int mid_var;
	//zack's tweak variable
	int auto_mode_type;
	//old variables used as aliases for the auto switches
	char auto_sel, auto_sel_arm;
	//temporary drive variables used in the correction loops
	char temp_L_drive, temp_R_drive;
	//drive_mode: the driving mode of the robot (absolute, straight, dist/angle, none)
	//arm_mode: the driving mode of the arm (PID correction, none)
	//rout_mode: variable that allows for the state machine to run
	char drive_mode = drive_none_state, arm_mode = arm_none_state, rout_mode = rsm_target_search;
	//parameters for the drive-train correction system (errors, basically)
	int des_angle = 0, des_dist = 0;
	//intermediate correction values (unused)
	int temp_position_var = 127, temp_angle_var = 127, timer = 0;
	//temporary variabel for the gyro position
	long int gyro_angle = 0;
	//PID parameters for gyro correction.
	DT_PID temp_gyro_c;

  	/* Initialize all PWMs and Relays when entering Autonomous mode, or else it
     will be stuck with the last values mapped from the joysticks.  Remember, 
     even when Disabled it is reading inputs from the Operator Interface. 
  	*/
	pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
	pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;
	relay1_fwd = relay1_rev = relay2_fwd = relay2_rev = 0;
	relay3_fwd = relay3_rev = relay4_fwd = relay4_rev = 0;
	relay5_fwd = relay5_rev = relay6_fwd = relay6_rev = 0;
	relay7_fwd = relay7_rev = relay8_fwd = relay8_rev = 0;

	auto_mode_type = 1;

	rout_mode = rsm_to_other_side;

	init_pid(&temp_gyro_c, 20, 0, 0, 0, 30);

	//Configuration Values
	//auto_mode = auto_switch_A | (auto_switch_B << 1);  //0-3
	//auto_sel  = mode_switch_A | (mode_switch_B << 1);  //0-3
	auto_sel_arm = auto_switch_3;

	arm.loop_done = 0;
	wrist.loop_done = 0;
	timer = 0;

	//set the default wrist positions
	where_i_want_to_be = ARM_HOME;
	desired_wrist_pos = WRIST_HOME;

	//NOTE: THERE ARE MULTIPLE CHANGES MADE TO THIS AUTO MODE IN switch-case STATEMENTS
	//THAT ARE TWEAKS MADE BY ZACK.  CHANGING auto_mode_type WILL AFFECT THIS switch
	//STATEMENT

	while (autonomous_mode){   /* DO NOT CHANGE! */
		Process_Data_From_Local_IO();
		if (statusflag.NEW_SPI_DATA) {	//slow loop
			explode();  /* DO NOT DELETE, or you will not explode! */
			Camera_Handler();

			//we do NOT want the camera searching during auto other-side mode
			if (rout_mode != rsm_to_other_side) {
				Servo_Track();
			}else{
				PAN_SERVO = 124;
				TILT_SERVO = 144;
			}

			//retrieve the encoder counts AND gyro angle
			encoder_1_count = (int)Get_Encoder_1_Count();
			encoder_2_count = (int)Get_Encoder_2_Count();
			gyro_angle = Get_Gyro_Angle();
	
			//if we need to destroy the auto mode, change this to if(0) {
			if(1) {
				//printf("Mr L's equation: %i", 130 + ((PAN_SERVO - 90) / 5));
				switch (rout_mode) {
					//preliminary initialization mode to drive to the other side of the 
					//field.  If the auto switch says that it should not, then it simply
					//bypasses this mode, otherwise it will drive 300 program loops
					//straight backwards.
					case rsm_to_other_side:
						printf("G: %li", gyro_angle);
						drive_mode = drive_toAngle_state;
						arm_mode = arm_none_state;
	
						if (timer == 1) {
							temp_L_drive = gyro_angle;
						}
	
						des_dist = -60;
						des_angle = gyro_angle - temp_L_drive;
				
						timer ++;
	
						if (!auto_otherside || timer >= 220) {
							rout_mode = rsm_target_search;
							timer = 0;
						}
					break;

					//this mode sets the initial position of the arm, and allows for the
					//camera to smoothly find its target.
					case rsm_target_search:
						printf(" SEARCHING FOR TARGET ");
						//printf("\r\nauto_switch_1: %i | auto_switch_2: %i | auto_switch_3: %i | auto_switch_4: %i", auto_switch_1, auto_switch_2, auto_switch_3, auto_switch_4);
						drive_mode = drive_none_state; //drive_toAngle_state
						arm_mode = arm_correct_state;

						//hardcode section
						switch(auto_mode_type){
							case billy:
								if (auto_sel_arm == score_low) {
									set_arm_pos(23, WRIST_LOW);
								}else{
									set_arm_pos(ARM_TOP, 150);
								}
							break;

							case zach1:
								switch(auto_sel_arm){
									case score_low:
										set_arm_pos(23, WRIST_LOW);
									break;

									case score_mid:
										set_arm_pos(ARM_HOME, WRIST_HOME);
									break;
								}
							break;
						}

						robot_dist.loop_done = 0;
						Mr_Roboto.loop_done = 0;
						//printf("\r\npid_isDone(&arm): %i | pid_isDone(&wrist): %i | T_packet_Data.pixels: %i\r\ntimer: %i", pid_isDone(&arm), pid_isDone(&wrist), T_Packet_Data.pixels, timer);

						if (pid_isDone(&arm) == 1 && pid_isDone(&wrist) == 1 && T_Packet_Data.pixels > 0 && timer >= 100) {
							if (auto_otherside == 0){
								rout_mode = rsm_driving;
							}else{
								mid_var = (PAN_SERVO - 65) / 30;
								rout_mode = rsm_driving_far;
							}
							timer = 0;
						}
		
						timer++;
					break;

					//this is the driving out function for the far side of the field.
					//see the description in rsm_driving for the full description; this
					//is the same mode with a couple changes.
					case rsm_driving_far:
						printf(" DRIVING TO TARGET FAR ");
						if (T_Packet_Data.pixels > 0) {
							drive_mode = drive_toAngle_state;
							arm_mode = arm_correct_state;
						}else{
							drive_mode = drive_none_state;
							arm_mode = arm_none_state;
						}
						switch(auto_sel_arm){
							case score_low:
								des_dist = (int)PAN_SERVO - DIST_LOW_SCORE;
								//printf("\r\nPAN_SERVO: %i DIST_LOW_SCORE: %i", PAN_SERVO, DIST_LOW_SCORE);
								if (PAN_SERVO <= 153) {			//153
									set_arm_pos(ARM_LOW, WRIST_LOW);
								}else{
									set_arm_pos(23, WRIST_LOW);
								}
								des_angle = (((134 -  (long int)TILT_SERVO)*255)/127);	//135
							break;

							case score_mid:
								des_dist = (int)PAN_SERVO - DIST_MID2_SCORE;
								if (PAN_SERVO <= DIST_MID2_SCORE + 60){
									set_arm_pos(ARM_MID2, WRIST_MID2);
								}else{
									set_arm_pos(ARM_HOME, WRIST_HOME);
								}
								des_angle = (((137 - (long int)TILT_SERVO)*255)/127);	//135
							break;
						}
						arm.loop_done = 0;
						wrist.loop_done = 0;
						//printf("\r\npid_isDone(&robot_dist): %i pid_isDone(&Mr_Roboto): %i", pid_isDone(&robot_dist), pid_isDone(&Mr_Roboto));

						if (pid_isDone(&robot_dist) == 1 && pid_isDone(&Mr_Roboto) == 1) {
							rout_mode = rsm_scoring;
							timer = 0;
						}
					break;

					//this mode drives towards the light, correcting for both angle and
					//distance.  When the robot is nearing the target, the arm will raise
					//into the scoring position and hold there. The mode proceeds once the
					//PID loops have detected their completion.
					case rsm_driving:
						printf(" DRIVING TO TARGET ");
						if (T_Packet_Data.pixels > 0) {
							drive_mode = drive_toAngle_state;
							arm_mode = arm_correct_state;
						}else{
							drive_mode = drive_none_state;
							arm_mode = arm_none_state;
						}

						//hardcode section
						switch(auto_mode_type){
							case billy:
								if (auto_sel_arm == score_low) {
									des_dist = (int)PAN_SERVO - DIST_LOW_SCORE;
									//printf("\r\nPAN_SERVO: %i DIST_LOW_SCORE: %i", PAN_SERVO, DIST_LOW_SCORE);
									if (PAN_SERVO <= 153) {			//153
										set_arm_pos(ARM_LOW, WRIST_LOW);
									}else{
										set_arm_pos(23, WRIST_LOW);
									}
								}else{
									des_dist = (int)PAN_SERVO - DIST_MID_SCORE;
								}
								des_angle = (((141 - (long int)TILT_SERVO)*255)/127);
							break;

							case zach1:
								switch(auto_sel_arm){
									case score_low:
										des_dist = (int)PAN_SERVO - DIST_LOW_SCORE;
										//printf("\r\nPAN_SERVO: %i DIST_LOW_SCORE: %i", PAN_SERVO, DIST_LOW_SCORE);
										if (PAN_SERVO <= 153) {			//153
											set_arm_pos(ARM_LOW, WRIST_LOW);
										}else{
											set_arm_pos(23, WRIST_LOW);
										}
										//des_angle = (((149 + (((int)PAN_SERVO - 87) / 4) - (long int)TILT_SERVO)*255)/127);	//135
										des_angle = (((134 - (long int)TILT_SERVO)*255)/127);	//was 134 (135 a while ago)
									break;
//94 145
//165 143
									case score_mid:
										des_dist = (int)PAN_SERVO - DIST_MID2_SCORE;
										if (PAN_SERVO <= DIST_MID2_SCORE + 65){
											set_arm_pos(ARM_MID2, WRIST_MID2);
										}//else{
										//	set_arm_pos(ARM_HOME, WRIST_HOME);
										//}
										//FANCY EQN CONSTANTS   137, 93, /7
										des_angle = (((134 + (((int)PAN_SERVO - 93) / 7) - (long int)TILT_SERVO)*255)/127);	//lowered to 134 to move to the right
										//des_angle = (((137 - (long int)TILT_SERVO)*255)/127);	//135
									break;
								}//85 146; 87 149
							break;

							case zach2:
								switch(auto_sel_arm){
									case score_low:
										des_dist = (int)PAN_SERVO - DIST_LOW_SCORE;
										//printf("\r\nPAN_SERVO: %i DIST_LOW_SCORE: %i", PAN_SERVO, DIST_LOW_SCORE);
										if (PAN_SERVO <= 153) {			//153
											set_arm_pos(ARM_LOW, WRIST_LOW);
										}else{
											set_arm_pos(23, WRIST_LOW);
										}
										des_angle = (((141 - (long int)TILT_SERVO)*255)/127);
									break;

									case score_mid:
										des_dist = (int)PAN_SERVO - DIST_MID2_SCORE;
										if (PAN_SERVO <= DIST_MID3_SCORE + 60){
											set_arm_pos(ARM_MID2, WRIST_MID2);
										}
										des_angle = (((175 - (long int)TILT_SERVO)*255)/127);		//175
									break;
								}
							break;
						}

						arm.loop_done = 0;
						wrist.loop_done = 0;
						//printf("\r\npid_isDone(&robot_dist): %i pid_isDone(&Mr_Roboto): %i", pid_isDone(&robot_dist), pid_isDone(&Mr_Roboto));

						if (pid_isDone(&robot_dist) == 1 && pid_isDone(&Mr_Roboto) == 1) {
							rout_mode = rsm_scoring;
							timer = 0;
						}
					break;							
		
					//This mode allows for the robot to score; it lowers the arm into
					//scoring position and backs up once the PID loops on the arm/
					//wrist have been marked as completed.
					case rsm_scoring:
						printf(" SCORING ");
						drive_mode = drive_none_state;
						arm_mode = arm_correct_state;

						//hardcode section
						switch(auto_mode_type){
							case billy:
								switch (auto_sel_arm) {
									case score_top: set_arm_pos(ARM_TOP, WRIST_TOP); break;
									case score_mid: set_arm_pos(ARM_MID, WRIST_MID); break;
									case score_low: set_arm_pos(ARM_LOW, WRIST_LOW); break;
								}
							break;

							case zach1:
								switch (auto_sel_arm) {
									case score_top: set_arm_pos(ARM_TOP, WRIST_TOP); break;
									case score_mid: set_arm_pos(ARM_MID2, WRIST_MID2); break;
									case score_low: set_arm_pos(ARM_LOW, WRIST_LOW); break;
								}
							break;

							case zach2:
								switch (auto_sel_arm) {
									case score_top: set_arm_pos(ARM_TOP, WRIST_TOP); break;
									case score_mid: set_arm_pos(ARM_MID3, WRIST_MID3); break;
									case score_low: set_arm_pos(ARM_LOW, WRIST_LOW); break;
								}
							break;
						}

						robot_dist.loop_done = 0;
						Mr_Roboto.loop_done = 0;
						if (timer > 0) {
							if (pid_isDone(&arm)==1 && pid_isDone(&wrist)==1) {
								grabber = 1;
								rout_mode = rsm_driveback;
								timer = 0;
							}
						}
						timer++;

						
					break;

					//This is the short driveback routine that flicks the wrist and drives
	`				//backwards, ensuring that the tube gets released.
					case rsm_driveback:
						printf(" DRIVING BACK ");
						if (timer <= 80) {
							drive_mode = drive_toAngle_state;
							arm_mode = arm_correct_state;
						}else{
							drive_mode = drive_none_state;
							arm_mode = arm_correct_state;
						}
						desired_wrist_pos = 300;

						//grabber = 0;	//close gripper!!!
					
						des_dist = -20;//(int)PAN_SERVO - DIST_BACKUP;
						des_angle = 0;//(((127 - (long int)TILT_SERVO)*255)/127);
						if (timer > 30) {
							if (auto_driveback && !auto_otherside) {
								rout_mode = rsm_the_angle;
								robot_dist.loop_done = 0;
								Mr_Roboto.loop_done = 0;
								timer = 0;
							}
						}
						timer++;
					break;
	
					//this is the defense mode that makes the robot drive around the rack
					//and hopefully smash the begeebers out of robots with slower auto
					//modes.  This is the last step in the process.
					case rsm_the_angle:
						printf(" SPINNY ");
						//if (pid_isDone(&Mr_Roboto)) {
							drive_mode = drive_toAngle_state;
							arm_mode = arm_correct_state;
						/*}else{
							drive_mode = drive_none_state;
							arm_mode = arm_none_state;
						}*/
						
						grabber = 0;
						set_arm_pos(ARM_HOME, WRIST_HOME);
						
						if (auto_direction) {
							if (timer < 50) {
								des_dist = 0;
								des_angle = 55;//gyro_angle - 1800;
							}else if (timer < 200) {
								des_dist = -32;
								des_angle = 10;
							}else {
								des_dist = -32;
								des_angle = 0;
							}
						}else{
							if (timer < 50) {
								des_dist = 0;
								des_angle = -55;//gyro_angle - 1800;
							}else if (timer < 200) {
								des_dist = -32;
								des_angle = -10;
							}else {
								des_dist = -32;
								des_angle = 0;
							}
						}

						timer++;
					break;
				}
				
			}
					

			switch (drive_mode) {
				case drive_normal_state:
					drive_R1 = drive_R2 = temp_R_drive;
					drive_L1 = drive_L2 = temp_L_drive;
				break;

				case drive_toAngle_state:
					printf(" DRIVE TO ANGLE ");
					//printf("\r\ndes_dist: %i | des_angle: %i", des_dist, des_angle);
					temp_position_var = pid_control(&robot_dist, des_dist);
					temp_angle_var = pid_control((rout_mode == rsm_to_other_side) ? &temp_gyro_c : &Mr_Roboto, des_angle);
					//temp_angle_var = pid_control(&temp_gyro_c, des_angle);
					
					if (T_Packet_Data.pixels > 0 || rout_mode == rsm_the_angle || rout_mode == rsm_to_other_side) {
						//printf("\r\nLoop Entered|temp_position_var: %i|temp_angle_var: %i", temp_position_var, temp_angle_var);
						drive_R1 = drive_R2 = Limit_Mix(2000 + temp_position_var + temp_angle_var - 127);
						drive_L1 = drive_L2 = Limit_Mix(2000 + temp_position_var - temp_angle_var + 127);
					}else{
						drive_R1 = drive_R2 = drive_L1 = drive_L2 = 127;
					}
				break;

				case drive_none_state:
					//printf(" DRIVE NONE ");
					drive_R1 = drive_R2 = drive_L1 = drive_L2 = 127;
				break;
			}
			
			switch (arm_mode) {
				case arm_none_state:
					//printf(" ARM NONE ");
					arm_l_motor = arm_r_motor = wrist_motor = 127;
				break;

				case arm_correct_state:
					//printf(" ARM CORRECT ");
					arm_l_motor = arm_r_motor = pid_control(&arm, where_i_want_to_be - encoder_1_count);
					wrist_motor = pid_control(&wrist, desired_wrist_pos - encoder_2_count);
				break;
			}

			Generate_Pwms(pwm13,pwm14,pwm15,pwm16);
			//printf(" auto_mode: %i | auto_sel", auto_mode, auto_sel);
			printf("\r\n");
			Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
		}
		
	}
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Local_IO
* PURPOSE:       Execute user's realtime code.
* You should modify this routine by adding code which you wish to run fast.
* It will be executed every program loop, and not wait for fresh data 
* from the Operator Interface.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Local_IO(void)
{

	compressor = !pressure_switch;
  /* Add code here that you want to be executed every program loop. */
//start comment
  if(Get_ADC_Result_Count())
  {
    Process_Gyro_Data();
	
    Reset_ADC_Result_Count();
  }	
//end comment
}

/*******************************************************************************
* FUNCTION NAME: Serial_Char_Callback
* PURPOSE:       Interrupt handler for the TTL_PORT.
* CALLED FROM:   user_SerialDrv.c
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     data        unsigned char    I    Data received from the TTL_PORT
* RETURNS:       void
*******************************************************************************/

// void Serial_Char_Callback(unsigned char data)
// {
//   /* Add code to handle incomming data (remember, interrupts are still active) */
// }


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
