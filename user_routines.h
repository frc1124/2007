/*******************************************************************************
* FILE NAME: user_routines.h
*
* DESCRIPTION: 
*  This is the include file which corresponds to user_routines.c and
*  user_routines_fast.c
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __user_program_h_
#define __user_program_h_


/*******************************************************************************
                            MACRO DECLARATIONS
*******************************************************************************/
/* Add your macros (aliases and constants) here.                              */
/* Do not edit the ones in ifi_aliases.h                                      */
/* Macros are substituted in at compile time and make your code more readable */
/* as well as making it easy to change a constant value in one place, rather  */
/* than at every place it is used in your code.                               */
/*
 EXAMPLE CONSTANTS:
#define MAXIMUM_LOOPS   5
#define THE_ANSWER      42
#define TRUE            1
#define FALSE           0
#define PI_VAL          3.1415

 EXAMPLE ALIASES:
#define LIMIT_SWITCH_1  rc__int1  (Points to another macro in ifi_aliases.h)
#define MAIN_SOLENOID   solenoid1    (Points to another macro in ifi_aliases.h)
*/

/* Used in limit switch routines in user_routines.c */
#define OPEN        1     /* Limit switch is open (input is floating high). */
#define CLOSED      0     /* Limit switch is closed (input connected to ground). */


/*******************************************************************************
                            TYPEDEF DECLARATIONS
*******************************************************************************/
/* EXAMPLE DATA STRUCTURE */
/*
typedef struct
{
  unsigned int  NEW_CAPTURE_DATA:1;
  unsigned int  LAST_IN1:1;
  unsigned int  LAST_IN2:1;
  unsigned int  WHEEL_COUNTER_UP:1;
  unsigned int  :4;
  unsigned int wheel_left_counter;
  unsigned int wheel_right_counter;
} user_struct;
*/

//#define none 			pwm10
//#define ddr				pwm08
//#define ddl				pwm09

#define set_arm_pos(x,y)   {where_i_want_to_be = x; \
							desired_wrist_pos = y; }
#define explode()			Getdata(&rxdata)

//MOTORS
#define arm_l_motor		pwm03
#define arm_r_motor		pwm05
#define wrist_motor		pwm04
#define drive_L1 		pwm06
#define drive_L2 		pwm07
#define drive_R1 		pwm08
#define drive_R2 		pwm09
	
//RELAYS
#define compressor	 	relay8_fwd
#define grabber 		relay1_fwd
#define extension		relay2_fwd
#define ramp_up			relay7_fwd
#define ramp_down		relay7_rev

//SWITCHES
#define pressure_switch	rc_dig_in18
#define auto_switch_1	rc_dig_in13 //14
#define auto_switch_2	rc_dig_in14 //xx
#define auto_switch_3	rc_dig_in15 //15
#define auto_switch_4	rc_dig_in16 //16

#define auto_driveback	auto_switch_4
#define auto_goal		auto_switch_3
#define auto_direction	auto_switch_2
#define	auto_otherside	auto_switch_1

//ARM ABOUT TO SCORE POSITIONS
#define ARM_TOP		330		//362
#define WRIST_TOP	2		//44
#define ARM_MID		262		//335
#define WRIST_MID	388		//381
#define ARM_MID2	20		//79
#define WRIST_MID2	-249	//-194
#define ARM_MID3	90		//79
#define WRIST_MID3	-194	//-194
#define ARM_LOW		96		//100
#define WRIST_LOW	292		//324
#define ARM_HOME	-153	
#define WRIST_HOME	-415

#define ARM_AUTO_MID 	322
#define WRIST_AUTO_MID 	245

//ROBOT DISTANCE ANGLES
#define DIST_BACKUP 	150		//150
#define DIST_LOW_SCORE	80		//87
#define DIST_MID2_SCORE	86		//it was 83, moved to 86 to stop earlier

//STUPID
#define DIST_MID_SCORE	100		//98
#define DIST_MID3_SCORE	175		//175

//SAFETIES
#define ARM_MAX		400
#define ARM_MIN		-400
#define WRIST_MAX	400
#define WRIST_MIN	-400

//AUTONOMIC MACROS
#define move_arm(x,y)	{ 	arm_mode = arm_correct_state; \
							set_arm_pos(x, y);}
#define stop_robot()	{	arm_mode = arm_none_state; \
							drive_mode = drive_none_state; }

//AUTONOMIC MODES
#define do_other_side_mode 		0
#define rack_score_mode			1

//AUTONOMIC DRIVE STATES
#define drive_normal_state		0
#define drive_toAngle_state		1
#define	drive_toPlace_state		2
#define drive_line_state		3
#define drive_none_state		4

//AUTONOMIC ARM STATES
#define arm_none_state		0
#define	arm_correct_state	1

//RACK SCORING MODE STATES
#define rsm_to_other_side		6
#define	rsm_target_search		0
#define rsm_angle_correct		1
#define rsm_driving				2
#define rsm_scoring				3
#define rsm_driveback			4
#define rsm_the_angle			5
#define rsm_driving_far			7

//RACK SCORER MODES
#define score_top	2
#define score_mid	1
#define	score_low 	0

//auto_mode_type modes
#define billy	0
#define zach1	1
#define zach2	2

//external
extern int where_i_want_to_be;
extern int desired_wrist_pos;
extern long int desired_robot_angle;
extern int encoder_1_count;
extern int encoder_2_count;
extern long int pan_gyro_angle;

//virtuals
#define virtual_pan 	PAN_ANGLE
#define virtual_tilt	TILT_ANGLE

/*******************************************************************************
                           FUNCTION PROTOTYPES
*******************************************************************************/

/* These routines reside in user_routines.c */
void User_Initialization(void);
void Process_Data_From_Master_uP(void);
void Default_Routine(void);

/* These routines reside in user_routines_fast.c */
void InterruptHandlerLow (void);  /* DO NOT CHANGE! */
void User_Autonomous_Code(void);  /* Only in full-size FRC system. */
void Process_Data_From_Local_IO(void);
unsigned char Limit_Mix (int intermediate_value);
unsigned char flip_axis(unsigned char valueToFlip, unsigned char axisFlipOver);


#endif
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
