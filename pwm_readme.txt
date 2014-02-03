***************************************************************

You are free to use this source code for any noncommercial
use. Please do not make copies of this source code, modified
or unmodified, publicly available on the internet or
elsewhere without permission. Thanks.

WARNING: This is experimental software and the author cannot
assume any responsibility for its use (or misuse). If you do 
not know how to prevent your robots motors from accidentally 
running away, please do not use this software.

Copyright ©2006-2007 R. Kevin Watson. All rights are reserved.

***************************************************************

The source code in pwm.c/.h contains software to replace
IFI's software loop-based Generate_Pwms() function, which is
responsible for generating PWM pulses on PWM output pins
13 through 16 on the FRC robot controller. The advantages of
using this implementation are:

The PWM outputs driven by Generate_Pwms() would jitter when
interrupts are firing-off in the background. This would
sometimes cause your servos to twitch or motors to change
velocity. PWM() fixes this problem by using the CCP hardware 
within the robot controllers CPU to precisely control the 
PWM pulse width without software intervention.

In addition, the center point and gain of each PWM channel is 
completely programmable with a resolution of 100 ns. This is a 
cool feature because it allows you to do things like map the 
entire 0 to 255 range to a very narrow range of servo travel 
for greater position accuracy, or to a narrower range of 
velocities from your motor. 

This source code will work with the PIC18F8520-based FIRST 
Robotics robot controller used in 2004/2005 and the PIC18F8722-
based robot controller used since 2006.


***************************************************************

Here's a description of the functions in pwm.c:

Initialize_PWM()

This function initializes the PWM software and must be called
before PWM() can be called. The ideal place to call this 
function is user_routines.c/User_Initialization().

PWM()

This function, when using the default operating parameters
found in pwm.h, behaves exactly like Generate_Pwms(). Each
time this function is called, one and only one pulse is
generated, just like Generate_Pwms(). If called from
Process_Data_From_Master_uP(), the update rate is just over
thirty-eight hertz. PWM() can be called from other sections
of your code for higher control rates, but keep in mind that
servos won't handle a rate much higher than that thirty-eight
hertz. On the other hand, IFI's Victors can handle control
rates of at least one-hundred hertz, which is nice for
applications like position control.

***************************************************************

Five things must be done before this software will work 
correctly:

  1) You must add the pwm.c/.h source files to your build
  directory and your MPLAB project.

  2) All arguments to the function Setup_PWM_Output_Type(),
  which can be found in user_routines.c/User_Initialization(),
  must be changed from "IFI_PWM" to "USER_CCP".

  3) As this software is intended to replace IFI's own
  PWM driver, the Generate_Pwms() function located in 
  user_routines.c/Process_Data_From_Master_uP() should be 
  removed or commented out.

  4) A #include statement for the pwm.h header file must 
  be included at the beginning of each source file that 
  calls the PWM() function. The statement should look like 
  this: #include "pwm.h".

  5) The Initialize_PWM() function must be called from 
  the User_Initialization() function located in the 
  user_routines.c source file.

  6) This software uses timer 3. If you're currently using
  this resource for another reason, you will need to modify
  this code to use timer 1, or use a different timer for
  your own code.


Kevin Watson
kevinw@jpl.nasa.gov
