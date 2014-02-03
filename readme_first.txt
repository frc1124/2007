****************************************************************

You are free to use this source code for any non-commercial
use. Please do not make copies of this source code, modified
or un-modified, publicly available on the internet or
elsewhere without permission. Thanks.

Copyright ©2005-2007 R. Kevin Watson. All rights are reserved.

****************************************************************

If you have any problems with the camera, please visit the CMUcam2 
Frequently-Asked-Questions document at http://kevin.org/frc/camera.

This project contains version 2.0 of the "streamlined" edition of
the IFI default code with CMUcam2 support. Differences from 1.0 are:

tracking.c/.h:
  Provided two new functions to set the pan and tilt servo position.
  This was done to provide a level of indirection so that the servos 
  could be controlled from the robot controller or the CMUcam2.

  Fixed bug in search initialization code where temp_pan_servo was
  initialized to zero instead of Tracking_Config_Data.Pan_Min_PWM.

  Altered tracking algorithm to use the t-packet confidence value to
  determine whether the code should track or search.

  Added Get_Tracking_State() function, which can be used to determine 
  if the camera is pointing at the target.

terminal.c:
  Added "no camera data" diagnostic information.

user_routines.c:
  Added code to send tracking state information to the user LEDs on
  the operator interface.


1/16/07 changes made for version 2.1:

Added IFI's new readme.txt file (replacing 2006 version).
Added IFI's new library files (replacing older versions).
Added IFI's new linker script files (replacing older versions).
Added IFI's new FRC_MASTER_V13.BIN file.
Added IFI's new LibReadMe.txt file.
Added IFI's new Using_a_2005_FRC.txt file (replacing older Using_Last_Years_FRC.txt).
Added new hardware-based PWM code (see pwm_readme.txt).
Added CMUCam2_commands.pdf file
Added CMUCam2_data_packets.pdf file
Updated user_routines.c/Default_Routine() with 2007 version.
Added Virtual_Window() function to camera.c/.h.

  
