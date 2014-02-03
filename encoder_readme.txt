****************************************************************

You are free to use this source code for any non-commercial
use. Please do not make copies of this source code, modified
or un-modified, publicly available on the internet or
elsewhere without permission. Thanks.

Copyright ©2004-2006 R. Kevin Watson. All rights are reserved.

****************************************************************

This source code in this file implements an interface up to
six quadrature output encoders. Used with suitable PID control 
software, encoders can be used to control the position and 
velocity of of your robot. In general, for velocity control
with low count rates (~100/s), the encoder channels are 
interchangeable and it's anticipated that teams won't need 
all six inputs and will cut and paste the code as needed. For 
high count rates or position control, the channels are
optimized for specific applications and are not interchangable.

Encoder channels one and two are optimized for velocity control
and will generate the least number of interrupts per encoder 
count (one per encoder count). These channels may have problems
in position control applications because they can be fooled 
into thinking that the encoder shaft is rotating if the shaft 
happens to stop very near a phase-A transition and then wobbles 
back and forth across that transition.

Encoder channels three and four are optimized for position
control. For these channels, software examines both transitions
of phase-A and can't be fooled into miscounting the number
of encoder counts. The downside to using these channels is that
for a given encoder, they will generate twice the number of
interrupts as channels one and two (two per encoder count).

Encoder channels five and six are just like channels three and
four, but offer the bonus of increasing the precision of your
encoder by a factor of two for free. Unlike channels one 
through four, which will increment or decrement on each rising 
(zero to one) transition, these two channels will increment or
decrement on both transitions of phase-A. In other words, if
you attach a 64 count-per-revolution encoder to one of these
two channels, you'll read 128 counts when you rotate the shaft
exactly one revolution.


This software was tested with Grayhill 63R256 and 61K128 
quadrature output optical encoders. Data sheets for these 
devices are included.								

This source code will work with the PIC18F8520-based FIRST 
Robotics robot controller, the PIC18F8722-based FIRST Robotics
robot controller, and the Robovation/EDU robot controller.

                    ** IMPORTANT **

On a 40MHz PIC18Fxxx, this software can track peak encoder 
count rates as high as a few thousand counts per second, which
should be more than adequate for most applications. To meet
your performance expectations, selecting the proper Counts Per 
Revolution (CPR) parameter of your encoder is very important.
If the CPR is too high, the robot controller will spend too
much time counting encoder "ticks" and not enough time on
other tasks. At the extreme, you will see very wacky behavior
in your robot controller including corrupted data, the red-
light-of-death or the controller may even think the robot is
traveling in a direction that it isn't. Selecting a CPR that
is too low will not give you the resolution you desire. The
CPR should be optimized to minimize the number of interrupts
your robot controller will have to service yet meet your
resolution expectations (yes, millimeter position resolution
to too much to ask for).

Another potential problem with high count rates is that
your encoder probably won't have enough drive capability
to send the phase-A and phase-B signals though your cables
without signal degradation, which can cause all kinds of
problems. If you notice that your encoder counts just fine
at low count rates, but exhibits wacky behavior at higher
count rates, you'll probably need to build a line driver
circuit to provide more current drive. An integrated circuit
that can provide this added drive is the 74ACT244 octal
line driver. Mount the circuit close to the encoder.

The included project files were built with MPLAB version 7.20.
If your version of MPLAB complains about the project version, 
the best thing to do is just create a new project with MPLAB's 
project wizard. Include every file except: FRC_alltimers.lib 
and ifi_alltimers.lib and you should be able to build the code.

Support for this software can be received by posting a help
request in the programming section of the Chief Delphi forums
(http://www.chiefdelphi.com).

This file is best viewed with tabs set to four characters.

****************************************************************

Here's a description of the functions in encoder.c:


Initialize_Encoders()

This function initializes the encoder software. It should be
called from user_routines.c/User_Initialization().
 

Get_Encoder_n_Count()

These functions will return the current number of encoder
counts or "ticks" for encoder number n, where n is a number
between one and six.


Reset_Encoder_n_Count()

This function can be used to reset individual encoder counts
to zero.


Encoder_n_Int_Handler()
This function is automatically called by the microcontroller
when the phase-A signal of an encoder transitions from zero to 
one, and in the case of encoders three through six, one to a 
zero. You shouldn't have to call these functions yourself.

****************************************************************

Nine things must be done before this software will work 
correctly on the FRC-RC:

1) Each encoder's phase-A output is wired to one of the six
available interrupt inputs on digital inputs one through six.
Encoder one is wired to digital input one, encoder two is
wired to digital input two, etc.

2) Each encoder's phase-B output is wired to any free digital
input. By default, encoder one is wired to digital input
eleven, encoder two is wired to digital input twelve, ...,
encoder six is wired to digital input sixteen. These default
assignments can be changed by editing encoder.h

3) You must add the encoder.c/.h source files to your MPLAB 
project.

4) Disable encoders not needed for your design by following 
the instructions in encoder.h. By default, all six encoders
are enabled.

5) Digital I/O pins used in step 2 above must be declared as
an input in user_routines.c/User_Initialization(). If you 
notice an encoder that only counts in one direction, you 
forgot to do this step.

6) The encoder interrupt handlers must be installed in
User_Routines_Fast.c/InterruptHandlerLow(). See the included 
copy of User_Routines_Fast.c to see how this is done.

7) A #include statement for the encoder.h header file must be 
included at the beginning of each source file that calls the 
encoder functions. The statement should look like this: 
#include "encoder.h".

8) Initialize_Encoders() must be called from user_routines.c/
User_Initialization().

9) The p18f8722.h header file included with the C18 2.4
compiler may have an error that will prevent you from
compiling this software. If you get an error like:

...Error [1205] unknown member 'INT3IP'...

You'll need to replace your copy of p18f8722.h with the
version included with this project. If you installed your
compiler using the default path, this file should be located
at c:\mcc18\h.


Nine things must be done before this software will work 
correctly on the EDU-RC:

1) Each encoder's phase-A output is wired to one of the six
available interrupt inputs. Encoder one is wired to interrupt
one, encoder two is wired to interrupt two, etc.

2) Each encoder's phase-B output is wired to any free digital
input. By default, encoder one is wired to digital input
eleven, encoder two is wired to digital input twelve, ...,
encoder six is wired to digital input sixteen. These default
assignments can be changed by editing encoder.h.

3) You must add the encoder.c/.h source files to your MPLAB 
project.

4) Disable encoders not needed for your design by following 
the instructions in encoder.h. By default, all six encoders
are enabled.

5) Digital I/O pins used in step 2 above must be declared as
an input in user_routines.c/User_Initialization(). If you 
notice an encoder that only counts in one direction, you 
forgot to do this step.

6) The encoder interrupt handlers must be installed in
User_Routines_Fast.c/InterruptHandlerLow(). See the included 
copy of User_Routines_Fast.c to see how this is done.

7) A #include statement for the encoder.h header file must be 
included at the beginning of each source file that calls the 
encoder functions. The statement should look like this: 
#include "encoder.h".

8) You'll need to comment out the #include <p18f8722.h> line 
near the top of encoder.c and then uncomment the #include 
<p18f8520.h> line below it.

9) Initialize_Encoders() must be called from user_routines.c/
User_Initialization().

Kevin Watson
kevinw@jpl.nasa.gov
