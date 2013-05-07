UnderStanding Test Results
==========================

Test Output
-----------

For background on the QEI protocol see the overview document in directory module_foc_qei/doc

An example of test output from a working QEI component is held in qei_results.txt

There are two columns of output.
The left hand column is the test input data.
The right hand column is the QEI output data.

   * First, Motor_0 is tested, this is indicated by a prefix of '0:'
   * Second, Motor_1 is tested, this is indicated by a prefix of '1:'

For each Motor, 
   * first the clock-wise direction is tested,
   * secondly the anti-clockwise direction is tested, and 
   * finally the error bit is tested.

For each spin direction, the following tests are done:-
   * Accelerate up to maximum speed (approx. 4000 RPM)
   * Hold steady at maximum speed, while origin detected (see below)
   * Decelerate to minimum speed, (approx. 50 RPM)

In the left hand column of the printout, is printed the value of the 4 raw QEI bits used for each test, as a number in the range [0..15]
In the right hand column of the printout, are printed the 4 QEI parameters [Error_status, Revolution_Counter, angular_Position, angular_Velocity]
 
At start-up,  the motor's origin is unknown, so the initial QEI position (P) is assumed to be zero, and P increments from zero.
During the acceleration test, the velocity(V) increases to its maximum value.
During the Maximum Speed test, the Origin-bit is tested. When the origin is detected, the angular position is reset to zero, and the rev. counter is updated.
During the deceleration test, the velocity(V) decreases to its minimum value.

Notice that the Velocity values do NOT appear to update immediately; this is due to filtering. The QEI values can be unreliable, therefore the following filter is used. At start-up, or when the spin direction changes, 3 consecutive movements in the same direction are required before a new spin direction is established.

When both clock-wise and anti-clock wise tests have been done. The Error-bit is tested. Three consecutive detections of a low(0) raw Error-bit are required before the Error_status parameter flag is switched on(1).

The receipt of the Error_status flag by the QEI client is used by the test harness to terminate testing of the current motor, and switch to a different motor (or end the program).
