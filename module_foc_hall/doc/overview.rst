Overview
========

This module contains a Hall-sensor interface component for Motor Control systems.

The Hall Server runs in its own logical core and receives raw Hall-sensor information from one or more motors on its input ports. This is then processed and the resulting Hall parameters are transmitted down the Hall channel to the Hall Client. The client places the data in a data structure, ready to be used by a main Motor Control loop, test harness, or other supervisory function.

The raw Hall data consists of 4 bits. [E C B A], with the following functionality

   * Bit_3 is the error bit, it is held high(1) when there is NO error.
   * Bit_2, Bit_1 and Bit_0 are 'phase bits', the 3-bit pattern changes each time the motor turns slightly. E.g. 24 times per revolution.

WARNING: By Convention Phase_A is the most significant bit. However on the XMOS Motor boards, Phase_A is the least significant bit.

There are 2 possible pattern sequences for the phase bits, and these are used to determine the direction in which the motor is spinning. Note well, 2 patterns (000, 111) are never used. By convention, the positive spin direction is the one where Phase_A leads Phase_B.	E.g. Phase_A goes high, one bit-change earlier than Phase_B goes high. This definition is based on time, and is therefore NOT dependent on the spatial orientation of the motor.

::

         -------------------------->  Negative Spin
   CBA:  001 101 100 110 010 011 001
         <--------------------------  Positive Spin

WARNING: Each motor manufacturer may use their own definition for spin direction.
