Overview
========

This module contains a Quadrature Encoder Interface (QEI) component for Motor Control systems.

The QEI Server runs in its own logical core and receives raw QEI information from one or more motors on its input ports which is then processed and the resulting QEI parameters are transmitted down the QEI channel to the QEI Client. The client places the data in a data structure, ready to be used by a main Motor Control loop, test harness, or other supervisory function.

The raw QEI data consists of 4 bits. [E I B A], with the following functionality

   * Bit_3 is the error bit, it is held high(1) when there is no error.
   * Bit_2 is the origin bit, it is low(0) most of the time, it switches to high once per revolution.	Note well, when the motor starts, it is NOT normally at the origin.
   * Bit_1 and Bit_0 are 'phase bits', the 2-bit pattern changes each time the motor turns slightly. E.g. 1024 times per revolution.

WARNING: By convention, Phase_A is normally the most significant bit of the pair. However on the XMOS motor control board, Phase_A has been connected to the least significant bit.
 
There are 2 possible pattern sequences for the phase bits, and these are used to determine the direction in which the motor is spinning. By convention, the positive spin direction is the one where Phase_A leads Phase_B.	E.g. Phase_A goes high, one bit-change earlier than Phase_B goes high. This definition is based on time, and is therefore NOT dependent on the spatial orientation of the motor.

::
   ------------------------>  Positive Spin
   BA:  00 01 11 10 00
   <------------------------  Negative Spin

WARNING: Each motor manufacturer may use their own definition for spin direction.

