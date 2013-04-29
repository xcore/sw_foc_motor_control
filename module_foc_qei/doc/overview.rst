Overview
========

This module contains a Quadrature Encoder Interface (QEI) component for a Motor Control system.

It is implemented as a QEI Server and QEI Client.

The QEI Server, receives raw QEI information from the motor(s) on its input port(s), the raw QEI data is then processed, and the resulting QEI parameters are transmitted down the QEI channel to ...

The QEI Client, receives QEI parameters from the QEI channel, and places them in a data structure, ready to be used by the main Motor Control loop, test harness, or other supervisory function.

The raw QEI data consists of 4 bits. [E I B A], with the following functionality

   * Bit_3 is the error bit, it is held high(1) when there is NO error.
   * Bit_2 is the origin bit, it is low(0) most of the time, it switches to high once per revolution.
   * Bit_1 and Bit_0 are 'phase bits', the 2-bit pattern changes each time the motor turns slightly. E.g. 1024 times per revolution.

There are 2 possible pattern sequences for the phase bits, and these are used to determine the direction in which the motor is spinning:

   *        ------------------------->  Counter-Clockwise
   *	BA:  00 01 11 10 00
   *        <------------------------  Clockwise
