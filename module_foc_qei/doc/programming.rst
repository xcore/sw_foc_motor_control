Programming Guide
=================

Key Files
---------

   * ``qei_client.xc``: Contains 'XC' function library for QEI Client
   * ``qei_server.xc``: Contains 'XC' function library for QEI Server

Usage
-----

There following 2 functions are designed to be called from an XC file.

   * ``foc_qei_get_data()`` Client function designed to be called from an XC file each time a new set of QEI data is required.
   * ``foc_qei_do_multiple()``, Server function designed to be called from an XC file. It runs on its own core, and receives data from all QEI motor ports.

To set the filter parameters, edit the following defines in the include file module_dsp_biquad/src/biquad_simple.h

   * QEI_PER_REV  // No of QEI positions per Revolution
   * QEI_PHASE_MASK // Bit Mask for [B A] phase info.
   * QEI_ORIG_MASK // Bit Mask for origin bit.
   * QEI_NERR_MASK // Bit Mask for error status bit (1 == No Errors)
   * PLATFORM_REFERENCE_HZ // Platform Reference Frequency (250 MHz)
   * QEI_FILTER // QEI filter switch (0 == Off)
   * MAX_SPEC_RPM // Maximium specified motor speed

Test Applications
=================

Quadrature Encoder Interface (QEI) Xcore Simulator
--------------------------------------------------

To get started with this application, run through the instructions in the quickstart guide.

This application uses module_foc_qei to process simulated QEI input test data.
The QEI input data is received on a 4-bit port.
The QEI outputs (motor position and velocity) are transmitted in a QEI data structure.

Makefile
........

The Makefile is found in the top level directory of the application (e.g. app_test_qei)

The application is for the simulator. 
However the platform being simulated is a Motor control board.
The Makefile TARGET variable needs to be set to Motor control board being used.
E.g. If the platform configuration file is XP-MC-CTRL-L2.xn, then
TARGET = XP-MC-CTRL-L2

The maximum number of motors supported in currently 2, this is set in app_global.h: e.g.
#define NUMBER_OF_MOTORS 2

Running the application with the Command Line Tools
...................................................

Move to the top level directory of the application (e.g. app_test_qei), and type

   * xmake clean
   * xmake all

To start the test type

   * xsim --plugin LoopbackPort.dll "-port tile[1] XS1_PORT_4B 4 0 -port tile[1] XS1_PORT_4F 4 0 -port tile[1] XS1_PORT_4A 4 0 -port tile[1] XS1_PORT_4E 4 0" bin/app_test_qei.xe

Test results will be printed to standard-out.
Remember this is a simulator, and is very slow.
There may be gaps of upto 1 minute between ouput lines.
The whole test takes upto 10 minutes to run.

An example of test output is shown in qei_results.txt.
There are two columns of output.
The left hand column is the test input data.
The right hand column is the QEI output data.

First Motor_0 is tested, this is indicated by a prefix of '0:'
Second Motor_1 is tested, this is indicated by a prefix of '1:'

For each Motor, first the clock-wise direction is tested,
secondly the anti-clockwise direction is tested.

For each spin direction, the following tests are done:-
1) Accelerate upto maximum speed (approx. 4000 RPM)
2) Hold steady at maximum speed, while origin detected (see below)
3) Decelerate to minimum speed, (approx. 50 RPM)

Initally the motor's origin is unknown, 
but the initial QEI position (A) is assumed to be zero.
When the motor origin is detected (QEI & 4) = 1

Trouble-shooting
................

During initialisation, and when the system is reconfigured, 
there may be audible clicks in the audio. This is expected.

The filter may produce output audio that is louder than the input,
especially when high value for Quality_Factor are selected.
This in turn may produce audible distortion in the output audio.
If this occurs, try one of the following:

   * Turning down the volume of the audio source (i.e. the volume level input to the application), 
   * Reduce the value of DEF_QUAL_FACT
