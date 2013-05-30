Programming Guide
=================

Key Files
---------

   * ``qei_client.xc``: Contains the XC implementation of the QEI Client API
   * ``qei_server.xc``: Contains the XC implementation of the QEI Server task

Usage
-----

The following 2 functions are designed to be called from an XC file.

   * ``foc_qei_get_parameters()`` Client function designed to be called from an XC file each time a new set of QEI parameters are required.
   * ``foc_qei_do_multiple()``, Server function designed to be called from an XC file. It runs on its own core, and receives data from all QEI motor ports.

The following QEI definitions are required. These are set in ``qei_common.h`` or ``app_global.h``

   * QEI_PER_REV  // No of QEI positions per Revolution
   * QEI_PHASE_MASK // Bit Mask for [B A] phase info.
   * QEI_ORIG_MASK // Bit Mask for origin bit.
   * QEI_NERR_MASK // Bit Mask for error status bit (1 == No Errors)
   * PLATFORM_REFERENCE_HZ // Platform Reference Frequency
   * QEI_FILTER // QEI filter switch (0 == Off)
   * MAX_SPEC_RPM // Maximium specified motor speed

Test Applications
=================

Quadrature Encoder Interface (QEI) Xcore Simulator
--------------------------------------------------

To get started with this application, run through the instructions in the quickstart guide.
The application is in app_test_qei
The quickstart guide is in doc_quickstart/qei/index.rst

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
There may be gaps of upto 1 minute between each printed line.
The whole test takes upto 10 minutes to run.

For a explanation of the test results refer to the quickstart guide in doc_quickstart/qei/index.rst

Trouble-shooting
................

The information in the 'check results' column may disappear.
This and almost any other problem are probably due to NOT setting the port configuration correctly when calling xsim

The printout may stop.
As mentioned above, depending on the speed of your PC (or Mac), there can be upto 1 minute gap between printed lines.
