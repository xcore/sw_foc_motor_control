Programming Guide
=================

Key Files
---------

   * ``adc_client.xc``: Contains the xC implementation of the ADC Client API
   * ``adc_server.xc``: Contains the xC implementation of the ADC Server task

Usage
-----

The following 2 functions are designed to be called from an xC file.

   * ``foc_adc_get_parameters()`` Client function designed to be called from an xC file each time a new set of ADC parameters are required.
   * ``foc_adc_7265_triggered()``, Server function designed to be called from an xC file. It runs on its own core, and receives data from one ADC_7265 chip. The chip multiplexes together the data from all motors.

The following ADC definitions are required. These are set in ``app_global.h``

   * PWM_RES_BITS 12 // Number of bits used to define number of different PWM pulse-widths
   * PLATFORM_REFERENCE_HZ // Platform Reference Frequency
   * ADC_FILTER // ADC filter switch (0 == Off)
   * MAX_SPEC_RPM // Maximium specified motor speed

Test Applications
-----------------

This module has a test harness called ``Analogue-to-Digital Converter (ADC) Test Harness`` which can be found in the xSOFTip Explorer pane.

To get started with this application, run through the instructions in the quickstart guide for the test harness. More details about using this test harness are given below.

Makefile
........

The Makefile is found in the top level directory of the application (e.g. app_test_adc)

The application is for the simulator. 
However the platform being simulated is a Motor control board.
The Makefile TARGET variable needs to be set to the Motor control board being used.
E.g. If the platform configuration file is XP-MC-CTRL-L2.xn, then
TARGET = XP-MC-CTRL-L2

The maximum number of motors supported in currently 2, this is set in app_global.h: e.g.
#define NUMBER_OF_MOTORS 2

Running the application with the Command Line Tools
...................................................

Move to the top level directory of the application (e.g. app_test_adc), and type

   * xmake clean
   * xmake all

To start the test type

   * xsim --plugin LoopbackPort.dll "-port tile[1] XS1_PORT_1G 1 0 -port tile[1] XS1_PORT_1C 1 0 -port tile[1] XS1_PORT_1H 1 0 -port tile[1] XS1_PORT_1D 1 0 -port tile[1] XS1_PORT_1I 1 0 -port tile[1] XS1_PORT_1E 1 0 -port tile[1] XS1_PORT_1J 1 0 -port tile[1] XS1_PORT_1F 1 0" bin/app_test_adc.xe

Test results will be printed to standard-out.
Remember this is a simulator, and is very slow.
The default test takes about a minute to run.
The 'slow' test make take over an hour.

For a explanation of the test results refer to the quickstart guide in doc_quickstart/adc/index.rst

Trouble-shooting
................

**The information in the 'check results' column may disappear**: This and almost any other problem are probably due to not setting the port configuration correctly when calling xSIM.

**The printout may stop** Depending on the speed of your PC (or Mac), there can be upto 1 minute gap between printed lines.
