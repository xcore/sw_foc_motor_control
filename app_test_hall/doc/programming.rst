Hall Sensor Test Application Programming Overview
=================================================

.. _test_hall_Programming:

This file should be read in conjunction with the Quick-Start guide for the Hall Test-bench

The generator runs through a set of tests, these are specified formally as a *test vector* and transmitted to the test checker. For each test the generator creates the appropriate Hall raw-data and drives this onto the output pins. The Hall Server recognises changes on its input pins, processes the new raw-data, and updates the Hall parameters (Hall_Sensor_value and Error_Status). The test checker reads the specification in the received test vector, then polls the Hall Client for parameters. These parameters are checked for correctness against the test vector specification.

The following tests are always performed

   #. A *Phase_Value* test: the 3 phase values should form a valid combination

The following tests are optional

   #. Select which Motor to test: Motor_0 or Motor_1
   #. A *Spin Direction* test: the phase values should change in the required order
   #. An *Error Status* test: the error-status flag should be raised when 3 consecutive error-bits are detected.

The options are selected by editing the flags in the file hall_tests.txt


How To Instrument the Code to Use ``xSCOPE`` 
--------------------------------------------

In order to instrument code to use xSCOPE the following actions are required. (For this application they have already been done) :-

   #. In the ``Makefile`` the option ``-fxscope`` needs to be added to the ``XCC`` flags.
   #. In the ``xC`` files that use xSCOPE functions, the header file <xscope.h> needs to be included.
   #. In the ``main.xc`` file, the xSCOPE initialisation function xscope_user_init() needs to be added.
   #. In each ``xC`` file that uses xSCOPE to plot variables, one or more xSCOPE capture functions are required.

The above requirements are discussed in more detail below in the section ``Look at the Code``. 


Look at the Code
----------------

The steps below are designed to guide an initial understanding of how the testbench is constructed. More detail on the testbench structure can also be found in the section below (``Testbench Structure``).

   #. Examine the application code. In xTIMEcomposer, navigate to the ``src`` directory under ``app_test_hall``  and double click on the ``main.xc`` file within it. The file will open in the central editor window.
   #. Find the ``main.xc`` file and note that main() runs 3 cores (processes) in parallel. All cores run on the same tile at a reference frequency of 100 MHz.
      * ``gen_all_hall_test_data()`` Generates test vectors and test data. The test vectors are transmitted using channel ``c_gen_chk`` to the Checker core. The test data is output on the 32-bit buffered test output port (``p4_tst``).
      * ``foc_hall_do_multiple()`` is the Hall sensor Server, receiving test data on the 4-bit Hall sensor port (``p4_hall``), processes the data, and transmitting output data over channel ``c_hall_chk``
      * ``check_all_hall_client_data()`` contains the Hall sensor Client which receives Hall sensor output data over channel ``c_hall_chk``, and displays the results. ``gen_all_hall_test_data()`` and ``check_all_hall_client_data()`` both produce display information in parallel. The other 2 functions in ``main.xc`` are ``init_locks()`` and ``free_locks()``. These are used control a MutEx which only allows one core at a time to print to the display.
      * As well as main(), there is a function called xscope_user_init(), this is called before main to initialise xSCOPE capability. In here are registered the 3 Hall signals that were described above, and seen in the xSCOPE viewer.
   #. Find the ``app_global.h`` header. At the top are the xSCOPE definitions, followed by the motor definitions, and then the Hall Sensor definitions, which are specific to the type of motor being used and are currently set up for the LDO motors supplied with the development kit.
   #. Note in ``app_global.h`` the define PRINT_TST_HALL used to switch on verbose printing. An example of this can be found in file ``hall_results.txt``.
   #. Find the file ``check_hall_tests.xc``. In here the function ``check_motor_hall_client_data()`` handles the Hall sensor output data for one motor. In the 'while loop' is a function ``foc_hall_get_parameters()``. This is the Hall sensor Client. It communicates with the Hall sensor server function ``foc_hall_do_multiple()`` via channel ``c_hall_chk``. The 'while loop' is paced to request Hall sensor data over the ``c_hall_chk`` channel every 40 micro-seconds. This is typical of the issue rate when using real hardware.
   #. Find the file ``hall_server.xc`` in the Hall Sensor module directory (``module_foc_hall``). Near the end of function ``foc_hall_do_multiple()`` are the xSCOPE instructions used to capture the signals seen in the xSCOPE viewer. Signal capture occurs each time a change on the input pins is detected.

Testbench Structure
-------------------

The test application uses 3 cores containing the following components
   #. A test-vector and Hall raw-data generator
   #. The Hall Server under test
   #. The Hall Client under test, and the test results checker

The test application uses 2 channels for the following data
   #. Transmission of test vectors between the Generator and Checker cores
   #. Transmission of Hall parameters between Server and Client cores

The test application uses 2 ports for the following data
   #. A 4-bit output port for transmission of generated Hall raw-data
   #. A 4-bit input port for the Hall server to receive Hall raw-data

The output pins driven by the generator are looped back to the Hall Server input pins using the *loopback plugin* functionality included within the xSIM simulator, which allows arbitrary definition of pin level loopbacks.

