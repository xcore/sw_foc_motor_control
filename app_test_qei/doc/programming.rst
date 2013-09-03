Quadrature Encoder (QEI) Test Application Programming Overview
==============================================================

.. _test_qei_Programming:

This file should be read in conjunction with the Quick-Start guide for the QEI Test-bench

The generator runs through a set of tests, these are specified formally as a *test vector* and transmitted to the test checker. For each test the generator creates the appropriate QEI raw-data and drives this onto the output pins. The QEI Server recognises changes on its input pins, processes the new raw-data, and updates the QEI parameters (Velocity, Direction, etc). The test checker reads the specification in the received test vector, then polls the QEI Client for parameters. These parameters are checked for correctness against the test vector specification.

The following tests are always performed

   #. A *Fast* test: the speed should be fast and steady
   #. A *Slow* test: the speed should be slow and steady
   #. An *Acceleration* test: the speed should increase
   #. A *Deceleration* test: the speed should decrease

The following tests are optional

   #. Select which Motor to test: Motor_0 or Motor_1
   #. A *Spin Direction* test: the spin direction bit should be correctly set
   #. An *Error Status* test: the error-status flag should be raised when 3 consecutive error-bits are detected.

The options are selected by editing the flags in the file qei_tests.txt


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

   #. Examine the application code. In xTIMEcomposer, navigate to the ``src`` directory under ``app_test_qei``  and double click on the ``main.xc`` file within it. The file will open in the central editor window.
   #. Review the ``main.xc`` and note that main() runs 4 tasks on 4 logical cores in parallel. All cores run on the same tile at a reference frequency of 100 MHz.

         * ``gen_all_qei_test_data()`` generates test vectors and test data. The test vectors are transmitted using channel ``c_gen_chk`` to the Checker core. The test data is output on the 32-bit buffered test output port (``p4_tst``).
         * ``disp_gen_data()`` Accepts raw QEI data values over a channel (``c_gen_dis``), formats them, and then prints them.
         * ``foc_qei_do_multiple()`` is the QEI Server, receiving test data on the 4-bit buffered QEI port (``pb4_qei``), processes the data, and transmitting output data over channel ``c_qei_chk``
         * ``check_all_qei_client_data()`` contains the QEI Client which receives QEI output parameters over channel ``c_qei_chk``, checks the QEI parameters, and displays the results. ``gen_all_qei_test_data()`` and ``check_all_qei_client_data()`` both produce display information in parallel. 
         * The other 2 functions in ``main.xc`` are ``init_locks()`` and ``free_locks()``. These are used control a MutEx which allows only one core at a time to print to the display.
         * As well as main(), there is a function called xscope_user_init(), this is called before main to initialise xSCOPE capability. In here are registered the 4 QEI signals that were described above, and seen in the xSCOPE viewer.

   #. Find the ``app_global.h`` header. At the top are the xSCOPE definitions, followed by the motor definitions, and then the QEI definitions, which are specific to the type of motor being used and are currently set up for the LDO motors supplied with the development kit.
   #. Note in ``app_global.h`` the define VERBOSE_PRINT used to switch on verbose printing. WARNING. Currently, the use of verbose printing breaks timing, and some tests will fail.
   #. Find the file ``check_qei_tests.xc``. In here the function ``check_motor_qei_client_data()`` handles the QEI output data for one motor. In the 'while loop' is a function ``foc_qei_get_parameters()``. This is the QEI Client. It communicates with the QEI server function ``foc_qei_do_multiple()`` via channel ``c_qei``. The 'while loop' is paced to request QEI data over the ``c_qei`` channel every 40 micro-seconds. This is typical of the issue rate when using real hardware.  Directly after ``foc_qei_get_parameters()`` are the xSCOPE functions which allow the QEI values to be captured.
   #. Now that the application has been run with the default settings, you could try selecting the QEI filter by setting ``#define QEI_FILTER 1`` in the app_global.h file This selects a low-pass filter that smooths out changes in velocity values. Make this change and then rebuild and rerun the simulation. The test harness will now report many speed/spin failures due to the filtering applied. To get more information on which tests are failing select ``#define PRINT_TST_QEI 0`` in app_global.h. Make this change and then rebuild and rerun the simulation.
   #. To further explore the capabilities of the simulator, find the items under ``XMOS Examples:Simulator`` in the xSOFTip browser pane. Drag one of them into the Project Explorer to get started.

Testbench Structure
-------------------

The test application uses 4 cores containing the following components

   #. A test-vector and QEI raw-data generator
   #. The QEI value display function
   #. The QEI Server under test
   #. The QEI Client under test, and the test results checker

The test application uses 3 channels for the following data

   #. Transmission of test vectors between the Generator and Checker cores
   #. Transmission of raw QEI values between the Generator and Display cores
   #. Transmission of QEI parameters between Server and Client cores

The test application uses 2 ports for the following data

   #. A 4-bit output port for transmission of generated QEI raw-data
   #. A 4-bit input port for the QEI server to receive QEI raw-data

The output pins driven by the generator are looped back to the QEI Server input pins using the *loopback plugin* functionality included within the xSIM simulator, which allows arbitrary definition of pin level loopbacks.

