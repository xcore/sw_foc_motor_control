Quadrature Encoder (QEI) Simulator Testbench
============================================

.. _test_qei_Quickstart:

This application is an xSIM test harness for the quadrature encoder interface using xTIMEcomposer Studio. It tests the QEI functions in the ``Quadrature Encoder Interface (QEI) Component`` xSOFTip component and directs test results to STDOUT.

The application runs the QEI component in one logical core, and generates stimulus for the components pins from a second logical core. The pins driven by the stimulus generator are looped back to the QEI component pins using the *loopback plugin* functionality included within the xSIM simulator, which allows arbitrary definition of pin level loopbacks. 

No hardware is required to run it.

Import and Build the Application
--------------------------------

   1. Open xTIMEcomposer and check that it is operating in online mode. Open the edit perspective (Window->Open Perspective->XMOS Edit).
   #. Locate the ``Quadrature Encoder Interface Test Harness`` item in the xSOFTip pane on the bottom left of the window and drag it into the Project Explorer window in the xTIMEcomposer. This will also cause the modules on which this application depends to be imported as well. These modules are: ``module_foc_qei``, and ``module_locks``.
   #. Click on the app_test_qei item in the Explorer pane then click on the build icon (hammer) in xTIMEcomposer. 
   #. Check the console window to verify that the application has built successfully. 

For help in using xTIMEcomposer, try the xTIMEcomposer tutorial, that can be found by selecting Help->Tutorials from the xTIMEcomposer menu.

Note that the Developer Column in the xTIMEcomposer on the right hand side of your screen 
provides information on the xSOFTip components you are using. 
Select the ``module_foc_qei`` component in the Project Explorer, and you will see its description together with API documentation. 
Having done this, click the ``back`` icon until you return to this quickstart guide within the Developer Column.

Configure And Run The Simulator
-------------------------------

   #. Double click ``app_test_qei`` in the left hand ``Project Explorer`` window.
   #. Click on the arrow next to the ``Run`` icon (the white arrow in the green circle) in the top menu bar. Select ``Run Configurations``
   #. In ``Run Configurations`` window, double click on ``xCORE Application``.
   #. You should find that the left hand side of the ``Run Configurations`` window, should be populated with details from the ``app_test_qei`` project. If the details are blank, this is probably because the project was not selected correctly in the first step. If this has happened, and the problem persists, browse to the correct project, and select the executable.
   #. Select the ``run on simulator`` button.
   #. Now setup the loopbacks between the stimulus generator and the
      QEI component.

      #. Select the ``Simulator`` tab.
      #. Select the ``Loopback`` tab.
      #. Click ``Enable pin connections``.
      #. Click ``Add`` and dialogue boxes will appear for Tile, Port, Offset and Width. These should be filled in with the following information and steps shown in the table below. The second time the simulator is run, it is only necessary to click on the ``Run`` icon (the white arrow in the green circle) in the top menu.

                +-------+--------+------------+-------+------+
                | From: |    1   | XS1_PORT_4B|   0   |   4  |
                +-------+--------+------------+-------+------+
                | To:   |    1   | XS1_PORT_4F|   0   |   4  |
                +-------+--------+------------+-------+------+

      #. Click ``Add`` again and then do the below

                +-------+--------+------------+-------+------+
                | From: |    1   | XS1_PORT_4A|   0   |   4  |
                +-------+--------+------------+-------+------+
                | To:   |    1   | XS1_PORT_4E|   0   |   4  |
                +-------+--------+------------+-------+------+

      #. Click ``Apply``
      #. Click ``Run``


Test Results 
------------

After a few seconds, test results will start to appear in the console
window, and note that there may be pauses of upto 1 minute in console
output. The test lasts upto 10 minutes. It is completed when the
following 2 messages have appeared::

   Test Generation Ends       
   Test Check Ends


For background on the QEI protocol see the overview document in directory module_foc_qei/doc

An example of working test output from a working QEI component can be found in a file named ``qei_results.txt``

Data in to the left is the test input data is printed the value of the 4 raw QEI bits used for each test, as a number in the range [0..15] corresponding to the A/B lines of the encoder, plus a digit showing which of the two QEI interface is being tested::

  0:QEI:11

The above means that a decimal input value of 11 is applied to the 4 bit port used for the first (0th) QEI interface. First, Motor_0 is tested, this is indicated by a prefix of '0:' as above. Second, Motor_1 is tested, this is indicated by a prefix of '1:'

For each Motor, 
   * first the clock-wise direction is tested,
   * secondly the anti-clockwise direction is tested, and 
   * finally the error bit is tested.

For each spin direction, the following tests are done:-
   * Accelerate up to maximum speed (approx. 4000 RPM)
   * Hold steady at maximum speed, while origin detected (see below)
   * Decelerate to minimum speed, (approx. 50 RPM)


In the right hand columns of the printout, are printed the 4 QEI parameters [Error_status, Revolution_Counter, angular_Position, angular_Velocity]
 
At start-up,  the motor's origin is unknown, so the initial QEI position (P) is assumed to be zero, and P increments from zero.
During the acceleration test, the velocity(V) increases to its maximum value.
During the Maximum Speed test, the Origin-bit is tested. When the origin is detected, the angular position is reset to zero, and the rev. counter is updated.
During the deceleration test, the velocity(V) decreases to its minimum value.

Notice that the Velocity values do not appear to update immediately; this is due to filtering. The QEI values can be unreliable, therefore the following filter is used. At start-up, or when the spin direction changes, 3 consecutive movements in the same direction are required before a new spin direction is established.

When both clock-wise and anti-clock wise tests have been done. The Error-bit is tested. Three consecutive detections of a low(0) raw Error-bit are required before the Error_status parameter flag is switched on(1).

The receipt of the Error_status flag by the QEI client is used by the test harness to terminate testing of the current motor, and switch to a different motor (or end the program).

Look at the Code
----------------

   #. Examine the application code. In xTIMEcomposer, navigate to the ``src`` directory under ``app_test_qei``  and double click on the ``main.xc`` file within it. The file will open in the central editor window.
      #. Review the ``main.xc`` and note that main() runs 3 tasks on 3 logical cores in parallel.
         * ``gen_all_qei_test_data()`` Generates test data and transmits it on the 4-bit test port (``p4_tst``).
         * ``foc_qei_do_multiple()`` is the QEI Server, receiving test data on the 4-bit QEI port (``p4_qei``), processes the data, and transmitting output data over channel ``c_qei``
         * ``disp_all_qei_client_data()`` contains the QEI Client which receives QEI output data over channel ``c_qei``, and displays the results. ``gen_all_qei_test_data()`` and ``disp_all_qei_client_data()`` both produce display information in parallel. 
         * The other 2 functions in ``main.xc`` are ``init_locks()`` and ``free_locks()``. These are used control a MutEx which allows only one core at a time to print to the display.
   #. Find the ``app_global.h`` header. At the top are the motor definitions. The QEI definitions are specific to the type of motor being used and are currently set up for the LDO motors supplied with the development kit.
   #. Find the file ``check_qei_tests.xc``. In here the function ``disp_motor_qei_client_data()`` handles the QEI output data for one motor. In the 'while loop' is a function ``foc_qei_get_parameters()``. This is the QEI Client. It communicates with the QEI server function ``foc_qei_do_multiple()`` via channel ``c_qei``. The 'while loop' is paced to request QEI data over the ``c_qei`` channel every 40 micro-seconds. This is typical of the issue rate when using real hardware.
   #. Now that the application has been run with the default settings, you could try selecting the QEI filter by setting ``#define QEI_FILTER 1`` in the FIXME.h file This selects a low-pass filter that smooths out changes in velocity values. Make this change and then rebuild and rerun the simulation.
   #. To further explore the capabilities of the simulator,find the items under ``XMOS Examples:Simulator`` in the xSOFTip browser pane. Drag one of them into the Project Explorer to get started.
