Quadrature Encoder (QEI) Simulator Testbench
============================================

.. _test_qei_Quickstart:

This application is an xSIM test harness for the quadrature encoder interface using xTIMEcomposer Studio. It tests the QEI functions in the ``Quadrature Encoder Interface (QEI) Component`` xSOFTip component and directs test results to STDOUT.

No hardware is required to run the test harness.

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

The generator runs through a set of tests, these are specified formally as a *test vector* and transmitted to the test checker. For each test the generator creates the appropriate QEI raw-data and drives this onto the output pins. The QEI Server recognises changes on its input pins, processes the new raw-data, and updates the QEI parameters (Velocity, Direction, etc). The test checker reads the specification in the received test vector, then polls the QEI Client for parameters. These parameters are checked for correctness against the test vector specification.

The following tests are always performed
   #. A *Fast* test: the speed should be fast and steady
   #. A *Slow* test: the speed should be slow and steady
   #. An *Acceleration* test: the speed should increase
   #. A *Deceleration* test: the speed should decrease

The following tests are optional
   #. An *Origin* test: the revolution counter should increment when the origin is detected.
   #. An *Error Status* test: the error-status flag should be raised when 3 consecutive error-bits are detected.
   #. A *Spin Direction* test: the spin direction bit should be correctly set

The options are selected by editing the flags in the file qei_tests.txt

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

After a few seconds, output will start to appear in the console window. A dot is printed every time a QEI client request is made. This gives confidence that the test harness is doing something! First Motor_0 is tested, and then Motor_1. The test lasts upto 10 minutes. It should complete with the message "ALL TESTS PASSED". If any tests fail, extra output will be generated giving details on the test(s) that failed.


For background on the QEI protocol see the ``Overview`` document for module_foc_qei

An example of working test output from a working QEI component can be found in a file named ``qei_results.txt``


Using The ``Value Change Dump`` (VCD) File
------------------------------------------

The waveforms on the output pins can be inspected by using a VCD file. This requires a lot of memory and considerably slows down the simulator. First ensure enough memory has been requested in the xTIMEcomposer init file. Go to the root directory where the XMOS tools are installed. Then edit file ``xtimecomposer_bin/xtimecomposer.exe.ini`` and ensure the requested memory is at least 4 GBytes (``-Xmx4096m``)

Now launch xTIMEcomposer and switch on VCD tracing as follows ...
   #. Repeat the actions described above up to but NOT including ...
   #. Click ``Apply``
   #. Now select the ``Signal Tracing`` tab.
   #. Tick the ``Enable Signal Tracing`` box
   #. Click the ``Add`` button
   #. Select ``tile[1]``
   #. Tick the ``+details`` box
   #. Click ``Apply``
   #. Click ``Run``

Test Results 
------------

You may want to kill the simulations after Motor_0 has been tested. This can be done by clicking on the red square button in the view-bar for the console window. 

When the executable has stopped running, view the VCD file as follows:-
   #. In the main toolbar select Tools->Waveform_Analyzer->Load_VCD_File
   #. Browse to the application root directory or where the VCD file was created.
   #. Select the VCD file and click the ``OK`` button.
   #. The VCD file will start loading, this may take some time, 
   #. WARNING If an ``out-of-memory`` error occurs, increase the xTIMEcomposer memory (described above) to be larger than the VCD file.
   #. When the VCD file has loaded correctly, a list of ports should appear in the ``Signals`` window.
   #. If not already active, open a ``Waveform`` window as follows:-
   #. In the main toolbar, select Window->Show_View->Waves
   #. Now add some signals to the Waves window as follows:-
   #. In the Signals window, open the Ports directory
   #. Now double click on tile[1]->ports->XS1_PORT_4E, a set of 12 waveforms should appear in the right column of the Waveform window.
   #. To view all the trace click the ``Zoom Fit`` icon (House) at the right of the Waveform window view-bar
   #. Now repeatedly click on the ``Zoom In`` button until the numbers [a b 9 8] can be seen in the top waveform (PORT_M1_ENCODER) 

These are the QEI raw-data values and indicate that Motor_0 is turning clock-wise. When the numbers are packed more closely the motor is spinning fast, when the numbers are packed more sparsely the motor is running slowly. Near the middle of the trace, the numbers change order and become [8 9 b a], this means the motor is now spinning in an anti-clockwise direction.

The waveforms for Motor_1 can be viewed by loading Port XS1_PORT_4F (PORT_M2_ENCODER).


Using The ``xSCOPE`` (xmt) File
-------------------------------

The values of variables in the program can be inspected using the xSCOPE functionality. This allow time-varying changes in variable values to be plotted in a similar manner to using an oscilloscope for real-signals. In order to use xSCOPE the following actions are required. (For this application they have already been done) :-

   #. In the ``Makefile`` the option ``-fxscope`` needs to be added to the ``XCC`` flags.
   #. In the ``xC`` files that use xSCOPE functions, the header file <xscope.h> needs to be included.
   #. In the ``main.xc`` file, the xSCOPE initialisation function xscope_user_init() needs to be added.
   #. In each ``xC`` file that uses xSCOPE to plot variables, one or more xSCOPE capture functions are required.

The above requirements are discussed in more detail below in the section ``Look at the Code``. Now rebuild the code as follows:-

   #. In the ``Run Configurations`` dialogue box (see above), select the xSCOPE tab
   #. Now select the ``Offline`` button, then click ``Apply``, then click ``Run``

The program will build and start to produce test output in the Console window. When the test has completed, move to the Project explorer window. In the app_test_adc directory there should be a file called ``xscope.xmt``. Double click on this file, and the xSCOPE viewer should launch. On the left-hand side of the viewer, under ``Captured Metrics``, select the arrow next to ``n``. A sub menu will open with 4 signals listed: ``Rev_Cnt``, ``Angle``, ``Velocity``, and ``Error``. Use the boxes to the left of each signal to switch the traces on and off. The tests take about 31.3ms. Now lets look at each trace in more detail:

   #. First, switch off all traces except the ``Error`` trace. The error signal is zero apart from at about 5.7ms when the error status was being tested.

   #. Second, switch off all traces except the ``Velocity`` trace. From 0 to 17.5ms we have the clockwise tests, and the velocity is positive, from 17.5 to 31.3ms we have the anti-clockwise tests and the velocity is negative. For each spin direction there are four modes, Acceleration, Fast-steady, Deceleration, and Slow-steady. During the Fast-steady mode, the speed reaches 4000 RPM.

   #. Third, switch off all traces except the ``Angle`` trace. Again, from 0 to 17.5ms we have the clockwise tests, and the angle increases to about 40 QEI points, then from 17.5 to 31.3ms we have the anti-clockwise tests and the angle decreases. Initially the angle value has not been calibrated, but by default it starts incrementing from arbitary zero. At 5.6ms the angle drops to back to zero. This is where the first origin signal has been detected and now the angle is calibrated (see also ``Rev_cnt`` below. During the anti-clockwise spin the angular position returns to zero at about 27.4ms, and wraps to a large value as the QEI position moves from 0 --> 1023. The angular position then continues to decrease until the end of the tests.

   #. Finally, switch off all traces except the ``Rev_cnt`` trace. The revolution counter increments from 0 to 1 at about 5.6ms. This is when the origin is first detected and the angular is calibrated (See ``Angle`` above). During the anti-clockwise spin tests, the revolution counter returns to zero at about 27.4ms. This corresponds to the angular position decreasing and passing through the origin. 

Note well, to view all the trace click the ``Zoom Fit`` icon (House) at the right of the Waveform window view-bar. To zoom in/out click the 'plus/minus' icons to the left of the ``Zoom Fit`` icon

To learn more about xSCOPE look at the ``How To`` by selecting ``Window --> Show_View --> How_To_Browser``. Then in the search box type ``xscope``. This should find the section titled ``XMOS Examples: Instrumentation and xSCOPE``. In the sub-section ``Event Examples`` you will find more information on capturing events. In the sub-section ``IO Examples`` you will find more information on re-directing I/O using xSCOPE.

Look at the Code
----------------

   #. Examine the application code. In xTIMEcomposer, navigate to the ``src`` directory under ``app_test_qei``  and double click on the ``main.xc`` file within it. The file will open in the central editor window.
   #. Review the ``main.xc`` and note that main() runs 4 tasks on 4 logical cores in parallel.
         * ``gen_all_qei_test_data()`` Generates test data and transmits it on the 4-bit test port (``p4_tst``).
         * ``disp_gen_data()`` Accepts raw QEI data values over a channel (``c_gen_dis``), formats them, and then prints them.
         * ``foc_qei_do_multiple()`` is the QEI Server, receiving test data on the 4-bit QEI port (``p4_qei``), processes the data, and transmitting output data over channel ``c_qei``
         * ``check_all_qei_client_data()`` contains the QEI Client which receives QEI output parameters over channel ``c_qei``, checks the QEI parameters, and displays the results. ``gen_all_qei_test_data()`` and ``check_all_qei_client_data()`` both produce display information in parallel. 
         * The other 2 functions in ``main.xc`` are ``init_locks()`` and ``free_locks()``. These are used control a MutEx which allows only one core at a time to print to the display.
         * As well as main(), there is a function called xscope_user_init(), this is called before main to initialise xSCOPE capability. In here are registered the 4 QEI signals that were described above, and seen in the xSCOPE viewer.
   #. Find the ``app_global.h`` header. At the top are the xSCOPE definitions, followed by the motor definitions, and then the QEI definitions, which are specific to the type of motor being used and are currently set up for the LDO motors supplied with the development kit.
   #. Note in ``app_global.h`` the define PRINT_TST_QEI used to switch on verbose printing. An example of this can be found in file ``qei_results.txt``.
   #. Find the file ``check_qei_tests.xc``. In here the function ``check_motor_qei_client_data()`` handles the QEI output data for one motor. In the 'while loop' is a function ``foc_qei_get_parameters()``. This is the QEI Client. It communicates with the QEI server function ``foc_qei_do_multiple()`` via channel ``c_qei``. The 'while loop' is paced to request QEI data over the ``c_qei`` channel every 40 micro-seconds. This is typical of the issue rate when using real hardware.  Directly after ``foc_qei_get_parameters()`` are the xSCOPE functions which allow the QEI values to be captured.
   #. Now that the application has been run with the default settings, you could try selecting the QEI filter by setting ``#define QEI_FILTER 1`` in the app_global.h file This selects a low-pass filter that smooths out changes in velocity values. Make this change and then rebuild and rerun the simulation. The test harness will now report many speed/spin failures due to the filtering applied. To get more information on which tests are failing select ``#define PRINT_TST_QEI 0`` in app_global.h. Make this change and then rebuild and rerun the simulation.
   #. To further explore the capabilities of the simulator, find the items under ``XMOS Examples:Simulator`` in the xSOFTip browser pane. Drag one of them into the Project Explorer to get started.
