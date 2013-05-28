Quadrature Encoder (QEI) Simulator Testbench
============================================

.. _test_qei_Quickstart:

This application is an xSIM test harness for the quadrature encoder interface using xTIMEcomposer Studio. It tests the QEI functions in the ``Quadrature Encoder Interface (QEI) Component`` xSOFTip component and directs test results to STDOUT.

The application runs the QEI component in one logical core, and generates stimulus for the components pins from a second logical core. The pins driven by the stimulus generator are looped back to the QEI component pins using the *loopback plugin* functionality included within the xSIM simulator, which allows arbitrary definition of pin level loopbacks. 

No hardware is required to run the test harness.

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


For background on the QEI protocol see the overview document in directory module_foc_qei/doc

An example of working test output from a working QEI component can be found in a file named ``qei_results.txt``


Look at the Code
----------------

   #. Examine the application code. In xTIMEcomposer, navigate to the ``src`` directory under ``app_test_qei``  and double click on the ``main.xc`` file within it. The file will open in the central editor window.
      #. Review the ``main.xc`` and note that main() runs 3 tasks on 3 logical cores in parallel.
         * ``gen_all_qei_test_data()`` Generates test data and transmits it on the 4-bit test port (``p4_tst``).
         * ``foc_qei_do_multiple()`` is the QEI Server, receiving test data on the 4-bit QEI port (``p4_qei``), processes the data, and transmitting output data over channel ``c_qei``
         * ``check_all_qei_client_data()`` contains the QEI Client which receives QEI output parameters over channel ``c_qei``, checks the QEI parameters, and displays the results. ``gen_all_qei_test_data()`` and ``check_all_qei_client_data()`` both produce display information in parallel. 
         * The other 2 functions in ``main.xc`` are ``init_locks()`` and ``free_locks()``. These are used control a MutEx which allows only one core at a time to print to the display.
   #. Find the ``app_global.h`` header. At the top are the motor definitions. The QEI definitions are specific to the type of motor being used and are currently set up for the LDO motors supplied with the development kit.
   #. Find the file ``check_qei_tests.xc``. In here the function ``check_motor_qei_client_data()`` handles the QEI output data for one motor. In the 'while loop' is a function ``foc_qei_get_parameters()``. This is the QEI Client. It communicates with the QEI server function ``foc_qei_do_multiple()`` via channel ``c_qei``. The 'while loop' is paced to request QEI data over the ``c_qei`` channel every 40 micro-seconds. This is typical of the issue rate when using real hardware.
   #. Now that the application has been run with the default settings, you could try selecting the QEI filter by setting ``#define QEI_FILTER 1`` in the app_global.h file This selects a low-pass filter that smooths out changes in velocity values. Make this change and then rebuild and rerun the simulation. The test harness will now report many speed/spin failures due to the filtering applied. To get more information on which tests are failing select ``#define PRINT_TST_QEI 0`` in app_global.h. Make this change and then rebuild and rerun the simulation.
   #. To further explore the capabilities of the simulator, find the items under ``XMOS Examples:Simulator`` in the xSOFTip browser pane. Drag one of them into the Project Explorer to get started.
