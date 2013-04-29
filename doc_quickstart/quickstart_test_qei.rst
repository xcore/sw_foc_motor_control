.. _test_qei_Quickstart:

Quadrature Encoder (QEI) Simulator Testbench
============================================

This application is an xSIM test harness for the quadrature encoder interface using xTIMEcomposer Studio. It tests the QEI functions in the ``Quadrature Encoder Interface (QEI) Component`` xSOFTip component and directs test results to STDOUT.

The application runs the QEI component in one logical core, and generates stimulus for the components pins from a second logical core. The pins driven by the stimulus generator are looped back to the QEI component pins using the *loopback plugin* functionality included within the xSIM simulator, which allows arbitrary definition of pin level loopbacks. 

No hardware is required to run it.

Import and Build the Application
--------------------------------

1. Open xTIMEcomposer and check that it is operating in online mode. 
   Open the edit perspective (Window->Open Perspective->XMOS Edit).
#. Locate the ``'Test QEI Application'`` item in the xSOFTip pane on the bottom left of the window, 
   and drag it into the Project Explorer window in the xTIMEcomposer. 
   This will also cause the modules on which this application depends to be imported as well. 
   These modules are: ``module_test_qei``, and ``module_locks``.
#. Click on the app_foc_qei item in the Explorer pane then click on the build icon (hammer) in xTIMEcomposer. 
   Check the console window to verify that the application has built successfully. 

For help in using xTIMEcomposer, try the xTIMEcomposer tutorial, that can be found by selecting Help->Tutorials from the xTIMEcomposer menu.

Note that the Developer Column in the xTIMEcomposer on the right hand side of your screen 
provides information on the xSOFTip components you are using. 
Select the ``module_test_qei`` component in the Project Explorer, and you will see its description together with API documentation. 
Having done this, click the `back` icon until you return to this quickstart guide within the Developer Column.

Configure And Run The Simulator
-------------------------------

   #. Double click ``app_test_qei`` in the left hand ``Project Explorer`` window.
   #. Click on the ``Run`` icon (the white arrow in the green circle) in the top menu bar. Select ``Run Configurations``
   #. In ``Run Configurations`` window, double click on ``xCORE Application``.
   #. You should find that the left hand side of the ``Run Configurations`` window, should be populated with details from the ``app_test_qei`` project. If the details are blank, this is probably because the project was not selected correctly in the first step. If this has happened, and the problem persists, browse to the correct project, and select the executable.
   #. Select the ``run on simulator`` button.
   #. Now setup the loopbacks between the stimulus generator and teh QEI component.
      #. Select the ``Simulator`` tab.
      #. Select the ``Loopback`` tab.
      #. Click ``Enable pin connections``
      #. Click ``Add`` and dialogue boxes will appear for Tile, Port, Offset and Width. These should be filled in with the following information and steps shown in the table below. The second time the simulator is run, it is only necessary to click on the ``Run`` icon (the white arrow in the green circle) in the top menu.

+-------+--------+------------+-------+------+
|       |  Tile  |    Port    | Offset| Width|
+=======+========+============+=======+======+
| From: |    1   | XS1_PORT_4B|   0   |   4  |
+-------+--------+------------+-------+------+
| To:   |    1   | XS1_PORT_4F|   0   |   4  |
+-------+--------+------------+-------+------+
| Click ``Add`` again,                       |
| and fill in another set of data            |
+-------+--------+------------+-------+------+
| From: |    1   | XS1_PORT_4A|   0   |   4  |
+-------+--------+------------+-------+------+
| To:   |    1   | XS1_PORT_4E|   0   |   4  |
+-------+--------+------------+-------+------+
| Click ``Apply``                            | 
+-------+--------+------------+-------+------+
| Click ``Run``                              |
+-------+--------+------------+-------+------+

Results 
--------

After a few seconds, test results will start to appear in the console window, and note that ther may be pauses of upto 1 minute in console output. The test lasts upto 10 minutes. It is completed when the following 2 messages have appeared:

   ``Test Generation Ends``        
   ``Test Check Ends``

FIXME - the discussion of test results in the module README.rst needs to be put in here and removed from the module README.rst

Interpreting the Results
++++++++++++++++++++++++

Look at the Code
----------------

   #. Now that the application has been run with the default settings, you could try selecting the QEI filter by setting ``#define QEI_FILTER 1`` in the FIXME.h file This selects a low-pass filter that smooths out changes in velocity values.
   #. Examine the application code. In xTIMEcomposer, navigate to the ``src`` directory under ``app_test_qei``  and double click on the ``main.xc`` file within it. The file will open in the central editor window.
      #. Find the ``main.xc`` file and note that main() runs 3 cores (processes) in parallel. As all cores need to use the same reference frequency of 250 MHz, all cores run on the same tile. 250 MHz is only available on tile_1.
      #. ``gen_all_qei_test_data()`` Generates test data and transmits it on the 4-bit test port (``p4_tst``).
      #. ``foc_qei_do_multiple()`` is the QEI Server, receiving test data on the 4-bit QEI port (``p4_qei``), processes the data, and transmitting output data over channel ``c_qei``
      #. ``disp_all_qei_client_data()`` contains the QEI Client which receives QEI output data over channel ``c_qei``, and displays the results. ``gen_all_qei_test_data()`` and ``disp_all_qei_client_data()`` both produce display information in parallel. The other 2 functions in ``main.xc`` are ``init_locks()`` and ``free_locks()``. These are used control a MutEx which allows only one core at a time to print to the display.
   #. Find the ``app_global.h`` header. At the top are the motor definitions. The QEI definitions are specific to the type of motor being used and are currently set up for the LDO motors supplied with the development kit.
#. Find the file ``check_qei_tests.xc``. In here the function ``disp_motor_qei_client_data()`` 
   handles the QEI output data for one motor. In the 'while loop' is a function ``foc_qei_get_parameters()``. This is the QEI Client. It communicates with the QEI server function ``foc_qei_do_multiple()`` via channel ``c_qei``. The 'while loop' is paced to request QEI data over the ``c_qei`` channel every 40 micro-seconds. This is typical of the issue rate when using real hardware.
