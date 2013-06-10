Hall Sensor Simulator Testbench
===============================

.. _test_hall_Quickstart:

This application is an xSIM test harness for the Hall sensor interface using xTIMEcomposer Studio. It tests the Hall functions in the ``Hall Sensor Component`` xSOFTip component and directs test results to STDOUT.

No hardware is required to run the test harness.

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

The generator runs through a set of tests, these are specified formally as a *test vector* and transmitted to the test checker. For each test the generator creates the appropriate Hall raw-data and drives this onto the output pins. The Hall Server recognises changes on its input pins, processes the new raw-data, and updates the Hall parameters (Hall_Sensor_value and Error_Status). The test checker reads the specification in the received test vector, then polls the Hall Client for parameters. These parameters are checked for correctness against the test vector specification.

The following tests are currently performed
   #. An *Error Status* test: the error-status flag should be raised when 3 consecutive error-bits are detected.
   #. A *Phase_Value* test: the 3 phase values should form a valid combination
   #. A *Spin Direction* test: the phase values should change in the required order

Import and Build the Application
--------------------------------

   1. Open xTIMEcomposer and check that it is operating in online mode. Open the edit perspective (Window->Open Perspective->XMOS Edit).
   #. Locate the ``Test Hall sensor Application`` item in the xSOFTip pane on the bottom left of the window and drag it into the Project Explorer window in the xTIMEcomposer. This will also cause the modules on which this application depends to be imported as well. These modules are: ``module_foc_hall``, and ``module_locks``.
   #. Click on the app_test_hall item in the Explorer pane then click on the build icon (hammer) in xTIMEcomposer. 
   #. Check the console window to verify that the application has built successfully. 

For help in using xTIMEcomposer, try the xTIMEcomposer tutorial, that can be found by selecting Help->Tutorials from the xTIMEcomposer menu.

Note that the Developer Column in the xTIMEcomposer on the right hand side of your screen 
provides information on the xSOFTip components you are using. 
Select the ``module_foc_hall`` component in the Project Explorer, and you will see its description together with API documentation. 
Having done this, click the ``back`` icon until you return to this quickstart guide within the Developer Column.

Configure And Run The Simulator
-------------------------------

   #. Double click ``app_test_hall`` in the left hand ``Project Explorer`` window.
   #. Click on the ``Run`` icon (the white arrow in the green circle) in the top menu bar. Select ``Run Configurations``
   #. In ``Run Configurations`` window, double click on ``xCORE Application``.
   #. You should find that the left hand side of the ``Run Configurations`` window, should be populated with details from the ``app_test_hall`` project. If the details are blank, this is probably because the project was not selected correctly in the first step. If this has happened, and the problem persists, browse to the correct project, and select the executable.
   #. Select the ``run on simulator`` button.
   #. Now setup the loopbacks between the stimulus generator and the
      Hall sensor component.

      #. Select the ``Simulator`` tab.
      #. Select the ``Loopback`` tab.
      #. Click ``Enable pin connections``.
      #. Click ``Add`` and dialogue boxes will appear for Tile, Port, Offset and Width. These should be filled in with the following information and steps shown in the table below. The second time the simulator is run, it is only necessary to click on the ``Run`` icon (the white arrow in the green circle) in the top menu.

                +-------+--------+------------+-------+------+
                | From: |    1   | XS1_PORT_4F|   0   |   4  |
                +-------+--------+------------+-------+------+
                | To:   |    1   | XS1_PORT_4B|   0   |   4  |
                +-------+--------+------------+-------+------+

      #. Click ``Add`` again and then do the below

                +-------+--------+------------+-------+------+
                | From: |    1   | XS1_PORT_4E|   0   |   4  |
                +-------+--------+------------+-------+------+
                | To:   |    1   | XS1_PORT_4A|   0   |   4  |
                +-------+--------+------------+-------+------+

      #. Click ``Apply``
      #. Click ``Run``


Results 
--------

After a few seconds, test results will start to appear in the console window, and note that there may be pauses of upto 1 minute in console output. The test lasts upto 10 minutes. It is completed when the following 2 messages have appeared::

   Test Generation Ends       
   Test Check Ends

For background on the Hall protocol see the ``Overview`` document for module_foc_hall

An example of working test output from a working Hall component can be found in a file named ``hall_results.txt``


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
   #. In the Signals window, select tile[1]->ports->XS1_PORT_4A, and drag this to the left-hand column of the Waveform window
   #. This may not work first time, but try leaving a few seconds between selecting and dragging
   #. When successful a set of 12 waveforms should appear in the right column of the Waveform window.
   #. To view all the trace click the ``Zoom Fit`` icon (House) at the right of the Waveform window view-bar
   #. Now repeatedly click on the ``Zoom In`` button until the numbers [b a e c d 9] can be seen in the top waveform (PORT_M1_HALLSENSOR) 

These are the Hall raw-data values and indicate that Motor_0 is turning clock-wise. When the numbers change to [4 5 1 3] the error-bit has been set low to indicate an error condition. Near the middle of the trace, the numbers change order and become [9 d c e a b], this means the motor is now spinning in an anti-clockwise direction.

The waveforms for Motor_1 can be viewed by loading Port XS1_PORT_4B (PORT_M2_HALLSENSOR).


Look at the Code
----------------

   #. Examine the application code. In xTIMEcomposer, navigate to the ``src`` directory under ``app_test_hall``  and double click on the ``main.xc`` file within it. The file will open in the central editor window.
   #. Find the ``main.xc`` file and note that main() runs 3 cores (processes) in parallel. All cores run on the same tile at a reference frequency of 100 MHz.
   #. ``gen_all_hall_test_data()`` Generates test data and transmits it on the 32-bit buffered test port (``p4_tst``).
   #. ``foc_hall_do_multiple()`` is the Hall sensor Server, receiving test data on the 4-bit Hall sensor port (``p4_hall``), processes the data, and transmitting output data over channel ``c_hall``
   #. ``check_all_hall_client_data()`` contains the Hall sensor Client which receives Hall sensor output data over channel ``c_hall``, and displays the results. ``gen_all_hall_test_data()`` and ``check_all_hall_client_data()`` both produce display information in parallel. The other 2 functions in ``main.xc`` are ``init_locks()`` and ``free_locks()``. These are used control a MutEx which only allows one core at a time to print to the display.
   #. Find the ``app_global.h`` header. At the top are the motor definitions. The Hall sensor definitions are specific to the type of motor being used and are currently set up for the LDO motors supplied with the development kit.
   #. Find the file ``check_hall_tests.xc``. In here the function ``check_motor_hall_client_data()`` handles the Hall sensor output data for one motor. In the 'while loop' is a function ``foc_hall_get_parameters()``. This is the Hall sensor Client. It communicates with the Hall sensor server function ``foc_hall_do_multiple()`` via channel ``c_hall``. The 'while loop' is paced to request Hall sensor data over the ``c_hall`` channel every 40 micro-seconds. This is typical of the issue rate when using real hardware.

