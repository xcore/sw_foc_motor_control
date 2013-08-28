Hall Sensor Simulator Testbench
===============================

.. _test_hall_Quickstart:

This application is an xSIM test harness for the Hall sensor interface using xTIMEcomposer Studio. It tests the Hall functions in the ``Hall Sensor Component`` xSOFTip component and directs test results to STDOUT.

No hardware is required to run the test harness.

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
   #. Click on the arrow next to the ``Run`` icon (the white arrow in the green circle) in the top menu bar. Select ``Run Configurations``
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


Test Results 
------------

After a few seconds, output will start to appear in the console window. A dot is printed every time a Hall client request is made. This gives confidence that the test harness is doing something! The test lasts upto 5 minutes. It should complete with the message "ALL TESTS PASSED". If any tests fail, extra output will be generated giving details on the test(s) that failed.

For background on the Hall protocol see the ``Overview`` document for module_foc_hall

An example of working test output from a working Hall component can be found in a file named ``hall_results.txt``


Using The ``Value Change Dump`` (VCD) File
------------------------------------------

The waveforms on the output pins can be inspected by using a VCD file. This can require a lot of memory and considerably slows down the simulator. First ensure enough memory has been requested in the xTIMEcomposer init file. Go to the root directory where the XMOS tools are installed. Then edit file ``xtimecomposer_bin/xtimecomposer.exe.ini`` and ensure the requested memory is at least 2 GBytes (``-Xmx2048m``)

Now launch xTIMEcomposer and switch on VCD tracing as follows ...
   #. Repeat the actions described above up to but NOT including ...
   #. Click ``Apply``
   #. Now select the ``Signal Tracing`` tab.
   #. Tick the ``Enable Signal Tracing`` box
   #. Click the ``Add`` button
   #. Select ``tile[1]``
   #. Tick the ``ports`` box
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
   #. In the Signals window, expand the signal tree as far as tile[1]->ports->XS1_PORT_4A, now double click on the signal PORT_M1_HALLSENSOR
   #. A trace should appear in the right column of the Waveform window.
   #. To view all the trace click the ``Zoom Fit`` icon (House) at the right of the Waveform window view-bar
   #. Now repeatedly click on the ``Zoom In`` button until the numbers [b a e c d 9] can be seen.

These are the Hall raw-data values and indicate that Motor_0 is turning clock-wise. When the numbers change to [4 5 1 3] the error-bit has been set low to indicate an error condition. Near the middle of the trace, the numbers change order and become [9 d c e a b], this means the motor is now spinning in an anti-clockwise direction.

.. figure:: vcd_hall.*
   :width: 100%
   :align: center
   :alt: Example VCD Waveform

   VCD Waveform

The waveforms for Motor_1 can be viewed by loading Port XS1_PORT_4B (PORT_M2_HALLSENSOR).

Using The ``xSCOPE`` (xmt) File
-------------------------------

The values of variables in the program can be inspected using the xSCOPE functionality. This allows time-varying changes in variable values to be plotted in a similar manner to using an oscilloscope for real-signals.

Now rebuild the code as follows:-

   #. In the ``Run Configurations`` dialogue box (see above), select the xSCOPE tab
   #. Now select the ``Offline`` button, then click ``Apply``, then click ``Run``

The program will compile and build with the warning ``Constraints checks PASSED WITH CAVEATS``. This is because xSCOPE introduces an unspecified number of chan-ends. Test output will start to appear in the Console window. When the test has completed, move to the Project explorer window. In the app_test_hall directory there should be a file called ``xscope.xmt``. Double click on this file, and the xSCOPE viewer should launch. On the left-hand side of the viewer, under ``Captured Metrics``, select the arrow next to ``n``. A sub menu will open with 3 signals listed: ``Input_Pins``, ``Hall_Value``, and ``Err_Status``. Use the boxes to the left of each signal to switch the traces on and off. The tests take about 17.5ms. The tick marks at the bottom of the window show at what time xSCOPE sampled the signals. The signal is only sampled when the patterns on the Input-pins changes. This is currently approximately every 620us, but varies with both the speed and type of the motor. Now lets look at each trace in more detail:

   #. First, switch off all traces except the ``Err_Status`` trace. The error flag is zero apart from between 6.3 and 8.1ms when the error status was being tested. Now. switch on the Input-pins trace, it will be seen that this corresponds to bit_3 of the Input_pins going to zero (NERR bit). Note that, the Err_status does NOT switch on immediately. This is due to 'noise-filtering': a set of consecutive zero NERR bits are required to switch on the Err_status. Currently, this is set to 2 (using define MAX_HALL_STATUS_ERR in hall_server.h). Also the same number of NERR bits of value one are required to switch OFF the Err_Status flag.

   #. Second, switch off all traces except the ``Hall_Value`` trace. From 0 to 10.8ms we have the clockwise tests, where the Hall value sequence is 001 -> 011 -> 010 -> 110 -> 100 -> 101 -> 001, then from 10.9 to 16.8ms we have the anti-clockwise tests, where the Hall value sequence is 001 -> 101 -> 100 -> 110 -> 010 -> 0111 -> 001. The change in spin direction can be seen in the trace as a vertical line of symmetry at about 10.0ms.

Note well, to view all the trace click the ``Zoom Fit`` icon (House) at the right of the Waveform window view-bar. To zoom in/out click the 'plus/minus' icons to the left of the ``Zoom Fit`` icon

.. figure:: xscope_hall.*
   :align: center
   :width: 100%
   :alt: Example xSCOPE trace

   xSCOPE Trace

To learn more about xSCOPE look at the ``How To`` by selecting ``Window --> Show_View --> How_To_Browser``. Then in the search box type ``xscope``. This should find the section titled ``XMOS Examples: Instrumentation and xSCOPE``. In the sub-section ``Event Examples`` you will find more information on capturing events. In the sub-section ``IO Examples`` you will find more information on re-directing I/O using xSCOPE.
