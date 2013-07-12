Analogue to Digital Conversion (ADC) Simulator Testbench
========================================================

.. _test_adc_Quickstart:

This application is an xSIM test harness for the analogue to digital conversion interface using xTIMEcomposer Studio. It tests the ADC functions in the ``Analogue to Digital Conversion (ADC) Interface Component`` xSOFTip component and directs test results to STDOUT.

No hardware is required to run the test harness.

The test application uses a maximum of 4 cores containing the following components:-
   #. A test-vector generator and ADC-value generator
   #. An ADC Interface specific to the ADC chip being simulated (provides raw ADC data)
   #. The ADC_Server under test (captures raw ADC data)
   #. Captures ADC parameters using the ADC_Client under test and checks the results

The test application uses the following channels:-
   #. c_pwm2adc_trig[]: A channel for each motor, transmitting synchronisation trigger pulses from PWM server to ADC server
   #. c_adc_chk[]: A channel for each motor, transmitting time-stamps from the ADC_Interface core to the Checker core
   #. c_gen_chk: For communication between Generator and Checker cores
   #. c_gen_adc: For transmitting ADC-data from the Generator core to the ADC_Interface core


on tile[MOTOR_TILE]: buffered in port:32 pb32_adc_data[NUM_ADC_DATA_PORTS] = { PORT_ADC_MISOA ,PORT_ADC_MISOB }; // NB For Phase_A and Phase_B
on tile[MOTOR_TILE]: port p1_adc_ready = PORT_ADC_CONV; // bi-directional 1-bit port as used to as Input ready signal for pb32_adc_data ports, and Output port to ADC chip
on tile[MOTOR_TILE]: out port p1_adc_sclk = PORT_ADC_CLK; // 1-bit port connecting to external ADC serial clock
on tile[MOTOR_TILE]: out port p4_adc_mux = PORT_ADC_MUX; // 4-bit port used to control multiplexor on ADC chip
on tile[MOTOR_TILE]: clock adc_xclk = XS1_CLKBLK_2; // Internal XMOS clock

// Test ports (Borrowed from Motor_0 PWM hi-leg)
on tile[MOTOR_TILE]: buffered out port:32 pb32_tst_data[NUM_ADC_DATA_PORTS]	= {	PORT_M1_HI_A, PORT_M1_HI_B }; // NB For Phase_A and Phase_B
on tile[MOTOR_TILE]: in port p1_tst_ready = PORT_M1_LO_C; // 1-bit Input port as used to as ready signal for pb32_tst_data ports
on tile[MOTOR_TILE]: in port p1_tst_sclk = PORT_M1_HI_C; // 1-bit port receives serial clock
on tile[MOTOR_TILE]: clock tst_xclk = XS1_CLKBLK_3; // Internal XMOS clock
The test application uses the following ports:-
   #. pb32_adc_data[]: For Phase_A and Phase_B, a buffered input port for receiving raw ADC data
   #. p1_adc_ready: // bi-directional 1-bit port as used to as Input ready signal for pb32_adc_data ports, and Output port to ADC chip
   #. p1_adc_sclk: // 1-bit output port connecting to external ADC serial clock
   #. p4_adc_mux: // 4-bit output port used to control multiplexor on ADC chip
   #. pb32_tst_data[]: For Phase_A and Phase_B, a buffered output port for transmitting generated raw ADC data
   #. p1_tst_ready: // 1-bit input port as used to as ready signal for pb32_tst_data ports
   #. p1_tst_sclk: // 1-bit input port receives serial clock

The test application uses the following clocks:-
   #. adc_xclk: Internal XMOS clock, used a master for driving ADC serial clock
   #. tst_xclk: Internal XMOS clock, slaved from received ADC serial clock

The output pins driven by the ADC_Interface are looped back to the ADC_Server input pins using the *loopback plugin* functionality included within the xSIM simulator, which allows arbitrary definition of pin level loopbacks.

The generator runs through a set of tests, these are specified formally as a *test vector* and transmitted to the test checker. For each test, the generator creates 2 sinusoidal wave-trains of ADC-values with the appropriate characterisitics. (Phase_C is NOT simulated) Each pair of of ADC samples is sent to the ADC_Interface for conversion into the raw ADC format produced by the ADC hardware being simulated. Currently, this is an Analogue Devices ADC_7265 chip. The ADC Server receives the raw ADC data and converts it into the ADC parameters required by the ADC Client. 3 Phases of  in turn sends the pulse-width to the ADC Server. The ADC Server converts the pulse-width into a ADC wave-train and drives this onto the output pins. The 3 ADC Capture cores sample their respective input pins every 32-bits, if a new sample is detected this is transmitted to the ADC Checker. The ADC Checker stores the raw ADC data in a buffer until such time when it can be checked. The ADC test checker also reads the specification in the received test vector. The received ADC data is then checked for correctness against the test vector specification.

The following tests are always performed
   #. A Small width pulse: for slow speeds
   #. A Large width pulse: for fast speeds
   #. The 'Dead-Time' gap between adjacent High-Leg and Low-Leg edges

The following tests are optional
   #. A Narrow width pulse: A 32-bit wide pulse for testing Minimum and Maximum speeds
   #. An Equal width pulse: for Square Wave
   #. ADC tests: Measures accurracy of ADC to ADC trigger

The options are selected by editing the flags in the file adc_tests.txt

Import and Build the Application
--------------------------------

   1. Open xTIMEcomposer and check that it is operating in online mode. Open the edit perspective (Window->Open Perspective->XMOS Edit).
   #. Locate the ``Pulse Width Modulation Test Harness`` item in the xSOFTip pane on the bottom left of the window and drag it into the Project Explorer window in the xTIMEcomposer. This will also cause the modules on which this application depends to be imported as well. These modules are: ``module_adc_foc``, and ``module_locks``.
   #. Click on the app_test_adc item in the Explorer pane then click on the build icon (hammer) in xTIMEcomposer. 
   #. Check the console window to verify that the application has built successfully. 

For help in using xTIMEcomposer, try the xTIMEcomposer tutorial, that can be found by selecting Help->Tutorials from the xTIMEcomposer menu.

Note that the Developer Column in the xTIMEcomposer on the right hand side of your screen 
provides information on the xSOFTip components you are using. 
Select the ``module_adc_foc`` component in the Project Explorer, and you will see its description together with API documentation. 
Having done this, click the ``back`` icon until you return to this quickstart guide within the Developer Column.

Configure And Run The Simulator
-------------------------------

   #. Double click ``app_test_adc`` in the left hand ``Project Explorer`` window.
   #. Click on the arrow next to the ``Run`` icon (the white arrow in the green circle) in the top menu bar. Select ``Run Configurations``
   #. In ``Run Configurations`` window, double click on ``xCORE Application``.
   #. You should find that the left hand side of the ``Run Configurations`` window, should be populated with details from the ``app_test_adc`` project. If the details are blank, this is probably because the project was not selected correctly in the first step. If this has happened, and the problem persists, browse to the correct project, and select the executable.
   #. Select the ``run on simulator`` button.
   #. Now setup the loopbacks between the stimulus generator and the
      ADC component.

      #. Select the ``Simulator`` tab.
      #. Select the ``Loopback`` tab.
      #. Click ``Enable pin connections``.
      #. Click ``Add`` and dialogue boxes will appear for Tile, Port, Offset and Width. These should be filled in with the following information and steps shown in the table below. The second time the simulator is run, it is only necessary to click on the ``Run`` icon (the white arrow in the green circle) in the top menu.

                +-------+--------+------------+-------+------+
                | From: |    1   | XS1_PORT_1A|   0   |   1  |
                +-------+--------+------------+-------+------+
                | To:   |    1   | XS1_PORT_1K|   0   |   1  |
                +-------+--------+------------+-------+------+

      #. Click ``Add`` again and then do the below

                +-------+--------+------------+-------+------+
                | From: |    1   | XS1_PORT_1B|   0   |   1  |
                +-------+--------+------------+-------+------+
                | To:   |    1   | XS1_PORT_1L|   0   |   1  |
                +-------+--------+------------+-------+------+

      #. Click ``Add`` again and then do the below

                +-------+--------+------------+-------+------+
                | From: |    1   | XS1_PORT_1C|   0   |   1  |
                +-------+--------+------------+-------+------+
                | To:   |    1   | XS1_PORT_1M|   0   |   1  |
                +-------+--------+------------+-------+------+

      #. Click ``Add`` again and then do the below

                +-------+--------+------------+-------+------+
                | From: |    1   | XS1_PORT_1D|   0   |   1  |
                +-------+--------+------------+-------+------+
                | To:   |    1   | XS1_PORT_1N|   0   |   1  |
                +-------+--------+------------+-------+------+

      #. Click ``Add`` again and then do the below

                +-------+--------+------------+-------+------+
                | From: |    1   | XS1_PORT_1E|   0   |   1  |
                +-------+--------+------------+-------+------+
                | To:   |    1   | XS1_PORT_1O|   0   |   1  |
                +-------+--------+------------+-------+------+

      #. Click ``Add`` again and then do the below

                +-------+--------+------------+-------+------+
                | From: |    1   | XS1_PORT_1F|   0   |   1  |
                +-------+--------+------------+-------+------+
                | To:   |    1   | XS1_PORT_1P|   0   |   1  |
                +-------+--------+------------+-------+------+

      #. Click ``Apply``
      #. Click ``Run``


Test Results 
------------

After a few seconds, output will start to appear in the console window. A dot is printed every time a ADC client request is made. This gives confidence that the test harness is doing something! The test lasts about 2 minutes. It should complete with the message "ALL TESTS PASSED". If any tests fail, extra output will be generated giving details on the test(s) that failed.


For background on the ADC protocol see the ``Overview`` document for module_adc_foc

An example of working test output from a working ADC component can be found in a file named ``adc_results.txt``


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

You may want to kill the simulations before testing has finished. This can be done by clicking on the red square button in the view-bar for the console window. 

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
   #. In the Signals window, select tile[1]->ports->XS1_PORT_1N, and drag this to the left-hand column of the Waveform window
   #. This may not work first time, but try leaving a few seconds between selecting and dragging
   #. When successful a set of 12 waveforms should appear in the right column of the Waveform window. These are for Phase_A of the High-Leg
   #. Repeat the above process for tile[1]->ports->XS1_PORT_1K, (Phase_A of the Low-Leg), and tile[1]->ports->XS1_PORT_8C, (the ADC trigger) 
   #. To view all the trace click the ``Zoom Fit`` icon (House) at the right of the Waveform window view-bar
   #. It should be possible to see a train of different pulse widths in traces in PORT_M2_HI_A and PORT_M2_LO_A, and a series of spikes in trace XS1_PORT_8C[Waiting]

Notice that the pulses in PORT_M2_LO_A are slighlty wider than the pulses in PORT_M2_HI_A. This is because the Low-leg has been extended to prevent the potentially dangerous situation of the High-Leg and Low-leg switching at the same time. The ADC trigger should occur 1/4 of a ADC period before the centre of the pulse.

Look at the Code
----------------
   #. Examine the application code. In xTIMEcomposer, navigate to the ``src`` directory under ``app_test_adc``  and double click on the ``main.xc`` file within it. The file will open in the central editor window.
   #. Review the ``main.xc`` and note that main() runs 6 tasks on 6 logical cores in parallel.
         * ``gen_all_adc_test_data()`` Generates test data and pulse-widths on channels c_tst and c_adc respectively.
         * ``foc_adc_do_triggered()`` is the ADC Server, receiving pulse-widths on channel c_adc, and generating raw ADC data on an array of 32-bit buffered output ports(``pb32_adc_hi`` and ``pb32_adc_lo``), and the ADC to ADC trigger on channel ``c_adc2adc_trig``
         * ``capture_adc_leg_data()`` captures the raw ADC data from either the High-Leg or Low-leg ports which has been looped back onto a set of input pins, and transmits this over a channel to the Checker core
         * ``capture_adc_adc_data()`` captures the raw ADC data from the adc-to-adc trigger channel which has been looped back onto a set of input pins, and transmits this over a channel to the Checker core
         * ``check_adc_server_data()`` receives raw ADC data from a number of channels connected to Capture cores, checks it, and displays the results. ``gen_all_adc_test_data()`` and ``check_all_adc_server_data()`` both produce display information in parallel. 
         * ``config_all_ports()`` configures the timers on all ports used to capture ADC-data. These ports are all configured to run from the same clock so that their times are all synchronised.
         * The other 2 functions in ``main.xc`` are ``init_locks()`` and ``free_locks()``. These are used to control a MutEx which allows only one core at a time to print to the display.
   #. Find the file ``generate_adc_tests.xc``. In here the function ``do_adc_test()`` handles the ADC output data via the ADC Client function ``foc_adc_put_parameters()``. It communicates with the ADC server function ``foc_adc_do_triggered()`` via channel ``c_adc``. 
   #. Find the ``app_global.h`` header. At the top are the motor definitions. Next down are the ADC definitions.
   #. Note in ``app_global.h`` the define PRINT_TST_ADC used to switch on verbose printing. An example of this can be found in file ``adc_results.txt``.
   #. Find the ``adc_tests.txt`` file. In the left hand column are a set of flags to switch On/Off various sets of tests.
   #. Now that the application has been run with the default settings, you could try switching off all the optional tests, by setting the flags in the left hand column to 0 (zero). Make this change and then re-run the simulation (no need to re-build). The test harness will run a lot quicker. An example of the verbose printout for the minimum set of tests is in file ``adc_min_results.txt``.
   #. To further explore the capabilities of the simulator, find the items under ``XMOS Examples:Simulator`` in the xSOFTip browser pane. Drag one of them into the Project Explorer to get started.
