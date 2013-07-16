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

The generator runs through a set of tests, these are specified formally as a *test vector* and transmitted to the test checker. For each test, the generator creates 2 sinusoidal wave-trains of ADC-values with the appropriate characterisitics. (Phase_C is NOT simulated) Each pair of of ADC samples is sent to the ADC_Interface. For each test, the generator continues to create the wave-trains until it receives an 'end-of-test' signal from the test checker. 

The ADC_Interface receives a pair of sample from the test generator and converts them into the raw ADC format produced by the ADC hardware being simulated. Currently, this is an Analogue Devices ADC_7265 chip. 

The ADC Server receives the raw ADC data and converts it into the ADC parameters required by the ADC Client. 

The checker reads each test vector in turn sent from the test generator. For each test vector, the checker repeatedly calls the ADC Client to get a stream of ADC parameters until it has enough data to do all the required tests. The checker then sends the 'end-of-test' signal to the generator. Currenetly the ADC parameters contain 3 phases of ADC. These three wave-trains are subjected to many tests. 

The following tests are always performed
   #. Zero-Sum: For each set of 3 ADC samples, all three phases should sum to zero. 
   #. Zero-Mean: Each sinusoid should have a zero DC offset 
   #. Spin-direction: Checks that the phase difference between the 3 sinusoids is consistent with the demanded spin direction
   #. Gain: Checks the peak-to-peak amplitude is consistent with the demanded ADC Gain
   #. Period: Check the sinusoid period is consitent with the demanded angular speed of the motor.

The following tests are optional
   #. Which motor to test (0 or 1)
   #. Selection_1 tests: Small-Gain, Non-Paced, Clock-wise and Fast-Speed
   #. Selection_2 tests: Pacing-On, Large-Gain, Anti-Clockwise and Fast-Speed
   #. Slow_Speed tests: Non-Paced, Large-Gain, and Clock-wise (WARNING: Very Long Test)

These tests take a long time to run, so some combinations have been combined in the above selections. 'Paced' tests are very slow, possibly upto 90 minutes. This is because a new ADC sample is generated only every 40us, and at slow speeds it takes many samples to build a wave-train long enough to test. The 'Non-Paced' mode can be selected to speed up testing by a factor of five. This maintains a 'mock-time' variable that is incremented by 40us every iteration, rather than waiting for the H/W timer to reach that value. The simulator now moves swiftly from one time-point to the next as fast as possible.
 
The above options are selected by editing the flags in the file adc_tests.txt

Import and Build the Application
--------------------------------

   1. Open xTIMEcomposer and check that it is operating in online mode. Open the edit perspective (Window->Open Perspective->XMOS Edit).
   #. Locate the ``Analogue-to-Digital Conversion Test Harness`` item in the xSOFTip pane on the bottom left of the window and drag it into the Project Explorer window in the xTIMEcomposer. This will also cause the modules on which this application depends to be imported as well. These modules are: ``module_foc_adc``, and ``module_locks``.
   #. Click on the app_test_adc item in the Explorer pane then click on the build icon (hammer) in xTIMEcomposer. 
   #. Check the console window to verify that the application has built successfully. 

For help in using xTIMEcomposer, try the xTIMEcomposer tutorial, that can be found by selecting Help->Tutorials from the xTIMEcomposer menu.

Note that the Developer Column in the xTIMEcomposer on the right hand side of your screen provides information on the xSOFTip components you are using. 
Select the ``module_foc_adc`` component in the Project Explorer, and you will see its description together with API documentation. 
Having done this, click the ``back`` icon until you return to this quickstart guide within the Developer Column.

Configure And Run The Simulator
-------------------------------

   #. Double click ``app_test_adc`` in the left hand ``Project Explorer`` window.
   #. Click on the arrow next to the ``Run`` icon (the white arrow in the green circle) in the top menu bar. Select ``Run Configurations``
   #. In ``Run Configurations`` window, double click on ``xCORE Application``.
   #. You should find that the left hand side of the ``Run Configurations`` window, should be populated with details from the ``app_test_adc`` project. If the details are blank, this is probably because the project was not selected correctly in the first step. If this has happened, and the problem persists, browse to the correct project, and select the executable.
   #. Select the ``run on simulator`` button.
   #. Now setup the loopbacks between the stimulus generator and the ADC component.

      #. Select the ``Simulator`` tab.
      #. Select the ``Loopback`` tab.
      #. Click ``Enable pin connections``.
      #. Click ``Add`` and dialogue boxes will appear for Tile, Port, Offset and Width. These should be filled in with the following information and steps shown in the table below. The second time the simulator is run, it is only necessary to click on the ``Run`` icon (the white arrow in the green circle) in the top menu.

                +-------+--------+------------+-------+------+
                | From: |    1   | XS1_PORT_1G|   0   |   1  |
                +-------+--------+------------+-------+------+
                | To:   |    1   | XS1_PORT_1C|   0   |   1  |
                +-------+--------+------------+-------+------+

      #. Click ``Add`` again and then do the below

                +-------+--------+------------+-------+------+
                | From: |    1   | XS1_PORT_1H|   0   |   1  |
                +-------+--------+------------+-------+------+
                | To:   |    1   | XS1_PORT_1D|   0   |   1  |
                +-------+--------+------------+-------+------+

      #. Click ``Add`` again and then do the below

                +-------+--------+------------+-------+------+
                | From: |    1   | XS1_PORT_1I|   0   |   1  |
                +-------+--------+------------+-------+------+
                | To:   |    1   | XS1_PORT_1E|   0   |   1  |
                +-------+--------+------------+-------+------+

      #. Click ``Add`` again and then do the below

                +-------+--------+------------+-------+------+
                | From: |    1   | XS1_PORT_1J|   0   |   1  |
                +-------+--------+------------+-------+------+
                | To:   |    1   | XS1_PORT_1F|   0   |   1  |
                +-------+--------+------------+-------+------+

      #. Click ``Apply``
      #. Click ``Run``


Test Results 
------------

After a few seconds, output will start to appear in the console window. A dot is printed every time a ADC client request is made. This gives confidence that the test harness is doing something! The default test lasts about 1 minute. It should complete with the message "ALL TESTS PASSED". If any tests fail, extra output will be generated giving details on the test(s) that failed.

An example of the default test output from a working ADC component can be found in a file named ``adc_min_results.txt``


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
   #. In the Signals window, select tile[1]->ports->XS1_PORT_1J, and drag this to the left-hand column of the Waveform window
   #. This may not work first time, but try leaving a few seconds between selecting and dragging
   #. When successful a set of 12 waveforms should appear in the right column of the Waveform window. These are for ADC Serial clock. The top trace (PORT_ADC_CLK) should have a period of 140ns (about 7.142 MHz).
   #. Repeat the above process for tile[1]->ports->XS1_PORT_1G, This is the ready signal. The top trace (PORT_ADC_CONV) goes high when an ADC samples is sampled. This should occur on average about every 8 us (When run in 'Non-Paced' mode). 
   #. Repeat the above process for tile[1]->ports->XS1_PORT_1H, This is the data port for Phase_A. The 5th trace down (shiftReg[32]) shows the data being clocked out. 14 bits while the ready signal is high.
   #. Note well, to view all the trace click the ``Zoom Fit`` icon (House) at the right of the Waveform window view-bar. To zoom in/out click the 'lpus/minus' icons to the left of the ``Zoom Fit`` icon.


Look at the Code
----------------
   #. Examine the application code. In xTIMEcomposer, navigate to the ``src`` directory under ``app_test_adc``  and double click on the ``main.xc`` file within it. The file will open in the central editor window.
   #. Review the ``main.xc`` and note that main() runs 4 tasks on 4 logical cores in parallel.
         * ``gen_all_adc_test_data()`` generates test data and ADC values on channels c_gen_chk and c_gen_adc respectively.
         * ``adc_7265_interface()`` receives the ADC values from the test generator over channel c_gen_adc and converts them into the raw ADC format delivered by the H/W ADC chip, before transmitting them on 32-bit buffered 1-bit wide output port pb32_tst_data. The interface also receives the ADC trigger signal on channel c_pwm2adc_trig. The ADC_interface is also responsible for transmiting and receiving any control signals that are required by the H/W. For the ADC_7265, these are serial-clock, input on port p1_tst_sclk, and a ready signal input on port p1_tst_ready. Finally, an XMOS clock is required to pace the data ports, this is slaved to the serial clock.
         * ``foc_adc_7265_triggered()`` is the ADC Server, receiving raw ADC samples from the ADC chip on 32-bit buffered 1-bit wide input port pb32_adc_data and storing them in an internal buffer ready for transmitting to the ADC Client. The ADC Server communicates with the ADC client over channel c_adc_chk, receiving requests from the ADC Client, and then transmitting the most recent set of ADC parameters to the ADC Client. Ordinarily the ADC Server interfaces to the H/W ADC-7265 chip. The chip is paced using a serial clock output on port p1_adc_sclk. This in trun is slaved to an XMOS master clock, supplied by adc_xclk. A 'ready' signal is used on bi-directionsl port p1_adc_ready to gate the transfer of ADC samples. In addition, the ADC_Server also has a 4-bit output port for controlling the multiplexor on the ADC_7265 chip. This is redundant in the test harness, and it left unconnected.

         * ``check_all_adc_client_data()`` receives ADC parameters from the ADC Client down channel c_adc_chk. After sufficient sets of parameters are received, a number of checks are performed, and the results displayed. ``gen_all_adc_test_data()`` and ``check_all_adc_client_data()`` both produce display information in parallel. 
         * The other 2 functions in ``main.xc`` are ``init_locks()`` and ``free_locks()``. These are used to control a MutEx which allows only one core at a time to print to the display.
   #. Find the file ``check_adc_tests.xc``. In here the function ``get_adc_client_data()`` requests new the ADC parameters via the ADC Client function ``foc_adc_get_parameters()``. It communicates with the ADC server function ``foc_adc_7265_triggered()`` via channel ``c_adc``. 
   #. Find the ``app_global.h`` header. At the top are the motor definitions. Next down are the ADC definitions.
   #. Note in ``app_global.h`` the define PRINT_TST_ADC used to switch on verbose printing. If verbose printing is on, in the left hand column are the data values used by the test generator. These are a time-stamp, the Gain, and two 25-bit 'standardised' ADC values for Phase_A and Phase_B. In the right hand column are the data values received by the test checker. These are: the time-stamp, and a 12-bit ADC value for each of the 3 phases.
   #. Find the ``adc_tests.txt`` file. In the left hand column are a set of flags to switch On/Off various sets of tests.
   #. Now that the application has been run with the default settings, you could try switching off Selection_1 tests, and switching on Selection_2 tests, by changing the flags in the left hand column. Make this change and then re-run the simulation (no need to re-build). The test harness will run a lot slower as 'Paced' mode is being used. An example of running the maximum set of tests is in file ``adc_max_results.txt``. 
   #. To further explore the capabilities of the simulator, find the items under ``XMOS Examples:Simulator`` in the xSOFTip browser pane. Drag one of them into the Project Explorer to get started.
