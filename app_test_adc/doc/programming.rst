Analogue to Digital Conversion (ADC) Test Application Programming Overview
==========================================================================

.. _test_adc_Programming:

This file should be read in conjunction with the Quick-Start guide for the ADC Test-bench

The generator runs through a set of tests, these are specified formally as a *test vector* and transmitted to the test checker. For each test, the generator creates 2 sinusoidal wave-trains of ADC-values with the appropriate characteristics (Phase_C is NOT simulated). Each pair of ADC samples is sent to the ADC_Interface. For each test, the generator continues to create the wave-trains until it receives an 'end-of-test' signal from the test checker. 

The ADC_Interface receives a pair of samples from the test generator and converts them into the raw ADC format produced by the ADC hardware being simulated. Currently, this is an Analogue Devices ADC_7265 chip. 

The ADC Server receives the raw ADC data and converts it into the ADC parameters required by the ADC Client. 

The checker reads each test vector in turn sent from the test generator. For each test vector, the checker repeatedly calls the ADC Client to get a stream of ADC parameters until it has enough data to do all the required tests. The checker then sends the 'end-of-test' signal to the generator. Currently the ADC parameters contain 3 phases of ADC. These three wave-trains are subjected to many tests. 

The following tests are always performed

   #. Zero-Sum: For each set of 3 ADC samples, all three phases should sum to zero. 
   #. Zero-Mean: Each sinusoid should have a zero DC offset 
   #. Spin-direction: Checks that the phase difference between the 3 sinusoids is consistent with the demanded spin direction
   #. Gain: Checks the peak-to-peak amplitude is consistent with the demanded ADC Gain
   #. Period: Check the sinusoid period is consistent with the demanded angular speed of the motor.

The following tests are optional

   #. Select which Motor to test: Motor_0 or Motor_1
   #. Selection_1 tests: Small-Gain, Non-Paced, Clock-wise and Fast-Speed
   #. Selection_2 tests: Pacing-On, Large-Gain, Anti-Clockwise and Fast-Speed
   #. Slow_Speed tests: Non-Paced, Large-Gain, and Clock-wise (WARNING: Very Long Test)

These tests take a long time to run, so some combinations have been combined in the above selections. 'Paced' tests are very slow, possibly upto 90 minutes. This is because a new ADC sample is generated only every 40us, and at slow speeds it takes many samples to build a wave-train long enough to test. The 'Non-Paced' mode can be selected to speed up testing by a factor of five. This maintains a 'mock-time' variable that is incremented by 40us every iteration, rather than waiting for the H/W timer to reach that value. The simulator now moves from one time-point to the next as fast as possible.
 
The above options are selected by editing the flags in the file adc_tests.text

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

   #. Examine the application code. In xTIMEcomposer, navigate to the ``src`` directory under ``app_test_adc``  and double click on the ``main.xc`` file within it. The file will open in the central editor window.
   #. Review the ``main.xc`` and note that main() runs 4 tasks on 4 logical cores in parallel.

         * ``gen_all_adc_test_data()`` generates test data and ADC values on channels c_gen_chk and c_gen_adc respectively.
         * ``adc_7265_interface()`` receives the ADC values from the test generator over channel c_gen_adc and converts them into the raw ADC format delivered by the H/W ADC chip, before transmitting them on 32-bit buffered 1-bit wide output port pb32_tst_data. The interface also receives the ADC trigger signal on channel c_pwm2adc_trig. The ADC_interface is also responsible for transmitting and receiving any control signals that are required by the H/W. For the ADC_7265, these are serial-clock, input on port p1_tst_sclk, and a ready signal input on port p1_tst_ready. Finally, an XMOS clock is required to pace the data ports, this is slaved to the serial clock.
         * ``foc_adc_7265_triggered()`` is the ADC Server, receiving raw ADC samples on a 32-bit buffered 1-bit wide input port pb32_adc_data, and storing them in an internal buffer ready for transmitting to the ADC Client. The ADC Server communicates with the ADC client over channel c_adc_chk, receiving requests from the ADC Client, and then transmitting the most recent set of ADC parameters to the ADC Client. Ordinarily the ADC Server interfaces to the H/W ADC-7265 chip. The chip is paced using a serial clock output on port p1_adc_sclk. This in turn is slaved to an XMOS master clock, supplied by adc_xclk. A 'ready' signal is used on bi-directional port p1_adc_ready to gate the transfer of ADC samples. In addition, the ADC_Server also has a 4-bit output port for controlling the multiplexor on the ADC_7265 chip. This is redundant in the test harness, and is left unconnected.

         * ``check_all_adc_client_data()`` receives ADC parameters from the ADC Client down channel c_adc_chk. After sufficient sets of parameters are received, a number of checks are performed, and the results displayed. ``gen_all_adc_test_data()`` and ``check_all_adc_client_data()`` both produce display information in parallel. 
         * The other 2 functions in ``main.xc`` are ``init_locks()`` and ``free_locks()``. These are used to control a MutEx which allows only one core at a time to print to the display.
         * As well as main(), there is a function called xscope_user_init(), this is called before main to initialise xSCOPE capability. In here are registered the 3 ADC signals that were described above, and seen in the xSCOPE viewer.
   #. Find the file ``check_adc_tests.xc``. In here the function ``get_adc_client_data()`` requests new ADC parameters via the ADC Client function ``foc_adc_get_parameters()``. It communicates with the ADC server function ``foc_adc_7265_triggered()`` via channel ``c_adc``. Directly after ``foc_adc_get_parameters()`` are the xSCOPE functions which allow the ADC values to be captured.
   #. Find the ``app_global.h`` header. At the top are the xSCOPE definitions, followed by the motor definitions, and then the ADC definitions.
   #. Note in ``app_global.h`` the define PRINT_TST_ADC used to switch on verbose printing. If verbose printing is on, in the left hand column are the data values used by the test generator. These are a time-stamp, the gain, and two 25-bit 'standardised' ADC values for Phase_A and Phase_B. (The 25 bits are assigned as follows 1 sign-bit, 8 gain-bits and the remaining 16 bits are sinusoid values). In the right hand column are the data values received by the test checker. These are: the time-stamp, and a 12-bit ADC value for each of the 3 phases.
   #. Find the ``adc_tests.txt`` file. In the left hand column are a set of flags to switch On/Off various sets of tests.
   #. Now that the application has been run with the default settings, you could try switching off Selection_1 tests, and switching on Selection_2 tests, by changing the flags in the left hand column. Make this change and then re-run the simulation (no need to re-build). The test harness will run a lot slower as 'Paced' mode is being used. An example of running the maximum set of tests is in file ``adc_max_results.txt``. 
   #. To further explore the capabilities of the simulator, find the items under ``XMOS Examples:Simulator`` in the xSOFTip browser pane. Drag one of them into the Project Explorer to get started.

Testbench Structure
-------------------

The test application uses a maximum of 4 cores containing the following components:-

   #. A test-vector generator and ADC-value generator
   #. An ADC Interface specific to the ADC chip being simulated (provides raw ADC data)
   #. The ADC_Server under test (captures raw ADC data)
   #. The results checker, which captures the ADC parameters using the ADC_Client under test and checks the results

The test application uses the following channels:-

   #. c_pwm2adc_trig[]: A channel for each motor, transmitting synchronisation trigger pulses from PWM server to ADC server
   #. c_adc_chk[]: A channel for each motor, transmitting time-stamps from the ADC_Interface core to the Checker core
   #. c_gen_chk: For communication between Generator and Checker cores
   #. c_gen_adc: For transmitting ADC-data from the Generator core to the ADC_Interface core

The test application uses the following ports:-

   #. adc_xclk: Internal XMOS clock, used as master for driving ADC serial clock
   #. tst_xclk: Internal XMOS clock, slaved from received ADC serial clock

The output pins driven by the ADC_Interface are looped back to the ADC_Server input pins using the *loopback plugin* functionality included within the xSIM simulator, which allows arbitrary definition of pin level loopbacks.
