Field Oriented Control (FOC) Demo Application Programming Overview
==================================================================

.. _app_foc_demo_Programming:

When power-up both motors will start spinning at a slow speed, one in a positive direction, the other in a negative direction.

There are 4 buttons next to the display screen labelled A, B, C, & D. These can be used to control the motors as follows :-

   #. A: Increase the speed of each motor.
   #. B: Decrease the speed of each motor.
   #. C: Run a set of pre-written tests, (lasts about 150 seconds).
   #. D: Reverse the direction of spin

If C is pressed, the other buttons will no longer have any effect, until all tests have finished.

Note well, 'key bounce' is a problem with these buttons. 'sharp stabs' are recommended for clean operation.

Look at the Code
----------------

The steps below are designed to guide an initial understanding of how the application is constructed. Detail on driving the application can also be found in the section below (``Application Programming Interface (API)``).

   #. Examine the application code. In xTIMEcomposer, navigate to the ``src`` directory under ``app_foc_demo``  and double click on the ``main.xc`` file within it. The file will open in the central editor window.
   #. Review the ``main.xc`` and note that main() runs on 2 tiles. One handles all the I/O routines. The other is dedicated to the motor-control functions. On the motor-control tile there are 7 logical cores in used in parallel. All cores run at a reference frequency of 100 MHz.

         * ``run_motor()`` contains the FOC main loop. (one for each motor)
         * ``foc_pwm_do_triggered()`` is the PWM Server, sending PWM voltages to the motor-coils. (one for each motor)
         * ``foc_qei_do_multiple()`` is the QEI Server, receiving data from the QEI sensors. (Shared between motors)
         * ``foc_hall_do_multiple()`` is the Hall Server, receiving data from the Hall sensors, currently unused. (Shared between motors)
         * ``foc_adc_7265_triggered()`` is the ADC Server, receiving data from the ADC sensors. (Shared between motors)
         * ``foc_pwm_config()`` and ``foc_qei_config()`` are used to configure clocks.
         * The remaining 2 functions ``init_locks()`` and ``free_locks()``. are used control a MutEx which allows only one core at a time to print to the display.
         * As well as main(), there is a function called xscope_user_init(), this is called before main to initialise xSCOPE capability. 

On the I/O tile there are currently 2 logical cores used in parallel. All cores run at a reference frequency of 100 MHz.

         * ``foc_display_shared_io_manager()`` is the I/O Server, this contains the API to the motor-control loop. (Shared between motors)
         * ``foc_loop_do_wd()`` is the Watchdog Server, this switches off power to the boards if the motor-contol loop malfunctions. (Shared between motors)

   #. Find the ``app_global.h`` header. At the top are the xSCOPE definitions, followed by the motor definitions, and then the QEI definitions, which are specific to the type of motor being used and are currently set up for the LDO motors supplied with the development kit.


	IO_CMD_GET_VALS	= 1,
	IO_CMD_DIR,
	IO_CMD_GET_VALS2,
	IO_CMD_GET_FAULT,




Application Programming Interface (API)
---------------------------------------

In module_foc_display there is a file called shared_io.xc. This contains the API, which is implemented as an array of channel (c_speed[]), one for each motor. The following user commands are currently implemented on these channels :-

 	 #. IO_CMD_GET_IQ: Get requested velocity and measured velocity (and send to LCD display)
	 #. IO_CMD_SET_SPEED: Set motor speed
 	 #. IO_CMD_INC_SPEED: Increment motor speed
	 #. IO_CMD_DEC_SPEED: Decrement motor speed
	 #. IO_CMD_FLIP_SPIN: Change direction of spin

The above commands can be initiated either by pressing the button A,B, and D. (See above), or from a test script in the function test_motor().

