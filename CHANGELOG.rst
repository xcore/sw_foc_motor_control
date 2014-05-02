sw_foc_motor_control Change Log
===============================

1.0.1
-----

  * QEI server algorithm now uses regular sampling of QEI ports (i.e. NOT triggered by change on port). This is more robust to QEI noise.

  * QEI client now transmits position of current motor AND master motor.

  * PWM bug-fix: 'Previous_PWM_Time' was being set to zero on re-start, now reads from timer.

  * PWM processing (and Start-up of motors) is now staggered, so common processing load is symmetrically interleaved.

  * ADC median filter added to remove glitch noise

  * ADC resolution increased, to reduce 'speed flutter' at low speeds.

1.0.0
-----
  * Test harnesses added for ADC, QEI, and Hall Sensor

  * Changes to dependencies:

    - sc_util: 1.0.1rc0 -> 1.0.4rc0

      + module_logging now compiled at -Os
      + debug_printf in module_logging uses a buffer to deliver messages unfragmented
      + Fix thread local storage calculation bug in libtrycatch
      + Fix debug_printf itoa to work for unsigned values > 0x80000000
      + Remove module_slicekit_support (moved to sc_slicekit_support)
      + Update mutual_thread_comm library to avoid communication race conditions
      + Fix module_slicekit_support to work with L16 target

0.0.1
-----
  * New repos established for motor control xSOFTip
