sw_foc_motor_control Change Log
===============================

1.0.0
-----
  * Test harnesses added for ADC, QEI, and Hall Sensor

  * Changes to dependencies:

    - sc_util: 1.0.0rc0 -> 1.0.3rc0

      + Remove module_slicekit_support (moved to sc_slicekit_support)
      + Update mutual_thread_comm library to avoid communication race conditions
      + Fix module_slicekit_support to work with L16 target
      + Fix to module_logging to remove excess warning and avoid compiler reserved _msg
      + Minor fixes and code tidying to lock module

0.0.1
-----
  * New repos established for motor control xSOFTip
