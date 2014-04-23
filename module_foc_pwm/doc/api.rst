.. _sec_module_foc_pwm_api:

API
===

.. _sec_conf_defines:

Configuration Defines
---------------------
.. doxygendefine:: PWM_RES_BITS
.. doxygendefine:: PWM_DEAD_TIME
.. doxygendefine:: PWM_SHARED_MEM
.. doxygendefine:: PWM_STAGGER
.. doxygendefine:: LOCK_ADC_TO_PWM
.. doxygendefine:: INIT_SYNC_INCREMENT
.. doxygendefine:: PLATFORM_REFERENCE_HZ

Functions
---------

Data Types
++++++++++
.. doxygentypedef:: PORT_TIME_TYP

Data Structures
+++++++++++++++
.. doxygenstruct:: PWM_PARAM_TAG
.. doxygenstruct:: PWM_COMMS_TAG

Configuration Functions
+++++++++++++++++++++++

Initialisation Functions
++++++++++++++++++++++++
.. doxygenfunction:: foc_pwm_config

Receive Functions
+++++++++++++++++
.. doxygenfunction:: foc_pwm_do_triggered

Transmit Functions
++++++++++++++++++
.. doxygenfunction:: foc_pwm_put_parameters

