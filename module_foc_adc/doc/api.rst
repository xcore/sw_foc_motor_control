.. _sec_module_foc_adc_api:

API
===

.. _sec_conf_defines:

Configuration Defines
---------------------
.. doxygendefine:: NUMBER_OF_MOTORS 
.. doxygendefine:: NUM_POLE_PAIRS 
.. doxygendefine:: MAX_SPEC_RPM 
.. doxygendefine:: ADC_FILTER 
.. doxygendefine:: PWM_RES_BITS
.. doxygendefine:: PLATFORM_REFERENCE_HZ  

Functions
---------

Data Types
++++++++++
.. doxygentypedef:: ADC_TYP

Data Structures
+++++++++++++++
.. doxygenstruct:: ADC_PARAM_TAG

Configuration Functions
+++++++++++++++++++++++

Receive Functions
+++++++++++++++++
.. doxygenfunction:: foc_adc_get_parameters

Transmit Functions
++++++++++++++++++
.. doxygenfunction:: foc_adc_7265_triggered
