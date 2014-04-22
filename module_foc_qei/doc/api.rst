.. _sec_module_foc_qei_api:

API
===

.. _sec_conf_defines:

Configuration Defines
---------------------
.. doxygendefine:: NUMBER_OF_MOTORS 
.. doxygendefine:: NUM_POLE_PAIRS 
.. doxygendefine:: QEI_PER_POLE 
.. doxygendefine:: MAX_SPEC_RPM 
.. doxygendefine:: QEI_FILTER 
.. doxygendefine:: QEI_RS_MODE
.. doxygendefine:: PLATFORM_REFERENCE_HZ  

Functions
---------

Data Types
++++++++++

Data Structures
+++++++++++++++
.. doxygenstruct:: QEI_PARAM_TAG

Configuration Functions
+++++++++++++++++++++++

Receive Functions
+++++++++++++++++
.. doxygenfunction:: foc_qei_get_parameters

Transmit Functions
++++++++++++++++++
.. doxygenfunction:: foc_qei_do_multiple
