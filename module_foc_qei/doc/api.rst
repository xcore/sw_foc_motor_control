.. _sec_module_foc_qei_api:

API
===

.. _sec_conf_defines:

Configuration Defines
---------------------
.. doxygendefine:: QEI_PER_REV
.. doxygendefine:: QEI_PHASE_MASK
.. doxygendefine:: QEI_ORIG_MASK
.. doxygendefine:: QEI_NERR_MASK
.. doxygendefine:: QEI_PER_POLE 
.. doxygendefine:: MAX_SPEC_RPM
.. doxygendefine:: NUMBER_OF_MOTORS 
.. doxygendefine:: NUM_POLE_PAIRS 
.. doxygendefine:: MAX_SPEC_RPM 


Functions
---------

Data Structures
+++++++++++++++
.. doxygenstruct:: QEI_PARAM_TAG

Configuration Functions
+++++++++++++++++++++++
.. doxygenfunction:: foc_qei_config

Receive Functions
+++++++++++++++++
.. doxygenfunction:: foc_qei_get_parameters

Transmit Functions
++++++++++++++++++
.. doxygenfunction:: foc_qei_do_multiple
