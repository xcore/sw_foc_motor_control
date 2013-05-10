hall Sensor Input
=================

The Hall sensor input module is provided with a library for both running the thread that handles the direct interface to the pins and also for retrieving and calculating the appropriate information from that thread. 

The particular interface that is implemented utilises four signals comprising three Hall sensors output (A, B, and C) and a No-Error output (E). A, B and C provide incremental information about the position of the motor rotor. The signals A, B and C are provided with 120 degrees of separation so that the direction of rotation can be resolved. There are only 6 valid states for [CBA]. These are [001 <--> 011 <--> 010 <--> 110 <--> 100 <--> 101]. Further more there are only 2 valid transitions from each state. This provides a degree of robustness against sensor errors. Currently non of the above redundancy is exploited, as a 'Hall value filter' has yet to be written.

  .. image:: images/HallOutput.pdf
     :align: center

Configuration
+++++++++++++

The Hall sensor module provides one or multiple Hall sensor device versions of the server. If more than one Hall sensor device is interfaced to the XMOS device, then the designer can opt to use multiple single-device Hall sensor server threads, or one multi-device thread.

The multiple-Hall sensor service loop has a worst-case timing of 2us, therefore being able to service two 24 position Hall sensor devices spinning at 635K RPM.

|newpage|

Hall sensor Server Usage
++++++++++++++++++++++++

To initiate the service the following include is required as well as the function call shown. This defines the ports that are required to read the interface and the channels that will be utilised by the client thread.  The compile time constant *NUMBER_OF_MOTORS* is used to determine how many clients and ports are serviced by the multi-device Hall sensor server.

::

  #include "hall_server.h"

  void foc_hall_do_multiple( streaming chanend c_hall[NUMBER_OF_MOTORS], port in p4_hall[NUMBER_OF_MOTORS] );


Hall sensor Client Usage
++++++++++++++++++++++++

To access the information provided by the Hall sensor the functions listed below can used.

::

  #include "hall_client.h"

  void foc_hall_get_parameters( HALL_PARAM_TYP &hall_param_s,	streaming chanend c_hall	);

The parameters are an array of samples from each of the Hall Sensors. All array values are sampled at the same time point.
