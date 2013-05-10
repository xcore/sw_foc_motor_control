Quadrature Encoder Input
========================

The quadrature encoder input (QEI) module is provided with a library for both running the thread that handles the direct interface to the pins and also for retrieving
and calculating the appropriate information from that thread. 

The particular interface that is implemented utilises three signals comprising of two quadrature output (A and B) and an index output (I). A and B provide incremental
information while I indicates a return to 0 or origin. The signals A and B are provided out of phase so that the direction of rotation can be resolved.

  .. image:: images/QeiOutput.pdf
     :align: center

Configuration
+++++++++++++

The QEI module provides a multiple QEI device version of the server. All devices run in the same core. It can be confiquired for one device. 

The multiple-QEI service loop has a worst-case timing of 2us per motor. Therefore it can service one 1024 position QEI device spinning at 30K RPM, or 2 devices at 15K RPM.

|newpage|

QEI Server Usage
++++++++++++++++

To initiate the service the following include is required as well as the function call shown. This defines the ports that are required to read the interface and the channel that will be utilised by the client thread.  The compile time constant *NUMBER_OF_MOTORS* is used to determine how many clients and ports are serviced by the multi-device QEI server.

::

  #include "qei_server.h"

  void foc_qei_do_multiple( streaming chanend c_qei[NUMBER_OF_MOTORS], port in p4_qei[NUMBER_OF_MOTORS] );


QEI Client Usage
++++++++++++++++

To access the information provided by the quadrature encoder the functions listed below can used.

::

  #include "qei_client.h"

  void foc_qei_get_parameters( QEI_PARAM_TYP &qei_param_s,	streaming chanend c_qei	);

The parameters are angular_velocity, angular_position, revolution_count and error_status. The position value is returned as a count from the 'index equals zero' position (once it has been detected) and velocity is returned in revolutions per minute (RPM). 

The error_status indicates whether the QEI parameters are valid.
