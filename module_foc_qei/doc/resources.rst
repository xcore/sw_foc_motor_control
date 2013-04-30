Resource Requirements
=====================

Processing Requirements
+++++++++++++++++++++++

+---------------+-------+-------+
| Resource      | Server| Client| 
+===============+=======+=======+
+---------------+-------+-------+
| Logical Cores |   1   |   1   |
+---------------+-------+-------+
| Input Ports   |   1   |       |
+---------------+-------+-------+
| Output Ports  |       |       |
+---------------+-------+-------+
| Channel Ends  |   1   |   1   |
+---------------+-------+-------+
| Timers        |   1   |       |
+---------------+-------+-------+
| Clocks        |       |       |
+---------------+-------+-------+


Memory Requirements
+++++++++++++++++++

Approximate memory usage for this module is (figures shown in Bytes):

* codememory: 5K
* datamemory: 1K


Performance
+++++++++++

When using a 500 MHz instruction clock, the server takes approximately 0.90 micro-seconds to service a change on the input pins, and 1.06 micro-seconds to service a client request.
