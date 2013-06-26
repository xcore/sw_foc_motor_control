Resource Requirements
=====================

+---------------+-------+
| Resource      | Used  |
+===============+=======+
| Logical Cores |   1   |
+---------------+-------+
| Input Ports   |   1   |
+---------------+-------+
| Output Ports  |       |
+---------------+-------+
| Channel Ends  |   1   |
+---------------+-------+
| Timers        |   1   |
+---------------+-------+
| Clocks        |       |
+---------------+-------+
| Code Memory   |  5KB  |
+---------------+-------+
| Data Memory   |  1KB  |
+---------------+-------+

Performance
===========

When using a 500 MHz instruction clock (e.g. on a -C5 or -C10 device) , the server takes approximately 0.90 micro-seconds to service a change on the input pins which means an encoder with 1024 graduations would be able to spin at approximately 60,000 RPM before this module would yield in inaccurate count to the client application.

The module takes 1.06 micro-seconds to service a client request.

