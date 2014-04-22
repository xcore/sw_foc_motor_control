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

When using a 100 MHz instruction clock (e.g. on a -C5 or -C10 device), the server takes approximately 0.95 micro-seconds to service a change on the input pins. With 2 motors running, this means the QEI data can be sampled every 1.9 micro-seconds. At a speed of 4000 RPM, this allows about 8 samples per phase_change.

The module takes 1.06 micro-seconds to service a client request.

