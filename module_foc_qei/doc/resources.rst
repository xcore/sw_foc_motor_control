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

The QEI server uses over-sampling. The higher the over-sampling ratio, the more efficient the noise rejection of the filter. Consequently, the over-sampling ratio is made as large as possible without 'breaking timing'. As a result, the QEI core is using almost all of the processing power available. The over-sampling ratio is set by using the definition HALF_PERIOD. When using a 100 MHz instruction clock (e.g. on a -C5 or -C10 device), there are about 200 ticks between each sample (for one motor), this allows about 8 samples per QEI phase change. With two motors running, the combined QEI data from both motors is sampled about every micro-second.

The module takes 1.06 micro-seconds to service a client request.
