Overview
========

This module contains a Pulse-Width-Modulation (PWM) interface component for Motor Control systems.

PWM consists of 3 phases (Phase_A, Phase_B, Phase_C). Each phase contains a periodic sequence of varying pulse-widths. A large width is used to generate a large coil current, and a small width used to generate a small coil current. Each phase contains the same sequence of pulse widths, however there is a phase difference of 120 degrees between the 3 phases. That is, the maximum pulse width on each phase does NOT occur at the same time.

By convention, the positive spin direction is defined as the one where Phase_A leads Phase_B. E.g. After Phase_A reaches it maximum, Phase_B is next to reach its maximum, followed by Phase_C. This definition is based on time, and is therefore NOT dependent on the spatial orientation of the motor.

WARNING: Each motor manufacturer may use their own definition for spin direction.

It currently has the following specification:-

  * Maximum loop frequency: 360 kHz if using a reference frequency of 100 MHz (inner PWM loop requires 276 cycles)
  * PWM duty cycle: 4096 cycles, (allows 2048 different voltages)
  * A trigger pulse is output at a fixed offset into the PWM duty cycle. This can be used to synchronise with the ADC xSOFTip module.
  * For each PWM phase, voltages are driven on both a high-leg and low-leg of a balanced line. The low-leg pulse is inverted with respect to the high-leg pulse. The high-leg and low-leg pulse widths can be different sizes. This avoids dangerous current overload of the FET's. Both high-leg and low-leg pulses are symmetrically aligned around a centre line.
