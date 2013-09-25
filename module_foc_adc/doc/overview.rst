Overview
========

This module contains an Analogue-to-Digital Converter (ADC) component for Motor Control systems.

When a motor is running, and the driving voltage is momentarily switched off, the resultant current in the coils can be used to estimate the motor's position. This requires specific hardware support. On the motor control board (XP-MC-CTRL-L2) there is circuitry including an ADC chip to perform this function. The coil currents can therefore be sampled and used in a motor control loop. This ADC module is designed to interface to an ADC chip which receives analogue currents from one or more motors, and converts them into digital samples. 

The ADC Server runs in its own logical core and supplies control signals to the ADC chip. Consequently, the module is hardware specific, and currently only an Analogue Devices ADC_7265 chip is supported. The ADC Server also receives the digital samples from the ADC chip on its input ports, which are then processed and the resulting ADC parameters are transmitted down the ADC channel to the ADC Client. A motor control loop, test harness, or other supervisory function can call the ADC Client to obtain the most recent set of ADC parameters.

This implementation is for motors that have 3 electrical phases [A, B, C]. For each motor, the ADC_7265 chip samples Phase_A and Phase_B in parallel. The samples can be 14, 15, or 16 bits wide. However, only 12 bits of data are active. The ADC Server converts the raw ADC data into 12-bit signed values [-2048..2047]. In addition, the ADC Server does the following processing:-

   * Removal of DC bias from Phase_A and Phase_B (zero-mean)
   * Optional noise filter
   * Synthesis of Phase_C from Phase_A and Phase_B
 
The ADC Server is currently configured with a sample frequency of approximately 7 MHz. When a request from the ADC Client is received, the most recent set of 3 processed ADC samples is returned.

Each ADC phase is very noisy, but follows an approximately periodic sequence of samples. A large magnitude indicates a large coil current, and a small magnitude indicates a small coil current. Each phase follows same periodic sequence, however there is a phase difference of 120 degrees between the 3 phases. That is, the maximum value on each phase does NOT occur at the same time.

By convention, the positive spin direction is defined as the one where Phase_A leads Phase_B. E.g. After Phase_A reaches it maximum, Phase_B is next to reach its maximum, followed by Phase_C. This definition is based on time, and is therefore NOT dependent on the spatial orientation of the motor.

WARNING: Each motor manufacturer may use their own definition for spin direction.
