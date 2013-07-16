Analogue-to-Digital Conversion (ADC) Component
==============================================

:scope: Early Development
:description: A component for interfacing to an ADC_7265 chip
:keywords: Analogue, Digital, Conversion, Sampling, FOC, Motor Control
:boards: XP-MC-CTRL-L2, XA-MC-PWR-DLV

Features
--------

   * Captures raw ADC input data (from motor)
   * Computes the following ADC parameters: signed 12-bit ADC value for Phase_A, Phase_B, and Phase_C
   * Can handle ADC for multiple motors in one logical core

Evaluation
----------

This module can be evaluated using the following demo applications:

   * ADC test harness ``Analogue-to-Digital Conversion Interface (ADC) Test Harness``
