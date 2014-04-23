Quadrature Encoder Interface (QEI) Component
============================================

:scope: Early Development
:description: A component for interfacing to quadrature encoders with A, B and Index outputs, and optionally an error output
:keywords: position encoder, FOC, Motor Control
:boards: XP-MC-CTRL-L2, XA-MC-PWR-DLV

Features
--------

   * Can handle QEI for multiple motors in one logical core
   * For each motor, each QEI phase of the raw QEI input data is filtered by over-sampling
   * Computes the following QEI parameters: Total_Angular_Position of current motor and master motor, Phase_Interval, Angular_Correction, Correction_Flag, Error_Status

Evaluation
----------

WARNING: This Test-harness is NOT up to date.
The module has PREVIOUSLY been evaluated using the following demo application, but is no longer relevant to the current QEI algorithm:

   * QEI test harness ``Quadrature Encoder Interface (QEI) Test Harness``
