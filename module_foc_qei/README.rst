Quadrature Encoder Interface (QEI) Component
============================================

:scope: Early Development
:description: A component for interfacing to quadrature encoders with A, B and Index outputs, and optionally an error output
:keywords: position encoder, FOC, Motor Control
:boards: XP-MC-CTRL-L2, XA-MC-PWR-DLV

Features
--------

   * Filters raw QEI input data (from motor)
   * Computes the following QEI parameters: Total_Angular_Position, Phase_Interval, Angular_Correction, Correction_Flag, Error_Status
   * Can handle QEI for multiple motors in one logical core

Evaluation
----------

WARMING: This Test-harness NOT upto date.
The module can be evaluated using the following demo applications:

   * QEI test harness ``Quadrature Encoder Interface (QEI) Test Harness``
