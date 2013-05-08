UnderStanding Test Results
==========================

Test Output
-----------

For background on the ADC protocol see the overview document in directory module_foc_adc/doc

An example of test output from a working ADC component is held in adc_results.txt

There are two columns of output.
The left hand column is the test input data.
The right hand column is the ADC output data.

   * First, Motor_0 is tested, this is indicated by a prefix of '0:'
   * Second, Motor_1 is tested, this is indicated by a prefix of '1:'

For each Motor, 
   * ADC samples generated from a scaled sine-waved are tested
   * The test for each motor is terminated by assigning the maximum ADC value to all phases.

In the left hand column of the printout, is printed the value of the raw ADC values used for each test,
In the right hand column of the printout, are printed the ADC parameter values for each phase [Phase_A, Phase_B, Phase_C]
