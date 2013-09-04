Programming Guide
=================

Key Files
---------

   * ``hall_client.xc``: Contains the XC implementation of the Hall Client API
   * ``hall_server.xc``: Contains the XC implementation of the Hall Server task

Usage
-----

The following 2 functions are designed to be called from an XC file.

   * ``foc_hall_get_parameters()`` Client function designed to be called from an XC file each time a new set of Hall parameters are required.
   * ``foc_hall_do_multiple()``, Server function designed to be called from an XC file. It runs on its own core, and receives data from all Hall motor ports.

The following Hall definitions are required. These are set in ``hall_common.h`` or ``app_global.h``

   * HALL_PER_REV  // No of Hall positions per Revolution
   * HALL_PHASE_MASK // Bit Mask for [C B A] phase info.
   * HALL_NERR_MASK // Bit Mask for error status bit (1 == No Errors)
   * PLATFORM_REFERENCE_HZ // Platform Reference Frequency
   * HALL_FILTER // Hall filter switch (0 == Off)
   * MAX_SPEC_RPM // Maximium specified motor speed

Test Applications
-----------------

This module has a test harness called ``Hall Sensor Interface Test Harness`` which can be found in the xSOFTip Explorer pane.

To get started with this application, run through the instructions in the quickstart guide for the test harness. More details about using this test harness are given below.

Makefile
........

The Makefile is found in the top level directory of the application (e.g. app_test_hall)

The application is for the simulator. 
However the platform being simulated is a Motor control board.
The Makefile TARGET variable needs to be set to Motor control board being used.
E.g. If the platform configuration file is XP-MC-CTRL-L2.xn, then
TARGET = XP-MC-CTRL-L2

The maximum number of motors supported in currently 2, this is set in app_global.h: e.g.
#define NUMBER_OF_MOTORS 2

Running the application with the Command Line Tools
...................................................

Move to the top level directory of the application (e.g. app_test_hall), and type

   * xmake clean
   * xmake all

To start the test type

   * xsim --plugin LoopbackPort.dll "-port tile[1] XS1_PORT_4B 4 0 -port tile[1] XS1_PORT_4F 4 0 -port tile[1] XS1_PORT_4A 4 0 -port tile[1] XS1_PORT_4E 4 0" bin/app_test_hall.xe

Test results will be printed to standard-out.
Remember this is a simulator, and is very slow.
There may be gaps of upto 1 minute between each printed line.
The whole test takes upto 10 minutes to run.

For a explanation of the test results refer to the quickstart guide in doc_quickstart/hall/index.rst

Trouble-shooting
................

The information in the 'check results' column may disappear.
This and almost any other problem are probably due to NOT setting the port configuration correctly when calling xsim

The printout may stop.
As mentioned above, depending on the speed of your PC (or Mac), there can be upto 1 minute gap between printed lines.

Example Usage In A Motor Control Loop
-------------------------------------

The code example below demonstrates how to use the Hall Sensor component with other components in a motor control loop. It shows part of a main.xc file. In here a set of ``par`` statements are used to run a number of components in parallel with a function called ``run_motor``, which contains all the code for a motor control loop.

::

  // PWM ports
  on tile[MOTOR_TILE]: buffered out port:32 pb32_pwm_hi[NUMBER_OF_MOTORS][NUM_PWM_PHASES] 
    = {  {PORT_M1_HI_A, PORT_M1_HI_B, PORT_M1_HI_C} ,{PORT_M2_HI_A, PORT_M2_HI_B, PORT_M2_HI_C} };
  on tile[MOTOR_TILE]: buffered out port:32 pb32_pwm_lo[NUMBER_OF_MOTORS][NUM_PWM_PHASES] 
    = {  {PORT_M1_LO_A, PORT_M1_LO_B, PORT_M1_LO_C} ,{PORT_M2_LO_A, PORT_M2_LO_B, PORT_M2_LO_C} };
  on tile[MOTOR_TILE]: clock pwm_clk[NUMBER_OF_MOTORS] = { XS1_CLKBLK_5 ,XS1_CLKBLK_4 };
  on tile[MOTOR_TILE]: in port p16_adc_sync[NUMBER_OF_MOTORS] = { XS1_PORT_16A ,XS1_PORT_16B }; // NB Dummy port
  
  // ADC ports
  on tile[MOTOR_TILE]: buffered in port:32 pb32_adc_data[NUM_ADC_DATA_PORTS] = { PORT_ADC_MISOA ,PORT_ADC_MISOB }; 
  on tile[MOTOR_TILE]: out port p1_adc_sclk = PORT_ADC_CLK; // 1-bit port connecting to external ADC serial clock
  on tile[MOTOR_TILE]: port p1_ready = PORT_ADC_CONV; // 1-bit port used to as ready signal for pb32_adc_data ports and ADC chip
  on tile[MOTOR_TILE]: out port p4_adc_mux = PORT_ADC_MUX; // 4-bit port used to control multiplexor on ADC chip
  on tile[MOTOR_TILE]: clock adc_xclk = XS1_CLKBLK_2; // Internal XMOS clock
  
  // Hall ports
  on tile[MOTOR_TILE]: port in p4_hall[NUMBER_OF_MOTORS] = { PORT_M1_HALLSENSOR ,PORT_M2_HALLSENSOR };
  
  /*****************************************************************************/
  int main ( void ) // Program Entry Point
  {
    chan c_pwm2adc_trig[NUMBER_OF_MOTORS];
    chan c_pwm[NUMBER_OF_MOTORS];
    streaming chan c_hall[NUMBER_OF_MOTORS];
    streaming chan c_adc_cntrl[NUMBER_OF_MOTORS];
  
  
    par 
    {
      on tile[MOTOR_TILE] :
      {
        par 
        {
          foc_hall_do_multiple( c_hall ,p4_hall );
      
          foc_adc_7265_triggered( c_adc_cntrl ,c_pwm2adc_trig ,pb32_adc_data ,adc_xclk ,p1_adc_sclk ,p1_ready ,p4_adc_mux );
    
          // Loop through all motors
          par (int motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
          {
            run_motor( motor_cnt ,c_pwm[motor_cnt] ,c_hall[motor_cnt] ,c_adc_cntrl[motor_cnt] );
    
            foc_pwm_do_triggered( motor_cnt ,c_pwm[motor_cnt] ,pb32_pwm_hi[motor_cnt] ,pb32_pwm_lo[motor_cnt] 
              ,c_pwm2adc_trig[motor_cnt] ,p16_adc_sync[motor_cnt] ,pwm_clk[motor_cnt] );
          } // par motor_cnt
        } // par
      } // on tile[MOTOR_TILE]
    } // par
  
    return 0;
  } // main
