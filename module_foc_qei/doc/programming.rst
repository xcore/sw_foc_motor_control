Programming Guide
=================

Key Files
---------

   * ``qei_client.xc``: Contains the xC implementation of the QEI Client API
   * ``qei_server.xc``: Contains the xC implementation of the QEI Server task

Usage
-----

The following 2 functions are designed to be called from an xC file.

   * ``foc_qei_get_parameters()`` Client function designed to be called from an Xx file each time a new set of QEI parameters are required.
   * ``foc_qei_do_multiple()``, Server function designed to be called from an xC file. It runs on its own core, and receives data from all QEI motor ports.

The following QEI definitions are required. These are set in ``qei_common.h`` or ``app_global.h``

   * QEI_PER_REV  // No of QEI positions per Revolution
   * QEI_PHASE_MASK // Bit Mask for [B A] phase info.
   * QEI_ORIG_MASK // Bit Mask for origin bit.
   * QEI_NERR_MASK // Bit Mask for error status bit (1 == No Errors)
   * PLATFORM_REFERENCE_HZ // Platform Reference Frequency
   * QEI_FILTER // QEI filter switch (0 == Off)
   * MAX_SPEC_RPM // Maximium specified motor speed

Test Applications
-----------------

This module has a test harness called ``Quadrature Encoder Interface (QEI) Test Harness`` which can be found in the xSOFTip Explorer pane.

To get started with this application, run through the instructions in the quickstart guide for the test harness. More details about using this test harness are given below.

Makefile
........

The Makefile is found in the top level directory of the application (e.g. app_test_qei)

The application is for the simulator. 
However the platform being simulated is a Motor control board.
The Makefile TARGET variable needs to be set to Motor control board being used.
E.g. If the platform configuration file is XP-MC-CTRL-L2.xn, then
TARGET = XP-MC-CTRL-L2

The maximum number of motors supported in currently 2, this is set in app_global.h: e.g.
#define NUMBER_OF_MOTORS 2

Running the application with the Command Line Tools
...................................................

Move to the top level directory of the application (e.g. app_test_qei), and type

   * xmake clean
   * xmake all

To start the test type

   * xsim --plugin LoopbackPort.dll "-port tile[1] XS1_PORT_4B 4 0 -port tile[1] XS1_PORT_4F 4 0 -port tile[1] XS1_PORT_4A 4 0 -port tile[1] XS1_PORT_4E 4 0" bin/app_test_qei.xe

Test results will be printed to standard-out.
Remember this is a simulator, and is very slow.
There may be gaps of upto 1 minute between each printed line.
The whole test takes upto 10 minutes to run.

For a explanation of the test results refer to the quickstart guide in doc_quickstart/qei/index.rst

Trouble-shooting
................

**The information in the 'check results' column may disappear**: This and almost any other problem are probably due to not setting the port configuration correctly when calling xSIM.

**The printout may stop** Depending on the speed of your PC (or Mac), there can be upto 1 minute gap between printed lines.

Example Usage In A Motor Control Loop
-------------------------------------

The code example below demonstrates how to use the QEI component with other components in a motor control loop. It shows part of a main.xc file. In here a set of ``par`` statements are used to run a number of components in parallel with a function called ``run_motor``, which contains all the code for a motor control loop.

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
  
  // QEI ports
  on tile[MOTOR_TILE]: buffered in port:4 pb4_qei[NUMBER_OF_MOTORS] = { PORT_M1_ENCODER, PORT_M2_ENCODER };
  
  /*****************************************************************************/
  int main ( void ) // Program Entry Point
  {
    chan c_pwm2adc_trig[NUMBER_OF_MOTORS];
    chan c_pwm[NUMBER_OF_MOTORS];
    streaming chan c_qei[NUMBER_OF_MOTORS];
    streaming chan c_adc_cntrl[NUMBER_OF_MOTORS];
  
  
    par 
    {
      on tile[MOTOR_TILE] :
      {
        par 
        {
          foc_qei_do_multiple( c_qei, pb4_qei );
      
          foc_adc_7265_triggered( c_adc_cntrl ,c_pwm2adc_trig ,pb32_adc_data ,adc_xclk ,p1_adc_sclk ,p1_ready ,p4_adc_mux );
    
          // Loop through all motors
          par (int motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
          {
            run_motor( motor_cnt ,c_pwm[motor_cnt] ,c_qei[motor_cnt] ,c_adc_cntrl[motor_cnt] );
    
            foc_pwm_do_triggered( motor_cnt ,c_pwm[motor_cnt] ,pb32_pwm_hi[motor_cnt] ,pb32_pwm_lo[motor_cnt] 
              ,c_pwm2adc_trig[motor_cnt] ,p16_adc_sync[motor_cnt] ,pwm_clk[motor_cnt] );
          } // par motor_cnt
        } // par
      } // on tile[MOTOR_TILE]
    } // par
  
    return 0;
  } // main
