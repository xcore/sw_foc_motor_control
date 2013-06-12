/**
 * The copyrights, all other intellectual and industrial 
 * property rights are retained by XMOS and/or its licensors. 
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2013
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the 
 * copyright notice above.
 **/ 

#include "generate_pwm_tests.h"

/*****************************************************************************/
static void init_pwm( // Initialise PWM parameters for one motor
	PWM_COMMS_TYP &pwm_comms_s, // reference to structure containing PWM data
	chanend c_pwm, // PWM channel connecting Client & Server
	unsigned motor_id // Unique Motor identifier e.g. 0 or 1
)
{
	int phase_cnt; // phase counter


	pwm_comms_s.buf = 0; // Current double-buffer in use at shared memory address
	pwm_comms_s.params.id = motor_id; // Unique Motor identifier e.g. 0 or 1

	// initialise arrays
	for (phase_cnt = 0; phase_cnt < NUM_PWM_PHASES; phase_cnt++)
	{ 
		pwm_comms_s.params.widths[phase_cnt] = 0;
	} // for phase_cnt

	// Receive the address of PWM data structure from the PWM server, in case shared memory is used
	c_pwm :> pwm_comms_s.mem_addr; // Receive shared memory address from PWM server

	return;
} // init_pwm 
/*****************************************************************************/
static void init_test_data( // Initialise PWM Test data
	GENERATE_PWM_TYP &tst_data_s // Reference to structure of PWM test data
)
{
	init_common_data( tst_data_s.common ); // Initialise data common to Generator and Checker
 
	tst_data_s.print = PRINT_TST_PWM; // Set print mode
	tst_data_s.dbg = 0; // Set debug mode

	tst_data_s.period = PWM_PERIOD; // Set period between generations of PWM Client data
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Initialise to skipped test for set-up mode
} // init_motor_tests
/*****************************************************************************/
static void assign_test_vector_width( // Assign Speed-state of test vector
	GENERATE_PWM_TYP &tst_data_s, // Reference to structure of PWM test data
	WIDTH_PWM_ENUM inp_width // Input speed-state
)
{
	switch( inp_width )
	{
		case LARGE: // Constant Fast Speed
			tst_data_s.width = MAX_PWM; // Set pulse width for High speed
		break; // case LARGE:

		case SMALL: // Constant Slow Speed
			tst_data_s.width = MIN_PWM; // Set pulse width for Slow speed
		break; // case SMALL:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown PWM Velocity-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_width )

	tst_data_s.vector.comp_state[WIDTH] = inp_width; // Update speed-state of test vector
} // assign_test_vector_width
/*****************************************************************************/
static void assign_test_vector_phase( // Assign Phase-state of test vector
	GENERATE_PWM_TYP &tst_data_s, // Reference to structure of PWM test data
	PWM_PHASE_ENUM inp_phase // Input phase-state
)
{
	tst_data_s.vector.comp_state[PHASE] = inp_phase; // Update phase-state of test vector
} // assign_test_vector_phase
/*****************************************************************************/
static void assign_test_vector_leg( // Assign PWM-leg state of test vector
	GENERATE_PWM_TYP &tst_data_s, // Reference to structure of PWM test data
	PWM_LEG_ENUM inp_leg // Input leg-state
)
{
	tst_data_s.vector.comp_state[LEG] = inp_leg; // Update PWM-leg state of test vector
} // assign_test_vector_leg
/*****************************************************************************/
static void do_pwm_test( // Performs one PWM test
	GENERATE_PWM_TYP &tst_data_s, // Reference to structure of PWM test data
	chanend c_pwm 				// Channel between Client and Server
)
{
	timer chronometer; // timer


	tst_data_s.time += tst_data_s.period; // Update time for next PWM pulse generation

	// Load test data into PWM phase under test
	tst_data_s.pwm_comms.params.widths[tst_data_s.vector.comp_state[PHASE]] = tst_data_s.width;

// acquire_lock(); printstr( "GP=" ); printuintln( tst_data_s.width ); release_lock(); //MB~

	if (0 == tst_data_s.print)
	{
		printchar('.'); // Progress indicator
	} // if (0 == tst_data_s.print)

	chronometer when timerafter(tst_data_s.time) :> void;	// Wait till test period elapsed

	foc_pwm_put_parameters( tst_data_s.pwm_comms ,c_pwm ); // Update the PWM values

	if (tst_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printstr( "PWM:" ); printintln( tst_data_s.width ); 
		release_lock(); // Release Display Mutex
	} // if (tst_data_s.print)

} // do_pwm_test
/*****************************************************************************/
static void do_pwm_vector( // Do all tests for one PWM test vector
	GENERATE_PWM_TYP &tst_data_s, // Reference to structure of PWM test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	chanend c_pwm, 				// Channel between Client and Server
	int test_cnt // count-down test counter
)
{
	c_tst <: tst_data_s.vector; // transmit test vector details to test checker

	if (tst_data_s.print)
	{
		print_test_vector( tst_data_s.common ,tst_data_s.vector ,"" );
	} // if (tst_data_s.print)

	// Loop through tests for current test vector
	while(test_cnt)
	{
		do_pwm_test( tst_data_s ,c_pwm ); // Performs one PWM test

		test_cnt--; // Decrement test counter
	} // while(test_cnt)

} // do_pwm_vector
/*****************************************************************************/
static void gen_motor_pwm_test_data( // Generate PWM Test data for one motor
	GENERATE_PWM_TYP &tst_data_s, // Reference to structure of PWM test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	chanend c_pwm 				// Channel between Client and Server
)
{
	timer chronometer; // timer


	chronometer :> tst_data_s.time;	// Get time

	if (tst_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printstrln( " Start Test Generation");
		release_lock(); // Release Display Mutex
	} // if (tst_data_s.print)

//MB~		chronometer when timerafter(ts1 + (MILLI_400_SECS << 1) + (256 * thread_id)) :> void;

	// NB These tests assume PWM_FILTER = 0
	assign_test_vector_leg( tst_data_s ,TEST_LEG); // Set which PWM-Leg to test
	assign_test_vector_phase( tst_data_s ,TEST_PHASE ); // Set PWM-phase to test

	assign_test_vector_width( tst_data_s ,LARGE ); // Set test vector to Slow width

	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Skip start-up
	do_pwm_vector( tst_data_s ,c_tst ,c_pwm ,2 );

	tst_data_s.vector.comp_state[CNTRL] = VALID; // Start-up complete, Switch on testing
	do_pwm_vector( tst_data_s ,c_tst ,c_pwm ,MAX_TESTS );

	tst_data_s.vector.comp_state[CNTRL] = QUIT; // Signal that testing has ended for current motor
	do_pwm_vector( tst_data_s ,c_tst ,c_pwm ,1 );

} // gen_motor_pwm_test_data
/*****************************************************************************/
void gen_all_pwm_test_data( // Generate PWM Test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	chanend c_pwm 				// Channel between Client and Server
)
{
	GENERATE_PWM_TYP tst_data_s; // Structure of PWM test data


	init_pwm( tst_data_s.pwm_comms ,c_pwm ,MOTOR_ID );	// Initialise PWM communication data

	init_test_data( tst_data_s );

	gen_motor_pwm_test_data( tst_data_s ,c_tst ,c_pwm );

	if (tst_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printstrln( "Test Generation Ends " );
		release_lock(); // Release Display Mutex
	} // if (tst_data_s.print)
} // gen_all_pwm_test_data
/*****************************************************************************/
