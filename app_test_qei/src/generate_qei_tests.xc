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

#include "generate_qei_tests.h"

/*	[BA] order is 00 -> 01 -> 11 -> 10  Clockwise direction
 *	[BA] order is 00 -> 10 -> 11 -> 01  Anti-Clockwise direction
 */
/*****************************************************************************/
static unsigned speed_to_ticks( // Convert Velocity (in RPM) to ticks per QEI position (in Reference Frequency Cycles)
	int speed // input speed
) // Returns time in ticks
{
	unsigned ticks; // Output ticks per QEI position (in Reference Frequency Cycles)


	ticks = (TICKS_PER_MIN_PER_QEI + (speed >> 1)) / speed;

	return ticks;
}	// speed_to_ticks
/*****************************************************************************/
static void init_test_data( // Initialise QEI Test data
	GENERATE_QEI_TYP &tst_data_s // Reference to structure of QEI test data
)
{
	QEI_PHASE_TYP clkwise = {{ 0 ,2 ,3 ,1 }};	// Array of QEI Phase values [BA} (NB Increment for clock-wise rotation)


	init_common_data( tst_data_s.common ); // Initialise data common to Generator and Checker
 
	tst_data_s.phases = clkwise; // Assign QEI Phase values (NB Increment for clock-wise rotation)

	// Convert Speed values to Ticks/QEI_position values
	tst_data_s.hi_ticks = speed_to_ticks( HIGH_SPEED ); // Convert HIGH_SPEED to ticks
	tst_data_s.lo_ticks = speed_to_ticks( LOW_SPEED ); // Convert LOW_SPEED to ticks

	tst_data_s.print = PRINT_TST_QEI; // Set print mode
	tst_data_s.dbg = 0; // Set debug mode

} // init_test_data
/*****************************************************************************/
static void init_motor_tests( // Initialisation for each set of motor tests
	GENERATE_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	int motor_id // Unique Motor Identifier
)
{
	tst_data_s.id = motor_id; // Assign motor identifier
	tst_data_s.period = tst_data_s.lo_ticks; // Set period for Low start speed
	tst_data_s.off = 0; // Initialise  QEI phase offset
	tst_data_s.cnt = (QEI_REV_MASK - 30); // Set QEI position counter to arbitary number (< QEI_REV_MASK)
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Initialise to skipped test for set-up mode
	tst_data_s.prev_qei = QEI_NERR_MASK; // Initialise to starting value
} // init_motor_tests
/*****************************************************************************/
static void assign_test_vector_error( // Assign Error-state of test vector
	GENERATE_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	ERROR_QEI_ENUM inp_err // Input Error-state
)
{
	switch( inp_err )
	{
		case ERR_OFF: // No Errors
			tst_data_s.nerr = QEI_NERR_MASK; // Set error flag to NO errors (Bit_3 = 1)
		break; // case ERR_OFF:

		case ERR_ON: // Force Error
			tst_data_s.nerr = 0; // Clear (Bit_3) to Signal error condition
		break; // case ERR_OFF:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown QEI Error-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_err )

	tst_data_s.vector.comp_state[ERROR] = inp_err; // Update Error-state of test vector
} // assign_test_vector_error
/*****************************************************************************/
static void assign_test_vector_origin( // Assign Origin-state of test vector
	GENERATE_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	ORIG_QEI_ENUM inp_orig // Input Origin-state
)
{
	switch( inp_orig )
	{
		case ORIG_OFF: // No Origin
			tst_data_s.orig = 0; // Clear origin flag (Bit_2 = 0)
		break; // case ORIG_OFF:

		case ORIG_ON: // Origin
			tst_data_s.orig = QEI_ORIG_MASK; // Set origin flag (Bit_2 = 1)
		break; // case ORIG_OFF:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown QEI Origin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_orig )

	tst_data_s.vector.comp_state[ORIGIN] = inp_orig; // Update Error-state of test vector
} // assign_test_vector_origin
/*****************************************************************************/
static void assign_test_vector_spin( // Assign Spin-state of test vector
	GENERATE_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	SPIN_QEI_ENUM inp_spin // Input Spin-state
)
{
	switch( inp_spin )
	{
		case CLOCK: // Clock-wise
			tst_data_s.inc = 1; // Increment for Clock-Wise direction
		break; // case CLOCK:

		case ANTI: // Anti-clockwise
			tst_data_s.inc = -1; // Decrement for Anti-Clockwise direction
		break; // case ANTI:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown QEI Spin-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_spin )

	tst_data_s.vector.comp_state[SPIN] = inp_spin; // Update Spin-state of test vector
} // assign_test_vector_spin
/*****************************************************************************/
static void assign_test_vector_speed( // Assign Speed-state of test vector
	GENERATE_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	SPEED_QEI_ENUM inp_speed // Input speed-state
)
{
	switch( inp_speed )
	{
		case ACCEL: // Accelerate
			tst_data_s.scale = ACC_SCALE; // Set speed scaling factor for acceleration
		break; // case ACCEL:

		case FAST: // Constant Fast Speed
			tst_data_s.period = tst_data_s.hi_ticks; // Set period for High speed
		break; // case FAST:

		case DECEL: // Accelerate
			tst_data_s.scale = DEC_SCALE; // Set speed scaling factor for deceleration
		break; // case DECEL:

		case SLOW: // Constant Slow Speed
			tst_data_s.period = tst_data_s.lo_ticks; // Low finish speed
		break; // case SLOW:

		default:
			acquire_lock(); // Acquire Display Mutex
			printstrln("ERROR: Unknown QEI Velocity-state");
			release_lock(); // Release Display Mutex
			assert(0 == 1);
		break; // default:
	} // switch( inp_speed )

	tst_data_s.vector.comp_state[SPEED] = inp_speed; // Update speed-state of test vector
} // assign_test_vector_speed
/*****************************************************************************/
static void do_qei_test( // Performs one QEI test
	GENERATE_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	port out p4_tst  // current port on which to transmit test data
)
{
	int qei_val;  // QEI value [E I B A]
	unsigned time_val; // Temporary time value


	tst_data_s.cnt = (tst_data_s.cnt + tst_data_s.inc) & QEI_REV_MASK; // Increment/Decrement (and wrap) QEI position
	tst_data_s.off = (tst_data_s.cnt & QEI_PHASE_MASK); // get phase position for current QEI position

	// Build QEI value ...

	qei_val = tst_data_s.phases.vals[tst_data_s.off]; // Initialise QEI Value with phase bits
	qei_val |= tst_data_s.orig; // OR with origin flag (Bit_2)
	qei_val |= tst_data_s.nerr; // OR with error flag (Bit_3)

	// Wait till test period elapsed
	time_val = tst_data_s.period;

// acquire_lock(); printstr( "GP=" ); printuintln( tst_data_s.period ); release_lock(); //MB~

	while (time_val > NUM_PORT_TIMES)
	{	// Wait for NUM_PORT_TIMES ticks
		p4_tst @ tst_data_s.time <: tst_data_s.prev_qei; // Resend old value
		time_val -= NUM_PORT_TIMES;
	} // while (time_val > (1 << 16))

	tst_data_s.time = (PORT_TIME_TYP)((unsigned)tst_data_s.time + time_val);
	p4_tst @ tst_data_s.time <: qei_val; // Now send new value
	tst_data_s.prev_qei = qei_val; // Store QEI value

	if (tst_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printint( tst_data_s.id ); printstr( ":QEI:" ); printintln( qei_val); 
		release_lock(); // Release Display Mutex
	} // if (tst_data_s.print)

} // do_qei_test
/*****************************************************************************/
static void do_qei_vector( // Do all tests for one QEI test vector
	GENERATE_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	port out p4_tst,  // current port on which to transmit test data
	int test_cnt // count-down test counter
)
{
	c_tst <: tst_data_s.vector; // transmit test vector details to test checker

	// Decelerate
	if (tst_data_s.print)
	{
		print_test_vector( tst_data_s.common ,tst_data_s.vector ,"" );
	} // if (tst_data_s.print)

	// Loop through tests for current test vector
	while(test_cnt)
	{
		do_qei_test( tst_data_s ,p4_tst ); // Performs one QEI test

		// Check which speed mode being tested
		switch( tst_data_s.vector.comp_state[SPEED] )
		{
			case ACCEL: // Accelerate
				tst_data_s.period = (tst_data_s.scale * tst_data_s.period + HALF_SCALE) >> SCALE_PRECISION; // Alter period to change speed
			break; // case ACCEL:
	
			case FAST: // Constant Fast Speed
			break; // case FAST:
	
			case DECEL: // Accelerate
				tst_data_s.period = (tst_data_s.scale * tst_data_s.period + HALF_SCALE) >> SCALE_PRECISION; // Alter period to change speed
			break; // case ACCEL:
	
			case SLOW: // Constant Slow Speed
			break; // case FAST:
	
			default:
				acquire_lock(); // Acquire Display Mutex
				printstrln("ERROR: Unknown QEI Velocity-state");
				release_lock(); // Release Display Mutex
				assert(0 == 1);
			break; // default:
		} // switch( tst_data_s.vector.comp_state[SPEED] )

		test_cnt--; // Decrement test counter
	} // while(test_cnt)

} // do_qei_vector
/*****************************************************************************/
static void gen_motor_qei_test_data( // Generate QEI Test data for one motor
	GENERATE_QEI_TYP &tst_data_s, // Reference to structure of QEI test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	port out p4_tst  // current port on which to transmit test data
)
{
	if (tst_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printstr( " Start Test Gen. For Motor_"); printintln( tst_data_s.id );
		release_lock(); // Release Display Mutex
	} // if (tst_data_s.print)

	p4_tst <: 0 @ tst_data_s.time; // Get start time

	// NB These tests assume QEI_FILTER = 0

	assign_test_vector_error( tst_data_s ,ERR_OFF ); // Set test vector to NO errors
	assign_test_vector_origin( tst_data_s ,ORIG_OFF ); // Set test vector to No Origin
	assign_test_vector_spin( tst_data_s ,CLOCK ); // Set test vector to Clock-wise spin
	assign_test_vector_speed( tst_data_s ,ACCEL ); // Set test vector to Accelerate

	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Switch off testing
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,(START_UP_CHANGES - 1) ); // Start-up phase

	tst_data_s.vector.comp_state[CNTRL] = VALID; // Start-up complete, Switch on testing
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,(ACC_TESTS - START_UP_CHANGES + 1) );

	// NB Do Origin tests while motor running fast, to speed-up simulation

	assign_test_vector_speed( tst_data_s ,FAST ); // Set test vector to constant Fast speed
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Switch off testing while motor settles
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,2 );

	tst_data_s.vector.comp_state[CNTRL] = VALID; // Settling complete, Switch on testing
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,(MAX_TESTS >> 1) );

	assign_test_vector_origin( tst_data_s ,ORIG_ON ); // Set test vector to Origin
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,1 );

	assign_test_vector_origin( tst_data_s ,ORIG_OFF ); // Set test vector to No Origin
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,(MAX_TESTS >> 1) );

	assign_test_vector_speed( tst_data_s ,DECEL ); // Set test vector to Decelerate
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Switch off testing while motor starts braking
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,1 );

	tst_data_s.vector.comp_state[CNTRL] = VALID; // Braking started, Switch on testing
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,(DEC_TESTS - 1) );

	assign_test_vector_speed( tst_data_s ,SLOW ); // Set test vector to constant Slow speed
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Switch off testing while motor settles
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,1 );

	tst_data_s.vector.comp_state[CNTRL] = VALID; // Settling complete, Switch on testing
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,(MIN_TESTS - 1) );

	assign_test_vector_spin( tst_data_s ,ANTI ); // Set test vector to Anti-clockwise spin
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Switch off testing, until server is confident of new spin direction
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,(MAX_CONFID + 2) );

	tst_data_s.vector.comp_state[CNTRL] = VALID; // Server confident, Switch on testing
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,MIN_TESTS );

	assign_test_vector_speed( tst_data_s ,ACCEL ); // Set test vector to Accelerate
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Switch off testing while motor starts accelerating
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,1 );

	tst_data_s.vector.comp_state[CNTRL] = VALID; // Acceleration started, Switch on testing
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,(ACC_TESTS - 1) );

	// NB Do Error tests while motor running fast, to speed-up simulation

	assign_test_vector_speed( tst_data_s ,FAST ); // Set test vector to constant Fast speed
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Switch off testing while motor settles
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,2 );

	tst_data_s.vector.comp_state[CNTRL] = VALID; // Settling complete, Switch on testing
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,((MAX_TESTS >> 1) - MAX_QEI_STATUS_ERR) );

	assign_test_vector_error( tst_data_s ,ERR_ON ); // Switch on error-bit
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Switch off testing, while server counts required consecutive errors
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,MAX_QEI_STATUS_ERR );

	tst_data_s.vector.comp_state[CNTRL] = VALID; // Switch on testing, now error-status should be set
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,MAX_QEI_STATUS_ERR );

	assign_test_vector_error( tst_data_s ,ERR_OFF ); // Switch off error-bit
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Switch off testing, while server counts required consecutive non-errors
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,MAX_QEI_STATUS_ERR );

	tst_data_s.vector.comp_state[CNTRL] = VALID; // Switch on testing, now error-status should be cleared
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,((MAX_TESTS - MAX_QEI_STATUS_ERR) >> 1) );

	assign_test_vector_speed( tst_data_s ,DECEL ); // Set test vector to Decelerate
	tst_data_s.vector.comp_state[CNTRL] = SKIP; // Switch off testing while motor starts braking
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,1 );

	tst_data_s.vector.comp_state[CNTRL] = VALID; // Braking started, Switch on testing
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,(DEC_TESTS - 1) );

	tst_data_s.vector.comp_state[CNTRL] = QUIT; // Signal that testing has ended for current motor
	do_qei_vector( tst_data_s ,c_tst ,p4_tst ,1 );

} // gen_motor_qei_test_data
/*****************************************************************************/
void gen_all_qei_test_data( // Generate QEI Test data
	streaming chanend c_tst, // Channel for sending test vecotrs to test checker
	port out p4_tst[]  // Array of ports on which to transmit test data
)
{
	GENERATE_QEI_TYP tst_data_s; // Structure of QEI test data
	int motor_cnt; // counts Motors 


	init_test_data( tst_data_s );

	// Loop through motors, so we can print results serially
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{
		init_motor_tests( tst_data_s ,motor_cnt ); // Initialise set of motor tests

		gen_motor_qei_test_data( tst_data_s ,c_tst ,p4_tst[motor_cnt] );
	} // for motor_cnt

	if (tst_data_s.print)
	{
		acquire_lock(); // Acquire Display Mutex
		printstrln( "Test Generation Ends " );
		release_lock(); // Release Display Mutex
	} // if (tst_data_s.print)
} // gen_all_qei_test_data
/*****************************************************************************/
