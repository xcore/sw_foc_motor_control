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

#include "slave_print_data.h"

/*****************************************************************************/
static void print_qei_parameters( // Print QEI parameters
	const COMMON_TST_TYP &comm_data_s, // Structure containing common test data
	DISP_DATA_TYP &disp_data_s, // Reference to structure containing display data
	QEI_PARAM_TYP params	// Structure containing current QEI parameters (received from Client)
)
{
	acquire_lock(); // Acquire Display Mutex
	printstr( comm_data_s.prefix[disp_data_s.inp_id].str ); // Print prefix string

	printstr( "E=" );
	printint( params.err );
	printstr( ":  R=" );
	printint( params.rev_cnt );
	printstr( "  P=" );
	printint( params.theta );
	printstr( "  V=" );
	printint( params.veloc );
	printcharln(' ');
	release_lock(); // Release Display Mutex
} // print_qei_parameters
/*****************************************************************************/
static void print_qei_value( // Prints QEI value in binary format
	DISP_DATA_TYP &disp_data_s, // Reference to structure containing display data
	QEI_RAW_TYP qei_val // Input QEI value
)
{
	unsigned bit_mask; // used to mask out 1 bit of QEI value
	int bit_cnt; // Counter for QEI bits


	bit_mask = 1; // Set bit-mask to LS-bit

	for	(bit_cnt=(QEI_BITS-1); bit_cnt>-1; bit_cnt--)
	{
		// Check for active bit
		if (qei_val & bit_mask)
		{
			disp_data_s.qei_str[bit_cnt] = '1';
		} // if (qei_val & bit_mask)
		else
		{
			disp_data_s.qei_str[bit_cnt] = '0';
		} // if (qei_val & bit_mask)

		bit_mask <<= 1; // Right shift bit-mask by 1 bit
	} // for bit_cnt

	acquire_lock(); // Acquire Display Mutex
	printstr( "QEI=" ); printstrln(disp_data_s.qei_str);
	release_lock(); // Release Display Mutex

} // print_qei_value
/*****************************************************************************/
static void init_print( // Initialise data printing
	DISP_DATA_TYP &disp_data_s // Reference to structure containing display data
)
{
	disp_data_s.print_cnt = 1; // Initialise print counter

	disp_data_s.qei_str[QEI_BITS] = 0; // Add string terminator

} // init_print
/*****************************************************************************/
static void print_test_vector( // print test vector details
	const COMMON_TST_TYP &comm_data_s, // Structure containing common test data
	DISP_DATA_TYP &disp_data_s, // Reference to structure containing display data
	TEST_VECT_TYP vect // Structure containing test vector data to be printed
)
{
	int comp_cnt; // Counter for Test Vector components
	int comp_state; // state of current component of input test vector

	acquire_lock(); // Acquire Display Mutex
	printstr( comm_data_s.prefix[disp_data_s.inp_id].str ); // Print prefix string

	// Loop through NON-control test vector components
	for (comp_cnt=1; comp_cnt<NUM_VECT_COMPS; comp_cnt++)
	{
		comp_state = vect.comp_state[comp_cnt];  // Get state of current component

		if (comp_state < comm_data_s.comp_data[comp_cnt].num_states)
		{
			printstr( comm_data_s.comp_data[comp_cnt].state_names[comp_state].str ); // Print component status
		} //if (comp_state < comm_data_s.comp_data[comp_cnt].num_states)
		else
		{
			printcharln(' ');
			printstr( "ERROR: Invalid state. Found ");
			printint( comp_state );
			printstr( " for component ");
			printstrln( comm_data_s.comp_data[comp_cnt].comp_name.str );
			assert(0 == 1); // Force abort
		} //if (comp_state < comm_data_s.comp_data[comp_cnt].num_states)
	} // for comp_cnt

	printchar( ':' );
	comp_state = vect.comp_state[CNTRL];  // Get state of Control/Comms. status
	printstrln( comm_data_s.comp_data[CNTRL].state_names[comp_state].str ); // Print Control/Comms. status

	release_lock(); // Release Display Mutex
} // print_test_vector
/*****************************************************************************/
static void print_progress( // Print progress indicator
	DISP_DATA_TYP &disp_data_s // Reference to structure containing display data
)
{
	// Check for display-wrap
	if (PRINT_WID > disp_data_s.print_cnt)
	{
		printchar('.');
		disp_data_s.print_cnt++;
	} // if (PRINT_WID > disp_data_s.print_cnt)
	else
	{
		printcharln('.');
		disp_data_s.print_cnt = 1;
	} // if (PRINT_WID > disp_data_s.print_cnt)
} // print_progress
/*****************************************************************************/
void slave_print( // Displays generated QEI test data
	const COMMON_TST_TYP &comm_data_s, // Structure containing common test data
	streaming chanend c_master, // Channel for receiving generated print data from print-master core
	int disp_id // Unique Display core identifier
)
{
	DISP_DATA_TYP disp_data_s; // Structure containing display data
	TEST_VECT_TYP vect; // Structure containing QEI test vector to be printed
	QEI_PARAM_TYP params;	// Structure containing QEI parameters to be printed
	QEI_RAW_TYP qei_val;  // QEI value  to be printed
	DISP_CLASS_ENUM_TYP disp_class; // Signals which print-class to receive
	int do_loop = 1;   // Flag set until loop-end condition found


	init_print( disp_data_s );

	// loop forever
	while(do_loop)
	{
		c_master :> disp_class; // Get print class

		switch(disp_class)
		{
			case DISP_CLASS_GENR8 : // Generated test data
				c_master :> disp_data_s.inp_id; // Get display input identifier
				c_master :> qei_val;

				print_qei_value( disp_data_s ,qei_val );
			break; // case DISP_CLASS_GENR8

			case DISP_CLASS_CHECK : // Parameters to Check
				c_master :> disp_data_s.inp_id; // Get display input identifier
				c_master :> params;

				print_qei_parameters( comm_data_s ,disp_data_s ,params );
			break; // case DISP_CLASS_CHECK

			case DISP_CLASS_VECT : // Test vectore
				c_master :> disp_data_s.inp_id; // Get display input identifier
				c_master :> vect; // Get vector data

				print_test_vector( comm_data_s ,disp_data_s ,vect );
			break; // case DISP_CLASS_VECT

			case DISP_CLASS_PROG : // Progress indicator
				print_progress( disp_data_s );
			break; // case DISP_CLASS_VECT

			default :
				acquire_lock(); // Acquire Display Mutex
				printstr("ERROR: Unexpected Print Class found : ");
				printuintln(disp_class);
				release_lock(); // Release Display Mutex
				assert( 0 == 1 ); // ERROR: Should never happen
			break; // default
		} // switch(disp_class)
	} // while(do_loop)

} // slave_print
/*****************************************************************************/
