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

#include "hall_server.h"

/*****************************************************************************/
static void init_hall_data( // Initialise Hall data structure for one motor
	int motor_id,	// Unique motor id
	HALL_DATA_TYP &hall_data_s, // Reference to structure containing HALL data for one motor
	unsigned &hall_buf // Reference to buffer of  raw hall data for one motor
)
{
	hall_data_s.id = motor_id; // Set unique motor id
	hall_data_s.params.hall_val = 0;
	hall_data_s.status_errs = 0; // Initialise counter for QEI status errors
	hall_buf = 0;

	return;
} // init_hall_data
/*****************************************************************************/
static void estimate_error_status( // Update estimate of error status based on new data
	HALL_DATA_TYP &hall_data_s, // Reference to structure containing HALL data for one motor
	unsigned new_err // Newly acquired error flag
)
// We require MAX_HALL_STATUS_ERR consecutive new errors before error estimate is set
{
	// Check if status changed
	if (new_err)
	{ // new error detected

		// Check previous error estimate
		if (0 == hall_data_s.params.err)
		{ // NO previous detected Error
			hall_data_s.status_errs++; // Increment new error count

			// Check if too many errors occured
			if (MAX_HALL_STATUS_ERR <=  hall_data_s.status_errs)
			{
				hall_data_s.params.err = 1; // Switch ON Error Estimate
			} // if (MAX_HALL_STATUS_ERR <=  hall_data_s.status_errs)
		} // if (0 == hall_data_s.params.err)
	} // if (new_err != hall_data_s.prev_err)
	else
	{ // NO new error detected

		// Check previous error estimate
		if (hall_data_s.params.err)
		{ // Already detected Error
			hall_data_s.status_errs--; // Decrement new error count

			// Check if all errors cleared
			if (0 >=  hall_data_s.status_errs)
			{
				hall_data_s.params.err = 0; // Switch OFF Error Estimate
			} // if (0 >=  hall_data_s.status_errs)
		} // if (hall_data_s.params.err)
	} // if (new_err != hall_data_s.prev_err)

} // estimate_error_status
/*****************************************************************************/
static void service_hall_input_pins( // Process new Hall data
	HALL_DATA_TYP &hall_data_s, // Reference to structure containing HALL data for one motor
	unsigned inp_pins // Set of raw data values on input port pins
)
{
	unsigned phase_val; // Raw phase value on input port pins
	unsigned err_flg; // Flag set when Error condition detected


// printstr("S1:"); printintln( hall_data_s.params.hall_val );
	phase_val = inp_pins & HALL_PHASE_MASK; // Mask out phase bits of Hall Sensor data
	err_flg = !(inp_pins & HALL_NERR_MASK); 	// NB Bit_3=0, and err_flg=1, if error detected, 

	// Update estimate of error status using new error flag
	estimate_error_status( hall_data_s ,err_flg ); // NB Updates hall_data_s.params.err

//MB~ TODO: Insert filter here

	hall_data_s.params.hall_val = phase_val; // NB Filtering not yet implemented
// printstr("S2:"); printintln( hall_data_s.params.hall_val );

	return;
} // service_hall_input_pins
/*****************************************************************************/
static void service_hall_client_request( // Send processed HALL data to client
	HALL_DATA_TYP &hall_data_s, // Reference to structure containing HALL data for one motor
	streaming chanend c_hall // Data channel to client (carries processed HALL data)
)
{
// printstr("S5:"); printintln( hall_data_s.params.hall_val );
	c_hall <: hall_data_s.params;
// printstr("S6:"); printintln( hall_data_s.params.hall_val );

	return;
} // service_hall_client_request
/*****************************************************************************/
void foc_hall_do_multiple( // Get Hall Sensor data from motor and send to client
	streaming chanend c_hall[], // Array of data channels to client (carries processed Hall data)
	port in p4_hall[]					// Array of input port (carries raw Hall motor data)
)
{
	HALL_DATA_TYP all_hall_data[NUMBER_OF_MOTORS]; // Array of structure containing HALL data for one motor
	unsigned hall_bufs[NUMBER_OF_MOTORS]; // Buffer array of raw hall data from input port pins for each motor
	int motor_cnt; // Counts number of motors


	// Initialise Hall data for each motor
	for (motor_cnt=0; motor_cnt<NUMBER_OF_MOTORS; motor_cnt++)
	{ 
		init_hall_data( motor_cnt ,all_hall_data[motor_cnt] ,hall_bufs[motor_cnt] );
	} // for motor_cnt

	// Loop forever
	while (1) {
#pragma xta endpoint "hall_main_loop"
#pragma ordered // If multiple cases fire at same time, service top-most first
		select {
			// Service any change on input port pins
			case (int motor_id=0; motor_id<NUMBER_OF_MOTORS; motor_id++) p4_hall[motor_id] when pinsneq(hall_bufs[motor_id]) :> hall_bufs[motor_id] :
			{
// printint(motor_id); printstr(":S0:"); printintln( all_hall_data[motor_id].params.hall_val );
				service_hall_input_pins( all_hall_data[motor_id] ,hall_bufs[motor_id] );
// printint(motor_id); printstr(":S3:"); printintln( all_hall_data[motor_id].params.hall_val );
			} // case
			break;

			// Service any client request for data
			case (int motor_id=0; motor_id<NUMBER_OF_MOTORS; motor_id++) c_hall[motor_id] :> int :
			{
// printint(motor_id); printstr(":S4:"); printintln( all_hall_data[motor_id].params.hall_val );
				service_hall_client_request( all_hall_data[motor_id] ,c_hall[motor_id] );
// printint(motor_id); printstr(":S7:"); printintln( all_hall_data[motor_id].params.hall_val );
			} // case
			break;
		} // select
	}	// while (1)

	return;
} // foc_hall_do_multiple
/*****************************************************************************/
