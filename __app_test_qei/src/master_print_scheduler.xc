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

#include "master_print_scheduler.h"

/*****************************************************************************/
void master_print( // Collects and schedules display data
	const COMMON_TST_TYP &comm_data_s, // Structure containing common test data
	streaming chanend c_slave[NUM_DISP_SLAVES], // Array of channels for sending data to Display cores
	streaming chanend c_inputs[MB_FIXME]	// Array of channel for receiving display data
)
{
	TEST_VECT_TYP vect; // Structure containing QEI test vector to be printed
	QEI_PARAM_TYP params;	// Structure containing QEI parameters to be printed
	QEI_RAW_TYP qei_val;  // QEI value  to be printed
	DISP_CLASS_ENUM_TYP disp_class; // Signals which print-class to receive
	unsigned chan_off = 0; // slave channel offset


	// loop forever
	while(1)
	{
		select
		{
			// Test for new data on one of display inputs
			case (int inp_id=0; inp_id<MB_FIXME; inp_id++) c_inputs[inp_id] :> disp_class :
				// new data detected on channel inp_id

				c_slave[chan_off] <: disp_class; // Signal print-class to slave_print core

				// Determine print class
				switch(disp_class)
				{
					case DISP_CLASS_VECT : // Test vectore
						c_inputs[inp_id] :> vect;
						c_slave[chan_off] <: inp_id; // Signal print-source to slave_print core
						c_slave[chan_off] <: vect; // Send test-vector data to slave_print core
					break; // case DISP_CLASS_VECT

					case DISP_CLASS_GENR8 : // Generated test data
						c_inputs[inp_id] :> qei_val;
						c_slave[chan_off] <: inp_id; // Signal print-source to slave_print core
						c_slave[chan_off] <: qei_val; // Send test-vector data to slave_print core
					break; // case DISP_CLASS_GENR8

					case DISP_CLASS_CHECK : // Parameters to Check
						c_inputs[inp_id] :> params;
						c_slave[chan_off] <: inp_id; // Signal print-source to slave_print core
						c_slave[chan_off] <: params; // Send parameter data to slave_print core
					break; // case DISP_CLASS_CHECK

					case DISP_CLASS_PROG : // Progress indicator
						// Nothing to do
					break; // case DISP_CLASS_PROG

					default :
						acquire_lock(); // Acquire Display Mutex
						printstr("ERROR: Unexpected Print Class found : ");
						printuintln(disp_class);
						release_lock(); // Release Display Mutex
						assert( 0 == 1 ); // ERROR: Should never happen
					break; // default
				} // switch(disp_class)
				// new QEI data written to buffer
			break; // case c_inputs[inp_id]
		} // select

		// Update circular channel offset
		chan_off++; // Increment channel counter
		chan_off = (chan_off & DISP_SLAVE_MASK); // Wrap offset into range [0..DISP_SLAVE_MASK];
	} // while(1)

} // master_print
/*****************************************************************************/
