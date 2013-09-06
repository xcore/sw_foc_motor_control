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

#include "display_generator_data.h"

/*****************************************************************************/
static void print_qei_value( // Prints QEI value in binary format
	streaming chanend c_gen, // Channel for receiving generated QEI values
	char disp_str[], // Display string
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
			disp_str[bit_cnt] = '1';
		} // if (qei_val & bit_mask)
		else
		{
			disp_str[bit_cnt] = '0';
		} // if (qei_val & bit_mask)

		bit_mask <<= 1; // Right shift bit-mask by 1 bit
	} // for bit_cnt

	acquire_lock(); // Acquire Display Mutex
	printstr( "QEI=" ); printstrln(disp_str);
	release_lock(); // Release Display Mutex

} // print_qei_value
/*****************************************************************************/
void disp_gen_data( // Displays generated QEI test data
	streaming chanend c_gen // Channel for receiving generated QEI values
)
{
	QEI_RAW_TYP buffer[DISP_BUF_SIZ]; // Buffers QEI values
	char disp_str[(QEI_BITS + 1)]; // display string for QEI value
	int read_cnt = 0; // No of QEI values read from buffer
	int write_cnt = 0; // No of QEI values written to buffer
	unsigned read_off = 0; // read offset into buffer
	unsigned write_off = 0; // wtite offset into buffer
	int do_loop = 1;   // Flag set until loop-end condition found


	acquire_lock(); // Acquire Display Mutex
	printstrln("Display Core Starts");
	release_lock(); // Release Display Mutex

	disp_str[QEI_BITS] = 0; // Add string terminator

	// loop forever
	while(do_loop)
	{
		select
		{
			case c_gen :> buffer[write_off] : // Check for new QEI data
				// new QEI data written to buffer
				write_cnt++; // Increment write counter
				write_off = write_cnt & DISP_BUF_MASK; // Wrap into buffer range

				assert( (write_cnt - read_cnt) < DISP_BUF_MASK); // Check for buffer overflow
			break; // case c_gen

			default :
				// Check if any buffer data needs printing
				if (write_cnt > read_cnt)
				{
					// Check for loop termination
					if (QEI_CMD_LOOP_STOP == buffer[read_off])
					{
						do_loop = 0; // Signal loop to stop
					} // if (QEI_CMD_LOOP_STOP == buffer[read_off])
					else
					{
						print_qei_value( c_gen ,disp_str ,buffer[read_off] );
					} // else !(QEI_CMD_LOOP_STOP == buffer[read_off])

					read_cnt++; // Increment read counter
					read_off = read_cnt & DISP_BUF_MASK; // Wrap into buffer range
				} // if (write_off != read_off)
			break; // default
		} // select
	} // while(do_loop)

	acquire_lock(); // Acquire Display Mutex
	printstrln("");
	printstrln("Display Ends");
	release_lock(); // Release Display Mutex
} // disp_gen_data
/*****************************************************************************/
