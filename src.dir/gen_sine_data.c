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

#include "gen_sine_data.h"

/* Generate File of Tabulated Sine data for 1st quadrant [0 .. PI/2] and write to file.
 * Each table entry is scaled by 65535 and stored as a 16-bit value
 * The data is written to a binary file.
 */
/*****************************************************************************/
void create_sine_table( // Populates table array with sine values
	SIN_TYP sine_tab[] // Array of tabulated Sine values
)
{
	int tab_cnt; // counts table entries
	SINE_T ang_off; // angle offset in radians
	SINE_T ang_val; // angle in radians
	SINE_T ang_inc; // angle increment in radians
	SINE_T sine_val; // sine value


	ang_off = PI / (SINE_T)(NUM_SIN_VALS << 2); // Initialise angular offset
	ang_val = ang_off; // Initialise angle
	ang_inc = 2 * ang_off; // Initialise angular increment

	// loop through table entries
	for (tab_cnt=0; tab_cnt<NUM_SIN_VALS; tab_cnt++)
	{
		sine_val = (SINE_T)sinl( ang_val ); // Get double-precision sine value
		sine_tab[tab_cnt] = (SIN_TYP)(sine_val * (SINE_T)SIN_SCALE + (SINE_T)0.5); // Scale and round

		ang_val += ang_inc; // Increment angle
	} // for tab_cnt

} // create_sine_table 
/*****************************************************************************/
int dump_table_to_file( // Dumps table array to binary file
	SIN_TYP sine_tab[] // Array of tabulated Sine values
)
{
	int file_id; // file identifier
	ssize_t written_bytes; // returned No. of bytes written


	// Open binary file for writing (create if necessary)
	file_id = open( SIN_NAME ,(O_WRONLY | O_CREAT) );
  if (-1 == file_id)
  {
    printf("ERROR: Opening File %s\n" ,SIN_NAME );
    return -1;
  } // if (file_id == NULL)

	// Write sine table to binary file, starting with 1st entry
	written_bytes = write( file_id ,&(sine_tab[0]) ,SIN_BUF_SIZE );
	if (SIN_BUF_SIZE != written_bytes)
	{
    printf("ERROR: Only %d of %d bytes written\n" ,(int)written_bytes ,(int)SIN_BUF_SIZE );
 	  return -1;
	} // if (SIN_BUF_SIZE != written_bytes)

	// Close binary file
	if (0 != close(file_id))
  {
    printf("ERROR: Closing File %s\n" ,SIN_NAME );
    return -1;
  } // if (0 != fclose(file_id))

 	return 0;
} // dump_table_to_file 
/*****************************************************************************/
int main( 
)
{
	SIN_TYP sine_tab[NUM_SIN_VALS]; // Array of tabulated Sine values


	create_sine_table( sine_tab );

	dump_table_to_file( sine_tab );
	
	return 0;
} // main
/*****************************************************************************/
