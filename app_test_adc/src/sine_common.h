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

#ifndef _SINE_COMMON_H_
#define _SINE_COMMON_H_

#define SIN_RES_BITS 12 // Controls Resolution of Sine table
#define NUM_SIN_VALS (1 << SIN_RES_BITS) // Number of Sine table Entries
#define MAX_SIN_INDEX (NUM_SIN_VALS - 1) // Max. table index value

#define SIN_NAME "sine_table.bin" // name of file containing binary tabulated sine values

typedef unsigned short SIN_TYP;

#define SIN_VAL_BITS (sizeof(SIN_TYP) * BITS_IN_BYTE) // No. of bits in SIN_TYP

#define SIN_BUF_SIZE (NUM_SIN_VALS * sizeof(SIN_TYP)) // Total size of Sine-table (in bytes)

#endif /* _SINE_COMMON_H_ */
