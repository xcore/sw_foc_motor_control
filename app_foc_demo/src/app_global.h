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
#ifndef _APP_GLOBAL_H_
#define _APP_GLOBAL_H_

#define USE_XSCOPE 1	// Define this to include XSCOPE support

#define CHECK_ERRORS 1	// Define this to switch on error checks

// Define the number of motors
#define NUMBER_OF_MOTORS 2

// This section to be used for specifying motor type ...

#define NUM_POLE_PAIRS	4 // Define the number of pole-pairs

// Define the number different QEI sensors (angular positions)
#ifdef FAULHABER_MOTOR
#define QEI_PER_REV (1024 * 4)
#else
#define QEI_PER_REV (256 * 4)
#endif

#define MAX_SPEC_RPM 4000 // Maximum specified motor speed

#endif /* _APP_GLOBAL_H_ */
