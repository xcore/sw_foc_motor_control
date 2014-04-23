
/******************************************************************************\
 * Description: This file is for XMOS internal use.
 * It allows an example application to be built and characterised.
 * (It contians definitions normally held in app_global.h)
 * NB These definitions will be passed to all modules used the example application
 *
 * Version: 0v1
 * Build:
 *
 * The copyrights, all other intellectual and industrial
 * property rights are retained by XMOS and/or its licensors.
 * Terms and conditions covering the use of this code can
 * be found in the XMOS End User License Agreement.
 *
 * Copyright XMOS Ltd 2013
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
\******************************************************************************/

#ifndef _MODULE_FOC_QEI_EXAMPLE_CONF_H_
#define _MODULE_FOC_QEI_EXAMPLE_CONF_H_

/** Define this to switch on error checks */
#define CHECK_ERRORS 1

/** Define the number of motors */
#define NUMBER_OF_MOTORS 2

/** Define the number of pole-pairs in motor */
#define NUM_POLE_PAIRS 4

/** Define the number of different QEI sensor positions per pole-pair */
#define QEI_PER_POLE 256

/**  Default number of QEI positions per Revolution */
#define QEI_PER_REV (QEI_PER_POLE * NUM_POLE_PAIRS)

/**  Seconds in a minute */
#define SECS_PER_MIN 60

/** Define Maximum specified motor speed. WARNING: Safety critical */
#define MAX_SPEC_RPM 4000

#endif // _MODULE_FOC_QEI_EXAMPLE_CONF_H_
