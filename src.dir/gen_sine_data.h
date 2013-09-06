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

#ifndef _GEN_SINE_DATA_H_
#define _GEN_SINE_DATA_H_

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <math.h>

#include "sine_common.h"

#define STR_LEN 256 // String size

typedef double R64;
typedef R64 SINE_T;

/** Pi */
#define PI ((SINE_T)3.141592653589793)

#endif /* _GEN_SINE_DATA_H_ */
