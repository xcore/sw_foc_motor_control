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

#ifndef __QEI_COMMON_H__
#define __QEI_COMMON_H__

#include "app_global.h"

#define QEI_CMD_DATA_REQ	1	// QEI Data Request Command code

#define HALF_QEI_CNT (QEI_PER_REV >> 1) // 180 degrees of rotation
#define QEI_REV_MASK (QEI_PER_REV - 1) // Mask used to force QEI count into base-range [0..QEI_REV_MASK] 

#endif /* __QEI_COMMON_H__ */
