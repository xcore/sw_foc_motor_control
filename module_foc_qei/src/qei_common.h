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

#ifndef _QEI_COMMON_H_
#define _QEI_COMMON_H_

#ifndef QEI_PER_REV 
	#error Define. QEI_PER_REV in app_global.h
#endif // QEI_PER_REV

#define QEI_REV_MASK (QEI_PER_REV - 1) // Mask used to force QEI count into base-range [0..QEI_REV_MASK] 

// QEI Command Codes (Client --> Server) 
#define QEI_CMD_DATA_REQ	1	// QEI Data Request

#endif /* _QEI_COMMON_H_ */
