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

#ifndef _GENERATE_QEI_TESTS_H_
#define _GENERATE_QEI_TESTS_H_

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "app_global.h"
#include "test_qei_common.h"

#define QEI_CYCLE_BITS 2
#define QEI_CYCLE_LEN (1 << QEI_CYCLE_BITS) 
#define QEI_CYCLE_MSK (QEI_CYCLE_LEN - 1)

typedef struct QEI_CYC_TAG // Structure containing Array of QEI values
{
	int vals[QEI_CYCLE_LEN];	// Array of QEI values
} QEI_CYC_TYP;

typedef struct TEST_QEI_TAG // Structure containing Error handling data
{
	QEI_CYC_TYP f_cyc;	// Structure of QEI cycle values for forward rotation
	QEI_CYC_TYP r_cyc;	// Structure of QEI cycle values for reverse rotation
	int cyc_off;			// Offset into QEI cycle
} TEST_QEI_TYP;


/*****************************************************************************/
void gen_all_qei_test_data( // Generate QEI Test data for all motors
	port out p4_tst[]  // Array of ports on which to transmit test data
);
/*****************************************************************************/
#endif /* _GENERATE_QEI_TESTS_H_ */
