/*
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
 */

#ifndef _CONTROL_COMMS_CAN_H_
#define _CONTROL_COMMS_CAN_H_

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "CanIncludes.h"
#include "CanPhy.h"
#include "CanFunctions.h"
#include "shared_io.h"

/** Max. value for packet count */
#define COUNTER_MASK  0xfff

/*****************************************************************************/
/**
 *  \brief Initialise CAN Interface
 *
 *  Use it in conjunction with the thread 'canPhyRxTx' from the module module_can.
 *
 *  \param rxChan Channel connecting to the rxChan port
 *  \param txChan Channel connecting to the txChan port
 *	\param p_can_clk  CAN clock
 *	\param p_can_rx CAN 32-Input port
 *	\param p_can_tx CAN Output port
 *	\param p_shared_rs CAN reset port
 */
void foc_comms_init_can( // Initialise CAN Interface
	chanend c_rxChan, 						// CAN Receive Channel
	chanend c_txChan, 						// CAN Transmit Channel
	clock p_can_clk,							// CAN clock
	buffered in port:32 p_can_rx,	// CAN 32-Input port
	port p_can_tx,  							// CAN Output port
	out port p_shared_rs					// CAN reset port
	);
/*****************************************************************************/
/**
 *  \brief This is a thread which performs the higher level control for the CAN interface.
 *
 *  Use it in conjunction with the thread 'canPhyRxTx' from the module module_can.
 *
 *  \param c_commands Channel array for interfacing to the motors
 *  \param rxChan Connect to the rxChan port on the canPhyRxTx
 *  \param txChan Connect to the txChan port on the canPhyRxTx
 */
void foc_comms_do_can( chanend c_commands[], chanend rxChan, chanend txChan);
/*****************************************************************************/

#endif /* _CONTROL_COMMS_CAN_H_ */

