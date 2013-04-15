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

#ifndef _CONTROL_COMMS_ETH_H_
#define _CONTROL_COMMS_ETH_H_

#include <xs1.h>
#include <assert.h>
#include <print.h>
#include <xccompat.h>

#include "ethernet_xtcp_server.h"
#include "uip_server.h"
#include "shared_io.h"

/*****************************************************************************/
void foc_comms_init_eth(	// The Ethernet & TCP/IP server core
	REFERENCE_PARAM( ethernet_xtcp_ports_t, ports ),  // Reference/pointer to structure containing ethernet ports
  REFERENCE_PARAM( xtcp_ipconfig_t, ipconfig ), // Reference/pointer  to structure containing IP addresses
  chanend c_ethernet[] // array of channels to the TCP/IP cores
);
/*****************************************************************************/
/** \brief Implement the high level Ethernet control server
 *
 * This control the motors based on commands from the ethernet/TCP stack
 *
 * \param c_commands Array of command channels for motors
 * \param tcp_svr channel to the TCP/IP thread
 */
void foc_comms_do_eth( chanend c_commands[], chanend tcp_svr );
/*****************************************************************************/

#endif /* _CONTROL_COMMS_ETH_H_ */
