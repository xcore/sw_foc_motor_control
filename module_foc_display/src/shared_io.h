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

#ifndef _SHARED_IO_H_
#define _SHARED_IO_H_

#include <stdio.h> // Required for sprintf()

#include <xs1.h>
#include <assert.h>
#include <print.h>
#include <xccompat.h>

#include "app_global.h"
#include "lcd.h"

#ifndef PLATFORM_REFERENCE_MHZ
	#error Define. PLATFORM_REFERENCE_MHZin app_global.h
#endif

#ifndef MIN_STALL_RPM
	#error Define. MIN_STALL_RPM in app_global.h
#endif // MIN_STALL_RPM

#ifndef MAX_SPEC_RPM
	#error Define. MAX_SPEC_RPM in app_global.h
#endif // MAX_SPEC_RPM

#define ERR_LIM 10 // No. of consecutive button value errors allowed

#define STEP_SPEED 		50 // Speed increment per button press

// NB 250 MHz Reference Clock has 4ns/tick (See .xn file)
#define UPDATE_WAIT_TIME 8 // time interval (in milli-secs) between speed updates
#define UPDATE_WAIT_TICKS (UPDATE_WAIT_TIME * PLATFORM_REFERENCE_KHZ)// time interval (in ticks) between speed updates

/** Different IO Commands */
typedef enum CMD_IO_ETAG
{
	IO_CMD_GET_VALS	= 1,
	IO_CMD_GET_IQ,
	IO_CMD_SET_SPEED,
	IO_CMD_DIR,
	IO_CMD_GET_VALS2,
	IO_CMD_GET_FAULT,
  NUM_IO_CMDS    // Handy Value!-)
} CMD_IO_ENUM;

/*****************************************************************************/
/** \brief Manages the display, buttons and shared ports.
 * \param c_speed // Display channel between Client & Server
 * \param lcd_interface_s // Reference/Pointer to structure containing data for LCD display 
 * \param btns // Input port buttons
 * \param leds // Output port LED's
 */
void foc_display_shared_io_manager( // Manages the display, buttons and shared ports.
	chanend c_speed[], // Display channel between Client & Server
	REFERENCE_PARAM( LCD_INTERFACE_TYP, lcd_interface_s ), // Reference/Pointer to structure containing data for LCD display 
	in port btns, // Input port buttons
	out port leds // Output port LED's
);
/*****************************************************************************/

#endif // _SHARED_IO_H_
