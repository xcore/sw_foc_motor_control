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

/*****************************************************************************\
This WatchDog protects a specific selection of motors from a lerger set.
There is an array of flags, one for each motor in the larger set.
For each flag, it is set to one if the motor needs protecting, zero otherwise.

WARNING: The user must set these flags (and update the checksum), 
at the following location:-
  File: watchdog.xc
  Function: init_wd
  Structure: guard_prot_s
  Method: Initialiser

If any of the selected motors stop sending their 'ticks',
then the WatchDog fires (and switches off the power board)
The WatchDog ignores the behaviour of 'ticks' from UN-selected motors.
\*****************************************************************************/

#ifndef _WATCHDOG_H_
#define _WATCHDOG_H_

#include <xs1.h>
#include <assert.h>
#include <print.h>

#include "use_locks.h"

#include "app_global.h"

#define ALL_MOTOR_FLAGS ((1 << NUMBER_OF_MOTORS) - 1) // Set of motor flags e.g. 0b11 for 2 motors
#define MAX_WD_TICKS 10 // No. of ticks before tick count reset

#define ENABLE_MASK 1 // WatchDog Enable is Bit_0 on WD port
#define TICK_MASK 2 // WatchDog Tick is Bit_1 on WD port

#define HALF_TICK_PERIOD (MILLI_SEC >> 2) // 0.25 mill-secs

/** Different WatchDog Commands */
typedef enum CMD_WD_ETAG
{
	WD_CMD_INIT = 0, // Initialise WatchDog Circuit
	WD_CMD_DISABLE, // Actively switch off power to motors
	WD_CMD_TICK, // Keep motor running for another 3.4 ms
	WD_CMD_ACK, // Acknowledge Command from Control loop
  NUM_WD_CMDS    // Handy Value!-)
} CMD_WD_ENUM;

/** Different WatchDog States */
typedef enum WD_STATE_ETAG
{
	WD_UNARMED = 0,	// (Motor warming-up) WatchDog not NOT yet offering protection
	WD_ARMED,				// WatchDog offering protection
	WD_STOP,				// WatchDog must stop motors (Final State)
  NUM_WD_STATES 	// Handy Value!-)
} WD_STATE_ENUM;

/** Structure containing guard data, one flag for each motor */
typedef struct WD_GUARD_TAG // 
{
	int guards[NUMBER_OF_MOTORS]; // Array of flags used as guards
	int num; // Number of protected motors
} WD_GUARD_TYP;

typedef struct WD_DATA_TAG // 
{
	WD_STATE_ENUM state; // WatchDog State
	WD_GUARD_TYP protect_s; // Structure of guard data, indicating which motors are protected
	unsigned shared_out; // Clear (Bit_0 and Bit_1 of) shared output port
	unsigned time; // time when previous WatchDog tick occured
} WD_DATA_TYP;

/*****************************************************************************/
/** \brief Controls the WatchDog circuit (2 chips)
 *
 * The Watchdog circuit needs a constant stream of +ve edges to prevent it
 * from shutting down the motor.
 * 
 * The Watchdog circuit consists of the following 2 chips:
 *	STWD100NP: A WatchDog chip, requires a 1-bit signal WD_TICK (Bit_1 on WD port)
 *	NC7SZ175: A FlipFlop chip, requires a 1-bit signal WD_EN (Bit_0 on WD port)
 *
 * WD_TICK is the +ve edge stream.
 * WD_EN is used to ENABLE the WatchDog Circuit.
 * NB The only way to disable the WatchDog Circuit, is to Stop toggling WD_TICK
 * If the WatchDog becomes disabled, it can only be re-enabled by restarting the program main()
 *
 * \param c_wd  // Array of control channels for controlling the watchdog
 * \param wd  // control port for the watchdog device
 */
void foc_loop_do_wd( // Controls the WatchDog circuit (2 chips)
	chanend c_wd[NUMBER_OF_MOTORS], // Array of WatchDog channels
	out port p2_wd // 2-bit port used to control WatchDog circuit (2 chips)
);
/*****************************************************************************/
#endif /* _WATCHDOG_H_ */
