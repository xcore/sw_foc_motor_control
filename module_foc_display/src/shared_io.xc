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

#include "shared_io.h"

#define ASJ 1

/*****************************************************************************/

#if (1 == ASJ) // Start-of Andrew_SJ's test-code

#include <stdlib.h>

#define MAX_TEST_RPM 4000
#define MIN_TEST_RPM 400
#define SPEED_2000 2000
#define EASY_SPEED 1000
#define LO_SPEED_INC 50
#define HI_SPEED_INC 100

#define FIRST_MOTOR 0 //MB~
#define LAST_MOTOR 0 //MB~

/*****************************************************************************/
static void wait(unsigned millis){
    timer t;
    unsigned time;
    t:> time;
    t when timerafter(time+millis*100000) :> int;
}
/*****************************************************************************/
static void stop(chanend c_speed[]){

	int motor_cnt;


	for (motor_cnt=FIRST_MOTOR; motor_cnt<=LAST_MOTOR; motor_cnt++)
	{
		c_speed[motor_cnt] <: IO_CMD_SET_SPEED;
		c_speed[motor_cnt] <: 0;
	} // for motor_cnt

	wait(300);
}
/*****************************************************************************/
static void set_both_motors_speed(chanend c_speed[], int rpm){

	int motor_cnt;


	for (motor_cnt=FIRST_MOTOR; motor_cnt<=LAST_MOTOR; motor_cnt++)
	{
		c_speed[motor_cnt] <: IO_CMD_SET_SPEED;
		c_speed[motor_cnt] <: rpm;
	} // for motor_cnt
}
/*****************************************************************************/
static void print_speed(chanend c_speed[])
{
	int meas_vel; // Measured velocity
	int req_vel; // Requested velocity
	int motor_cnt;


	for (motor_cnt=FIRST_MOTOR; motor_cnt<=LAST_MOTOR; motor_cnt++)
	{
		c_speed[motor_cnt] <: IO_CMD_GET_IQ;
		c_speed[motor_cnt] :> meas_vel;
		c_speed[motor_cnt] :> req_vel;

		printint(motor_cnt); printstr(":"); printint(meas_vel); printstr("\t"); 
	} // for motor_cnt

	printstrln(" ");
}
/*****************************************************************************/
static void print_asj_speed(chanend c_speed[], int target)
{
	int meas_vel; // Measured velocity
	int req_vel; // Requested velocity
	int motor_cnt;

	printint(target); printstr(" ->\t");
	for (motor_cnt=FIRST_MOTOR; motor_cnt<=LAST_MOTOR; motor_cnt++)
	{
		c_speed[motor_cnt] <: IO_CMD_GET_IQ;
		c_speed[motor_cnt] :> meas_vel;
		c_speed[motor_cnt] :> req_vel;

		printint(motor_cnt); printstr(":"); printint(meas_vel); printstr("\t"); 
	} // for motor_cnt

	printstrln(" ");
}
/*****************************************************************************/
static void test_motor( chanend c_speed[])
{
	printstrln("Start Motor Tests");
	wait(3000);
	stop(c_speed);

	printstrln("Test 1: LDO_MOTOR_STARTUP_PLUS (10s)");
	set_both_motors_speed(c_speed, EASY_SPEED);
	wait(3000);
	print_speed( c_speed );

	set_both_motors_speed(c_speed, MAX_TEST_RPM);
	wait(3000);
	print_speed( c_speed );
	printstrln("End of Test 1");

	stop(c_speed);

	printstrln("Test 2: LDO_MOTOR_STARTUP_MINUS (10s)");
	set_both_motors_speed(c_speed, -EASY_SPEED);
	wait(3000);
	print_speed( c_speed );
	printstrln("End of Test 2");

	stop(c_speed);

	printstrln("Test 3: LDO_RPM_PLUS_400 (15s)");
	set_both_motors_speed(c_speed, EASY_SPEED);
	wait(3000);
	print_speed( c_speed );
	for(unsigned speed = (EASY_SPEED - LO_SPEED_INC); speed >= MIN_TEST_RPM; speed -= LO_SPEED_INC)
		{
	  set_both_motors_speed(c_speed, speed);
	  wait(1000);
			print_speed( c_speed );
	}
	printstrln("End of Test 3");

	stop(c_speed);

	printstrln("Test 4: LDO_RPM_MINUS_400 (15s)");
	set_both_motors_speed(c_speed, -EASY_SPEED);
	wait(3000);
	print_speed( c_speed );
	for(unsigned speed = (LO_SPEED_INC - EASY_SPEED); speed <= MIN_TEST_RPM; speed += LO_SPEED_INC)
		{
	  set_both_motors_speed(c_speed, speed);
	  wait(1000);
			print_speed( c_speed );
	}
	printstrln("End of Test 4");

	stop(c_speed);

	printstrln("Test 5: LDO_RPM_PLUS_4000 (15s)");
	set_both_motors_speed(c_speed, EASY_SPEED);
	wait(3000);
	print_speed( c_speed );
	for(unsigned speed = (EASY_SPEED + HI_SPEED_INC); speed <= MAX_TEST_RPM;speed += HI_SPEED_INC)
		{
	  set_both_motors_speed(c_speed, speed);
	  wait(1000);
			print_speed( c_speed );
	}
	printstrln("End of Test 5");

	stop(c_speed);

	printstrln("Test 6: LDO_RPM_MINUS_4000 (15s)");
	set_both_motors_speed(c_speed, -EASY_SPEED);
	wait(3000);
	print_speed( c_speed );
	for(unsigned speed = (-EASY_SPEED - HI_SPEED_INC); speed >= -MAX_TEST_RPM; speed -= HI_SPEED_INC)
		{
	  set_both_motors_speed(c_speed, speed);
	  wait(1000);
			print_speed( c_speed );
	}
	printstrln("End of Test 6");

	stop(c_speed);

	printstrln("Test 7: LDO_RPM_PLUS_400_STARTUP (8s)");
	set_both_motors_speed(c_speed, MIN_TEST_RPM);
	wait(8000);
	print_speed( c_speed );
	printstrln("End of Test 7");

	stop(c_speed);

	printstrln("Test 8: LDO_RPM_MINUS_400_STARTUP (8s)");
	set_both_motors_speed(c_speed, -MIN_TEST_RPM);
	wait(8000);
	print_speed( c_speed );
	printstrln("End of Test 8");

	stop(c_speed);

	printstrln("Test 9: LDO_RPM_PLUS_4000_STARTUP (8s)");
	set_both_motors_speed(c_speed, MAX_TEST_RPM);
	wait(3000);
	print_speed( c_speed );
	printstrln("End of Test 9");

	stop(c_speed);

	printstrln("Test 10: LDO_RPM_MINUS_4000_STARTUP (8s)");
	set_both_motors_speed(c_speed, -MAX_TEST_RPM);
	wait(3000);
	print_speed( c_speed );
	printstrln("End of Test 10");

	stop(c_speed);

	printstrln("Test 11: LDO_CHANGE_DIRECTION_POS2NEG(6s)");
	set_both_motors_speed(c_speed, MAX_TEST_RPM);
	wait(3000);
	print_speed( c_speed );
	set_both_motors_speed(c_speed, -MAX_TEST_RPM);
	wait(3000);
	print_speed( c_speed );
	printstrln("End of Test 11");

	printstrln("Test 12: LDO_CHANGE_DIRECTION_NEG2POS (6s)");
	set_both_motors_speed(c_speed, MAX_TEST_RPM);
	wait(3000);
	print_speed( c_speed );
	printstrln("End of Test 12");

	stop(c_speed);

		// Set back to Easy Speed before exit
		set_both_motors_speed(c_speed, EASY_SPEED);
	wait(3000);

	printstrln(" ");
	printstrln("End Of Motor Tests");

	return;
} // test_motor 
/*****************************************************************************/
void pretty_print_speed(int speed){
    printstr("Motor speed: ");
    printint(speed);
    printstrln(" RPM");
}

static void demo_motor( chanend c_speed[])
{
    set_both_motors_speed(c_speed, 1000);
    wait(10000);
    print_asj_speed( c_speed, 1000);
    set_both_motors_speed(c_speed, 1000);
    wait(10000);
    print_asj_speed( c_speed, 1000);
#define MAX_TEST_RPM 2000
    while(1){

        //////////////////////////////////////////////////////////////
        set_both_motors_speed(c_speed, EASY_SPEED);
        pretty_print_speed(EASY_SPEED);
        wait(3000);

        set_both_motors_speed(c_speed, MAX_TEST_RPM);
        pretty_print_speed(MAX_TEST_RPM);
        wait(15000);

        stop(c_speed);
        wait(1000);
        pretty_print_speed(0);

        //////////////////////////////////////////////////////////////
        set_both_motors_speed(c_speed, -EASY_SPEED);
        pretty_print_speed(-EASY_SPEED);
        wait(3000);

        set_both_motors_speed(c_speed, -MAX_TEST_RPM);
        pretty_print_speed(-MAX_TEST_RPM);
        wait(15000);

        stop(c_speed);
        wait(1000);
        pretty_print_speed(0);

        //////////////////////////////////////////////////////////////
        set_both_motors_speed(c_speed, MAX_TEST_RPM);
        pretty_print_speed(MAX_TEST_RPM);
        wait(15000);

        set_both_motors_speed(c_speed, -MAX_TEST_RPM);
        pretty_print_speed(-MAX_TEST_RPM);
        wait(15000);

        stop(c_speed);
        wait(1000);
        pretty_print_speed(0);

        //////////////////////////////////////////////////////////////
        set_both_motors_speed(c_speed, EASY_SPEED);
        wait(1000);
        for(int i=EASY_SPEED;i > 220; i-= 20){
            pretty_print_speed(i);
            set_both_motors_speed(c_speed, i);
            wait(1000);
            print_asj_speed( c_speed, i);
        }
        for(int i=220;i >= 120; i-=4){
            pretty_print_speed(i);
            set_both_motors_speed(c_speed, i);
            wait(2000);
            print_asj_speed( c_speed, i);
        }
        wait(10000);
    }


    return;
} // demo_motor
/*****************************************************************************/
static void dbg_motor(
	chanend c_speed[NUMBER_OF_MOTORS]
)
{
	int cur_speed; // Current Speed


	printstrln("Debug Motor Tests");
	wait(3000);
	stop(c_speed);

	set_both_motors_speed(c_speed ,MAX_TEST_RPM );
	wait(3000);
	stop(c_speed);

while(1)
{
	set_both_motors_speed(c_speed ,MAX_TEST_RPM );
	wait(3000);
//	print_speed( c_speed );
} // while

	return;
} // dbg_motor
/*****************************************************************************/

#endif // ( 1 == ASJ) End-of Andrew_SJ's test-code

/*****************************************************************************/
static void update_speed_control( // Updates the speed control loop
	chanend c_speed[], // speed channel
	CMD_IO_ENUM cmd_id // Command identifier
)
{
	int motor_cnt; // motor counter


	// Loop through motors
	for (motor_cnt=FIRST_MOTOR; motor_cnt<=LAST_MOTOR; motor_cnt++) 
	{
		c_speed[motor_cnt] <: cmd_id;
	} // for motor_cnt
} // update_speed_control 
/*****************************************************************************/
void foc_display_shared_io_manager( // Manages the display, buttons and shared ports.
	chanend c_speed[], // Display channel between Client & Server
	LCD_INTERFACE_TYP &lcd_interface_s, // Reference to structure containing data for LCD display 
	in port btns, // Input port buttons
	out port leds // Output port LED's
)
{
	char my_string[50]; // array of display characters
	int new_meas_speed[NUMBER_OF_MOTORS]; // Array containing new measured speeds
	int old_meas_speed[NUMBER_OF_MOTORS]; // Array containing old measured speeds
	int fault[NUMBER_OF_MOTORS]; // Array containing motor fault ids
	int new_req_speed[NUMBER_OF_MOTORS]; // new requested speed
	int old_req_speed[NUMBER_OF_MOTORS] = { 1000 ,-1000 }; // old requested speed
	int speed_change; // flag set when new speed parameters differ from old
	unsigned int btn_en = 0; // button debounce counter
	int motor_cnt = -1; // motor counter
	unsigned btns_val; // value that encodes state of buttons
	int err_cnt; // error counter

	timer timer_10Hz; // 10Hz timer
	unsigned int time_10Hz_val; // 10Hz timer value


	// Initialise array of old measured speeds
	for (motor_cnt=FIRST_MOTOR; motor_cnt<=LAST_MOTOR; motor_cnt++)
	{
		old_meas_speed[motor_cnt]=0;
	} // for motor_cnt

	leds <: 0;

	/* Initiate the LCD ports */
	lcd_ports_init( lcd_interface_s );

	/* Output the default value to the port */
	lcd_interface_s.p_core1_shared <:0;

	/* Get the initial time value */
	timer_10Hz :> time_10Hz_val;

//	demo_motor( c_speed ); //ASJ~

	/* Loop forever processing commands */
	while (1)
	{
		select
		{
		/* Timer event at 10Hz */
			case timer_10Hz when timerafter(time_10Hz_val + 10000000) :> time_10Hz_val:
			{
				speed_change = 0; // Preset to speed change

				/* Get the motor speeds from channels. NB Do this as quickly as possible */
				for (motor_cnt=FIRST_MOTOR; motor_cnt<=LAST_MOTOR; motor_cnt++) 
				{
					c_speed[motor_cnt] <: IO_CMD_GET_IQ;
					c_speed[motor_cnt] :> new_meas_speed[motor_cnt];
					c_speed[motor_cnt] :> new_req_speed[motor_cnt];

					c_speed[motor_cnt] <: IO_CMD_GET_FAULT;
					c_speed[motor_cnt] :> fault[motor_cnt];

					// Check for speed change
					if (new_req_speed[motor_cnt] != old_req_speed[motor_cnt])
					{ 
						speed_change = 1; // flag speed change
					} // if (new_req_speed[motor_cnt] != old_req_speed[motor_cnt])

					sprintf( my_string ," SetVeloc%1d:%5d RPM\n" ,motor_cnt ,new_req_speed[motor_cnt] );
					lcd_draw_text_row( my_string ,(2*motor_cnt) ,lcd_interface_s );

					// Filter estimated velocity 
					old_meas_speed[motor_cnt] = (old_meas_speed[motor_cnt] + new_meas_speed[motor_cnt]) >> 1;

					if (fault[motor_cnt]) 
					{
						sprintf(my_string, "  Motor%1d: FAULT = %02d\n" ,motor_cnt ,fault[motor_cnt] );
					} 
					else 
					{
						sprintf(my_string, " EstVeloc%1d:%5d RPM\n" ,motor_cnt ,old_meas_speed[motor_cnt] );
					}

					lcd_draw_text_row( my_string ,(2*motor_cnt + 1) ,lcd_interface_s );

					old_req_speed[motor_cnt] = new_req_speed[motor_cnt]; // Store for next iteration
				} // for motor_cnt

#ifdef DEPRECIATED
				if (speed_change) 
				{
					if (1 == USE_CAN)
					{
						lcd_draw_text_row( "  XMOS Demo 2013: CAN\n" ,0 ,lcd_interface_s );
					} // if (1 == USE_CAN)

					if (1 == USE_ETH)
					{
						lcd_draw_text_row( "  XMOS Demo 2013: ETH\n" ,0 ,lcd_interface_s );
					} // if (1 == USE_ETH)
				} // if (speed_change)
#endif // DEPRECIATED

				if ( btn_en > 0) btn_en--; // decrement switch debouncer
			}
			break; // case timer_10Hz when timerafter(time_10Hz_val + 10000000) :> time_10Hz_val:

			case !btn_en => btns when pinsneq(btns_val) :> btns_val:
				btns_val = (~btns_val & 0x0000000F); // Invert and mask out 4 most LS (active) bits

				// check for button change
				if (btns_val)
				{
					// Decode buttons value
					switch( btns_val )
					{
						case 1 : // Increase the speed, by the increment
							err_cnt = 0; // Valid button value so clear error count
							leds <: 1;

							update_speed_control( c_speed ,IO_CMD_INC_SPEED ); // Increase speed
						break; // case 1
	
						case 2 : // Decrease the speed, by the increment
							err_cnt = 0; // Valid button value so clear error count
							leds <: 2;

							update_speed_control( c_speed ,IO_CMD_DEC_SPEED ); // Decrease speed
						break; // case 2
	
#if (1 == ASJ) // Call Andrew_SJ's test-code
						case 4 : // Test tests
							err_cnt = 0; // Valid button value so clear error count
							leds <: 3;

test_motor( c_speed ); //MB~
//	dbg_motor( c_speed ); //MB~
						break; // case 4
#endif // ( 1 == ASJ)
	
						case 8 : // Change direction of spin
							err_cnt = 0; // Valid button value so clear error count
							leds <: 4;
			
							update_speed_control( c_speed ,IO_CMD_FLIP_SPIN ); // Reverse spin direction
						break; // case 8
	
				    default: // btns_val unsupported
							assert(err_cnt < ERR_LIM); // Check for persistant error

							err_cnt++; // Increment error count
				    break;
					} // switch( btns_val )

					btn_en = 2;	// Set the debouncer
				} // if (btns_val)
				else
				{	// No change
					err_cnt = 0; // Valid button value so clear error count
					leds <: 0;
				} // else !(btns_val)
			break; // case !btn_en => btns when pinsneq(btns_val) :> btns_val:
		} // select
	} // while (1)
} // display_shared_io_manager
/*****************************************************************************/

