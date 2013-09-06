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

#include "adc_7265.h"

/*****************************************************************************/
static void init_adc_phase( // Initialise the data for this phase of one ADC trigger
	ADC_PHASE_TYP &phase_data_s // Reference to structure containing data for this phase of one ADC trigger
)
{
	phase_data_s.mean = 0;
	phase_data_s.filt_val = 0;
	phase_data_s.adc_val = 0;
	phase_data_s.coef_err = 0;
	phase_data_s.scale_err = 0;

} // init_adc_phase
/*****************************************************************************/
static void gen_filter_params( // Generates required filter parameters from 'inp_bits'
	ADC_FILT_TYP &filt_s, // Reference to structure containing filter parameters
	int inp_bits // used to specify filter: coef_val = 1/2^inp_bits
)
{
	filt_s.coef_div = (1 << inp_bits); // coef_val = 1/coef_div
	filt_s.half_div = (filt_s.coef_div >> 1); // Half coef_div (used for rounding)
	filt_s.coef_bits = inp_bits; // Store to use for fast divide
} // gen_filter_params( specify_filter
/*****************************************************************************/
static void init_adc_trigger( // Initialise the data for this ADC trigger
	ADC_DATA_TYP &adc_data_s, // Reference to structure containing data for this ADC trigger
	int inp_mux  // Mapping from 'trigger channel' to 'analogue ADC mux input'
)
{
	int phase_cnt; // ADC Phase counter


	for (phase_cnt=0; phase_cnt<USED_ADC_PHASES; ++phase_cnt)
	{
		init_adc_phase( adc_data_s.phase_data[phase_cnt] );
	} // for phase_cnt

	gen_filter_params( adc_data_s.filt ,0 ); // Initialise filter to fast response

	adc_data_s.guard_off = 0; // Initialise guard to ON
	adc_data_s.mux_id = inp_mux; // Assign Mux port for this trigger
	adc_data_s.filt_cnt = 0; // Initialise filter count

} // init_adc_trigger
/*****************************************************************************/
static void configure_adc_ports_7265( // Configure all ADC data ports
	in buffered port:32 p32_data[NUM_ADC_DATA_PORTS], // Array of 32-bit buffered ADC data ports
	clock xclk, // XMOS internal clock
	out port p1_serial_clk,	// 1-bit Port connecting to external ADC serial clock
	port p1_ready,	 // 1-bit port used to as ready signal for p32_adc_data ports and ADC chip
	out port p4_mux	// 4-bit port used to control multiplexor on ADC chip
)
{
	int port_cnt; // port counter


	/* xclk & p1_ready, are used as the clock & ready signals respectively for controlling the following 2 functions:-
		(1) Reading the Digital data from the AD7265 into an XMOS buffered 1-bit port
		(2) Initiating an Analogue-to-digital conversion on the AD7265 chip.

		For (1), Referring to XMOS XS1 library documentation ...
		By default, the ports are read on the rising edge of the clock, and when the ready signal is high.

		For (2), Referring to the  AD7265 data-sheet ...
		p1_ready is used to control CSi (Chip Select Inverted)
		When signal CSi falls, ( 1 -> 0 ) A2D conversion starts. When CSi rises ( 0 -> 1 ), conversion halts.
    The digital outputs are tri-state when CSi is high (1).
		xclk is used to control SCLK (Serial Clock).
		Succesive bits of the digital sample are output after a falling edge of SCLK. In the following order ...
		[0, 0, Bit_11, Bit_10, ... Bit_1, Bit_0, 0, 0]. If CSi rises early, the LSB bits (and zeros) are NOT output.

		We require the analogue signal to be sampled on the falling edge of the clock,
		According to the AD7265 data-sheet, the output data is ready to be sampled 36 ns after the falling edge.
		If we use the rising edge of the xclk to read the data, an xclk frequency of 13.8 MHz or less is required.
		Frequencies above 13.9 MHz require the data to be read on the next falling edge of xclk.

		We require the analogue signal to be sampled when CSi goes low,
		and we require data to be read when the ready signal goes high.
		By using the set_port_inv() function to invert the ready signal, it can be used for both (1) & (2).
		NB If an inverted port is used as ready signals to control another port,
    the internal signal (used by XMOS port) is inverted with respect to the external signal (used to control AD7265).
	*/

	// configure the clock to be (at most) 13 MHz. (NB This is independent of the XCore Reference Frequency)
	configure_clock_rate_at_most( xclk ,ADC_SCLK_MHZ ,1 );
	configure_port_clock_output( p1_serial_clk, xclk ); // Drive ADC serial clock port with XMOS clock

	configure_out_port( p1_ready ,xclk ,0 ); // Set initial value of port to 0 ( NOT ready )
	set_port_inv( p1_ready ); // Invert p1_ready for connection to AD7265, which has active low

	// For each port, configure to read into buffer when using the serial clock
	for (port_cnt=0; port_cnt<NUM_ADC_DATA_PORTS; port_cnt++)
	{
		configure_in_port_strobed_slave( p32_data[port_cnt] ,p1_ready ,xclk );
	} // for port_cnt

	// Start the ADC serial clock port
	start_clock( xclk );
} // configure_adc_ports_7265
/*****************************************************************************/
static void get_adc_port_data( // Get ADC data from one port
	ADC_PHASE_TYP &phase_data_s, // Reference to structure containing data for this phase of one ADC trigger
	in buffered port:32 inp_data_port // ADC input data port for one phase
)
{
	unsigned inp_val; // input value read from buffered ports
	unsigned tmp_val; // Temporary manipulation value
	short word_16; // signed 16-bit value
	ADC_TYP int_32; // signed (32-bit) ADC value


	endin( inp_data_port ); // End the previous input on this buffered port

	inp_data_port :> inp_val; // Get new input


	// This section extracts active bits from sample with padding zeros
	tmp_val = bitrev( inp_val );	// Reverse bit order. WARNING. Machine dependent
	tmp_val <<= ADC_SHIFT_BITS;		// Align active bits to MS 16-bit boundary
	word_16 = (short)(tmp_val & ADC_MASK);	// Mask out active bits and convert to signed word
	int_32 = ((int)word_16) >> ADC_DIFF_BITS; // Convert to int and recover original magnitude

	// Check if filtering selected
	if (1 == ADC_FILTER)
	{
		ADC_TYP sum_val = phase_data_s.adc_val; // get old value

		// Create filtered value and store in int_32 ...
		int_32 = (sum_val + (sum_val << 1) + int_32 + 2) >> 2; // 1st order filter (uncalibrated value)
	} // if (1 == ADC_FILTER)

	phase_data_s.adc_val = int_32; // Store uncalibrated value

} // get_adc_port_data
/*****************************************************************************/
static void get_trigger_data_7265(
	ADC_DATA_TYP &adc_data_s, // Reference to structure containing data for this ADC trigger
	in buffered port:32 p32_data[NUM_ADC_DATA_PORTS],  // Array of 32-bit buffered ADC data ports
	port p1_ready,	 // 1-bit port used to as ready signal for p32_adc_data ports and ADC chip
	out port p4_mux	// 4-bit port used to control multiplexor on ADC chip
)
{
	int port_cnt; // port counter
	unsigned time_stamp;


	p4_mux <: adc_data_s.mux_id; // Signal to Multiplexor which input to use for this trigger

	// Loop through phases
	for (port_cnt=0; port_cnt<NUM_ADC_DATA_PORTS; port_cnt++)
	{
		clearbuf( p32_data[port_cnt] );
	} // for port_cnt

	p1_ready <: 1 @ time_stamp; // Switch ON input reads (and ADC conversion)
	time_stamp += ADC_TOTAL_BITS; // Allows sample-bits to be read on buffered input ports
	p1_ready @ time_stamp <: 0; // Switch OFF input reads, (and ADC conversion)

	sync( p1_ready ); // Wait until port has completed any pending outputs

	// Get ADC data for each used phase
	for (port_cnt=0; port_cnt<NUM_ADC_DATA_PORTS; port_cnt++)
	{
		get_adc_port_data( adc_data_s.phase_data[port_cnt] ,p32_data[port_cnt] );
	} // for port_cnt

} // get_trigger_data_7265
/*****************************************************************************/
static void filter_adc_data( // Low-pass filter generate a mean value which is used to 'calibrate' the ADC data
	ADC_PHASE_TYP &phase_data_s, // Reference to structure containing adc phase data
	ADC_FILT_TYP &filt_s // Reference to structure containing filter parameters
)
/* In its simplest from, this is a 1st order IIR filter, it is configured as a low-pass filter,
 * However, the impulse response of the filter can have a short decay or a long decay,
 * depending on the value of 'coef_bits'. Therefore the filter response can be changed dynamically.
 * Due to the changing coefficient, the output is NOT like a 1st order IIR filter, and may have overshoot.
 * The input ADC value is up-scaled, to allow integer arithmetic to be used.
 * The output mean value is down-scaled by the same amount.
 * Error diffusion is used to keep control of systematic quantisation errors.
 */
{
	int scaled_inp = ((int)phase_data_s.adc_val << ADC_SCALE_BITS); // Upscaled ADC input value
	int diff_val; // Difference between input and filtered output
	int increment; // new increment to filtered output value


	// Form difference with previous filter output
	diff_val = scaled_inp - phase_data_s.filt_val;

	// Multiply difference by filter coefficient (alpha)
	diff_val += phase_data_s.coef_err; // Add in diffusion error;
	increment = (diff_val + filt_s.half_div) >> filt_s.coef_bits ; // Multiply by filter coef (with rounding)
	phase_data_s.coef_err = diff_val - (increment << filt_s.coef_bits); // Evaluate new quantisation error value

	phase_data_s.filt_val += increment; // Update (up-scaled) filtered output value

	// Update mean value by down-scaling filtered output value
	phase_data_s.filt_val += phase_data_s.scale_err; // Add in diffusion error;
	phase_data_s.mean = (phase_data_s.filt_val + ADC_HALF_SCALE) >> ADC_SCALE_BITS; // Down-scale
	phase_data_s.scale_err = phase_data_s.filt_val - (phase_data_s.mean << ADC_SCALE_BITS); // Evaluate new remainder value

} // filter_adc_data
/*****************************************************************************/
static void update_adc_trigger_data( // Update ADC values for this trigger
	ADC_DATA_TYP &adc_data_s, // Reference to structure containing data for this ADC trigger
	in buffered port:32 p32_data[NUM_ADC_DATA_PORTS], // Array of 32-bit buffered ADC data ports
	port p1_ready,	 // 1-bit port used to as ready signal for p32_adc_data ports and ADC chip
	int trig_id, // trigger identifier
	out port p4_mux	// 4-bit port used to control multiplexor on ADC chip
)
{
	int phase_cnt; // ADC Phase counter


	get_trigger_data_7265( adc_data_s ,p32_data ,p1_ready ,p4_mux );	// Get ADC values for this trigger

	// Loop through used phases
	for (phase_cnt=0; phase_cnt<USED_ADC_PHASES; ++phase_cnt)
	{
		filter_adc_data( adc_data_s.phase_data[phase_cnt] ,adc_data_s.filt ); // Sum ADC values
	} // for phase_cnt

	/* A low-pass filter is used to 'calibrate' the ADC values, see filter_adc_data for more detail.
	 * The filter is initially dynamic. When there are only a few samples the filter has a fast response,
	 * as the number of samples increases the response gets slower,
	 * until the filter is producing a mean over about ADC_MAX_COEF_DIV samples (e.g. 8192)
	 */
	// Check if filter in 'dynamic' mode
	if (ADC_MAX_COEF_DIV > 	adc_data_s.filt_cnt)
	{
		adc_data_s.filt_cnt++; // Update sample count

		// Check if time to slow down filter response
		if (adc_data_s.filt.coef_div == adc_data_s.filt_cnt)
		{
			gen_filter_params( adc_data_s.filt ,(adc_data_s.filt.coef_bits + 1) ); // Double decay time of filter
		} // if (adc_data_s.filt.coef_div == adc_data_s.filt_cnt)
	} // if (ADC_MAX_COEF_DIV > 	adc_data_s.filt_cnt)

	adc_data_s.guard_off = 0; // Reset guard to ON
} // update_adc_trigger_data
/*****************************************************************************/
static void enable_adc_capture( // Do set-up to allow ADC values for this trigger to be captured
	ADC_DATA_TYP &adc_data_s // Reference to structure containing data for this ADC trigger
)
{
	adc_data_s.my_timer :> adc_data_s.time_stamp; 	// get current time
	adc_data_s.time_stamp += ADC_TRIGGER_DELAY;				// Increment to time of ADC value capture
	adc_data_s.guard_off = 1;													// Switch guard OFF to allow capture
} // enable_adc_capture
/*****************************************************************************/
static void service_control_token( // Services client control token for this trigger
	ADC_DATA_TYP &adc_data_s, // Reference to structure containing data for this ADC trigger
	int trig_id, // trigger identifier
	unsigned char inp_token // input control token
)
{
	switch(inp_token)
	{
		case XS1_CT_END : // ADC values ready for capture
			enable_adc_capture( adc_data_s ); // Enable capture of ADC values
		break; // case XS1_CT_END

    default: // Unsupported Control Token
			assert(0 == 1); // Error: Unknown Control Token
		break;
	} // switch(inp_token)

} // service_control_token
/*****************************************************************************/
static void service_data_request( // Services client command data request for this trigger
	ADC_DATA_TYP &adc_data_s, // Reference to structure containing data for this trigger
	streaming chanend c_control, // ADC Channel connecting to Control, for this trigger
	int trig_id, // trigger identifier
	int inp_cmd // input command
)
{
	int phase_cnt; // ADC Phase counter
	ADC_TYP adc_val; // ADC value
	ADC_TYP adc_sum; // Accumulator for transmitted ADC Phases


	switch(inp_cmd)
	{
		case ADC_CMD_DATA_REQ : // Request for ADC data
			adc_sum = 0; // Clear Accumulator for transmitted ADC Phases

			// Loop through used ADC phases
			for (phase_cnt=0; phase_cnt<USED_ADC_PHASES; ++phase_cnt)
			{
				// Convert parameter phase values to zero mean
				adc_val = adc_data_s.phase_data[phase_cnt].adc_val - adc_data_s.phase_data[phase_cnt].mean;
				adc_sum += adc_val; // Add adc value to sum
				adc_data_s.params.vals[phase_cnt] = adc_val; // Load adc value into parameter structure
			} // for phase_cnt

			// Calculate last ADC phase from previous phases (NB Sum of phases is zero)
			adc_data_s.params.vals[(NUM_ADC_PHASES - 1)] = -adc_sum;

			c_control <: adc_data_s.params; // Return structure of ADC parameters
		break; // case ADC_CMD_DATA_REQ

    default: // Unsupported Command
			assert(0 == 1); // Error: Received unsupported ADC command
	  		  break;
	} // switch(inp_cmd)
} // service_data_request
/*****************************************************************************/
void foc_adc_7265_triggered( // Thread for ADC server
	streaming chanend c_control[NUM_ADC_TRIGGERS], // Array of ADC control Channels connecting to Control (inner_loop.xc)
	chanend c_trigger[NUM_ADC_TRIGGERS], // Array of channels receiving control token triggers from PWM threads
	in buffered port:32 p32_data[NUM_ADC_DATA_PORTS], // Array of 32-bit buffered ADC data ports
	clock xclk, // Internal XMOS clock
	out port p1_serial_clk, // 1-bit port connecting to external ADC serial clock
	port p1_ready,	 // 1-bit port used to as ready signal for p32_adc_data ports and ADC chip
	out port p4_mux	// 4-bit port used to control multiplexor on ADC chip
)
{
	// Mapping array from 'trigger channel' to 'analogue ADC mux input' See. AD7265 data-sheet
	int trigger_channel_to_adc_mux[NUM_ADC_TRIGGERS] = { 0, 2 };

	ADC_DATA_TYP all_adc_data[NUM_ADC_TRIGGERS];
	unsigned char cntrl_token; // control token
	int cmd_id; // command identifier
	int trig_id; // trigger identifier
	int do_loop = 1;   // Flag set until loop-end condition found


	acquire_lock();
	printstrln("                                             ADC Server Starts");
	release_lock();

	// Initialise data structure for each trigger
	for (trig_id=0; trig_id<NUM_ADC_TRIGGERS; ++trig_id)
	{
		init_adc_trigger( all_adc_data[trig_id] ,trigger_channel_to_adc_mux[trig_id] );
	} // for trig_id

	set_thread_fast_mode_on();

	configure_adc_ports_7265( p32_data ,xclk ,p1_serial_clk ,p1_ready ,p4_mux );

	// Loop until loop termination condition detected
	while (do_loop)
	{
#pragma xta endpoint "adc_7265_main_loop"
#pragma ordered // If multiple cases fire at same time, service top-most first
		select
		{
			// Service any Control Tokens that are received
			case (int trig_id=0; trig_id<NUM_ADC_TRIGGERS; ++trig_id) inct_byref( c_trigger[trig_id], cntrl_token ):
				service_control_token( all_adc_data[trig_id] ,trig_id ,cntrl_token );
			break;

			// If guard is OFF, load 'my_timer' at time 'time_stamp'
			case (int trig_id=0; trig_id<NUM_ADC_TRIGGERS; ++trig_id) all_adc_data[trig_id].guard_off => all_adc_data[trig_id].my_timer when timerafter( all_adc_data[trig_id].time_stamp ) :> void:
				update_adc_trigger_data( all_adc_data[trig_id] ,p32_data ,p1_ready ,trig_id ,p4_mux );
			break;

			// Service any client request for ADC data
			case (int trig_id=0; trig_id<NUM_ADC_TRIGGERS; ++trig_id) c_control[trig_id] :> cmd_id:
				if (ADC_CMD_LOOP_STOP != cmd_id)
				{
					service_data_request( all_adc_data[trig_id] ,c_control[trig_id] ,trig_id ,cmd_id );
				} // if (ADC_CMD_LOOP_STOP != cmd_id)
				else
				{
					do_loop = 0; // Terminate loop
				} // else !(ADC_CMD_LOOP_STOP != cmd_id)
			break;
		} // select
	} // while (do_loop)

	acquire_lock();
	printstrln("");
	printstrln("                                             ADC Server Ends");
	release_lock();

} // foc_adc_7265_triggered
/*****************************************************************************/
// adc_7265.xc
