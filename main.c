/*	Full XSimCTRL interface with PID Feedback Control ***

Version 0.1.022
		
Synopsis
========
Every 1 ms (1000 Hz) for both motor axis:
		-ADC samples potentiometer on PC1 and PC2 pins.
		-PID feedback loop updates PWM duty cycles
Every 20 ms (50 Hz):
		-assembles one XsimCTRL Dynamic Static Block and sends it back to PC over serial port
Serial stream is decoded as fast as possible (unlimited thread loop).
xSimCTRL Dynamic Static Block contents:
		"[HH 00HH 00HH HH   HH HHHH 00HH HH]" (no spaces)
      sp1 gp1 pwm1 kp1  sp1 gp2 pwm2 kp2
		-sent as ascii string of length 26 (24 chars surrounded by [])
		where:
			sp1		set_position_1		signed short hex, 2 chars, -128...127
			gp1		get_position_1		signed short hex, 4 chars (left padded with 00), -128...127
			pwm1	PWM axis 1				signed hex, 4 chars, -32768....32767
			kp1		K_P_1							unsigned hex, 2 chars, 0...255
		repeats for axis #2

xSimCTRL Position Command Block:
		(received by MCU from PC)
		5-bytes in the format: 0x41, 0x42, 0xFF, a0, a1.
		a0 - one byte - position for motor 1, signed short (-128....127)
		a1 - one byte - position for motor 2  signed short (-128....127)
		0x41, 0x42, 0xFF - delimiter tokens separating position data
		
LED indicators:
		GREEN		- flashes 1/sec as program heartbeat
		BLUE		- PWM intensity for axis1
		ORANGE	- PWM intensity for axis2
		RED			- Dymanic State block detected from PC (toggles)
*/

#include <stdio.h>
#include <stdlib.h>
#include "math.h"

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#include "usbcfg.h"
#include "misc.h"



#define VERSION_MAJOR   0
#define VERSION_MINOR   1
#define VERSION_SUB     22
#define VERSION_ABOUT	  "internal development branch"

/* Macro explanations:
 * 
 *	NUM_AXIIS				number of motor axis
 * 	ACS							ACS715 Pololu current sensor constant milliamps per volt
 * 	VDD5						Actual 5V reading for the Vdd 5V plane (in volts)
 *	PID_FREQ				PID loop frequency (Hz)
 * 	K_P, K_I, K_D		Positional error feedback constants: Proportional, Integral and Derivative.
 *										note: 32bit integer math caps max permissible K_D~524 (for PID_FREQ=1000)
 *													K_P is nominally calculated as: 10000 / (desired 12bit Error value corresponding to 100% PWM),
														EX:  10000 / (2048) = 5. K_P=5 yields 100% PWM signal when 12-bit pot fully turned
														     (if "center" considered as 2048).
 *	PWMGAIN					scales final PWM output to this percentage (use 0-100). Used to match motor to a more powerful PSU.													
 *	VALET_MODE:			0, 1, 2 or 3 (0 means off, 3 means the motor end-points and speeds are the most reduced (i.e. 100%,75%,50%,25% effective)
 *	DEADBAND:				minimum PWM% needed to cause any movement (in tenths of %) 
 *	ADC_GRP1_NUM_CHANNELS	Total number of channels to be sampled by a single ADC operation.
 *	ADC_GRP1_BUG_DEPTH		Depth of the conversion buffer, channels are sampled four times each.
 *	DISPLAY_ON						Turn data display thread on or off (i.e. set TRUE for XsimCTRL, FALSE for X-Sim */

#define NUM_AXIIS	2
#define ACS   15160
#define VDD5  5.05 
#define K_P	 200
#define K_I	 0
#define K_D	 5
#define PWMGAIN  100
#define VALET_MODE  0
#define DEADBAND	0L
#define POT_NOISE_FLOOR		6
#define ADC_GRP1_NUM_CHANNELS   (NUM_AXIIS * 2)
#define ADC_GRP1_BUF_DEPTH      16
#define DISPLAY_ON		TRUE
#define PID_FREQ		1000L

/* 	Feedback struct defines */
#define MAX_FB_AGRESSION		4
#define MAX_FB_MODE		3
	/* modes */
#define FB_P		0
#define FB_PI		1
#define FB_PID	    2
#define FB_PID_IND  3
	/* strengths */
#define FB_SLOW	0
#define FB_MODERATE	1
#define FB_NORMAL	2
#define FB_FAST	3
#define FB_BONECRUSHER	4


typedef struct  {
	long signed ADC_centre [NUM_AXIIS];	/* calibration readings for axiis' centres */
	long signed ADC_stroke [NUM_AXIIS];	/* calibration reading for axiis' @ max machine travel (from centre) */
} CALIBRATION;

typedef struct fbs {
    const uint16_t mode;
    const char *desc;
    bool_t derivative_alt;
    uint16_t agression;
    float scaler[MAX_FB_AGRESSION + 1];
} FBS;

FBS FB_mode [MAX_FB_MODE + 1] = {
	/* P-only attributes */
	{
		FB_P,
		"P-only",
		NULL,
		FB_NORMAL,
		{ 0.5, 1.0, 2.5, 5.0, 7.5 }
	},
	/* PI-only attributes */
	{
		FB_PI,
		"PI-only",
		NULL,
		FB_MODERATE,
		{ 0.5, 0.16, 0.04, 0.01, 0.0025 }
	},
	/* PID Dependant-form attributes */
	{
		FB_PID,
		"PID dependant",
		FALSE,
		FB_MODERATE,
		{ 0.5, 0.16, 0.04, 0.01, 0.0025 }
	},
	/* PID Independant-form attributes */
	{
		FB_PID_IND,
		"PID independant",
		FALSE,
		FB_MODERATE,
		{ 0.5, 0.16, 0.04, 0.01, 0.0025 }
	}	
};

char *FB_agression_desc [MAX_FB_AGRESSION + 1] = { "Slow", "Moderate", "Normal", "Fast", "BoneCrusher" };

/* set feedback default P-only, moderate */
FBS *FB_current = &FB_mode[FB_PID_IND];

	
/* External Variables */
bool_t listening_active = FALSE;	/* serial stream listening status */

static SerialUSBDriver SDU1;		/* Virtual serial port over USB.*/

static char axis [NUM_AXIIS];												/* axis a0 and axis a1 values pulled from serial stream */
static long signed axis_command [NUM_AXIIS];		/* ...and translated/scaled relative to motor static calibration settings */															
							/* (I think the protocol author is wrong and this is actualy unsigned data. */
							/*  This must be so otherwise my code below would not work as written. */
static long signed pwm_display [NUM_AXIIS];	/* hack to get PWM values into display function */
uint16_t pwm_gain = 0;														/* actual PWM gain used */
uint16_t pwm_gain_set =   (uint16_t)PWMGAIN;		  /* load default */
uint16_t valet_mode =     (uint16_t)VALET_MODE; 	/* load default */	
long signed valet_scale = 0;									/* valet_mode with pre-calc  */
long signed deadband = (long signed)(DEADBAND * 10L);	/* scaled up to match PWM scale (hundreths of %) */
long signed pot_noise_floor = (long signed)POT_NOISE_FLOOR;	/* +/- this raw ADC value  */
long signed pot_noise_floor_10000;	/* expressed as % scaled up to match PWM scale (hundredths of percent */
			/* Kp is multiplied by PID_FREQ/PID_FREQ here only to save cycles within high frequency loops.   */	
			/* Kd is multiplied by (PID_FREQ^2)/PID_FREQ here, likewise. 																		 */	
long signed Kp = (long signed)(K_P);							/* proportional control K constant   */
long signed Ki = (long signed) K_I;								/* integral control K constant		   */
long signed Kd = (long signed)(K_D);							/* derivitate control K constant	   */
			/* K must be scaled relative to pot range % actually used (ie. half range==> double K)					 */
long signed Kp_scale = 0L;															/* Kp scaled 											   */
long signed Ki_scale = 0L;															/* Ki scaled						   					 */
long signed Kd_scale = 0L;															/* Kd scaled	   										 */
long signed error_prev [NUM_AXIIS];
long signed error_i [NUM_AXIIS];								/* integral error over time, for Ki  */
long signed avg_ch_prev [NUM_AXIIS];
long signed previous_derivative [NUM_AXIIS];
long signed min_d [NUM_AXIIS], max_d [NUM_AXIIS];
static uint16_t current [NUM_AXIIS];	/* motor current in tenths of Amps (+/-) */
static adcsample_t samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH]; /* ADC samples buffer */
static adcsample_t avg_ch [ADC_GRP1_NUM_CHANNELS];		/* ADC smoothed values */
CALIBRATION calibration;


/* Function prototypes */
void calibrate_load_defaults( CALIBRATION *cp );
static void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n);
void pwm_set( BaseSequentialStream *chp, bool_t enabling );
void pid_set( BaseSequentialStream *chp, bool_t enabling );
void command_arm( BaseSequentialStream *chp, bool_t arming );
void command_stop( BaseSequentialStream *chp );
void command_center( BaseSequentialStream *chp );
void PID_set_tune( BaseSequentialStream *chp );

/* Internal Model Control (IMC) PID variables */
double Tc = (double)0.0;	/* closed loop time constant scaled by agression mode */
uint16_t agression = 2;	/* IMC agressiveness (tweaks Tc) */
double Ti = (double)0.0;	/* Reset Time constant */
double Td = (double)0.0;	/* deritivate time constant*/
double Integral_scaler = (double)1.0;	/* DEBUG scaler for PI testing */
double Kc = (double)0.0;	/* "Controller Gain" */
double alpha_filter = (double)0.0;	/* Filter constant for PWM output smoothing */
double Tp = (double)2.0; /*  process time constant (time to 63% SS) */
double theta_p = (double)0.06000;	/* deadtime */
double Kprocess = (double)338.0000;	/* "Process Gain" */
//static double CO_bias = (double)0.0;

	/* based on bump test of my 14.4V drill motor */
		// Tp = 2.0;
		// theta_p = 0.06;
		// Kprocess = 338.0;	



/* Load calibration defaults for  physical machine for every axis
 * Range and centre.
 */
void calibrate_load_defaults(  CALIBRATION *cp ) {
	/* (calibration test data) */
	cp->ADC_stroke [0] = 2032L;
	cp->ADC_centre [0] = 2032L;
	cp->ADC_stroke [1] = 512L;
	cp->ADC_centre [1] = 1520L;
}

	/* Calc IMC-based PID tuning parameters from bump test data
	 * and other necessary PID loop pre-calcs done here too.
	 *
	 */
void PID_set_tune( BaseSequentialStream *chp ) {

	double agression_scaler;
	char *error1 = "\r\n\r\n  >>> ERROR: Invalid Feedback parameter detected ";
	char *error2 = "             Defaulting to P-only control at moderate setting.\r\n";

	valet_scale = (long signed)( 100L*(4 - valet_mode) );

	/* check for out of range values first*/
	if ( (FB_current->mode < 0) || (FB_current->mode > MAX_FB_MODE) ) {
		chprintf(chp, "%s [FB mode: %d].\r\n",error1,FB_current->mode);
		chprintf(chp, "%s",error2);
		FB_current = &FB_mode [FB_P];
		FB_current->agression = FB_MODERATE;
	}
	else if ( (FB_current->agression < 0) || (FB_current->agression > MAX_FB_AGRESSION) ) {
		chprintf(chp, "%s [FB agression: %d].\r\n",error1,FB_current->agression);
		chprintf(chp, "%s",error2);
		FB_current = &FB_mode [FB_P];
		FB_current->agression = FB_MODERATE;
	}
	
	agression_scaler = (double)FB_current->scaler [FB_current->agression];
	
	if ( (FB_current->mode == FB_PI) || (FB_current->mode == FB_PID) ) {	/* these calcs needed for PI, PID modes only */
		/* Tc scaled based on agression mode    1/( 5^(agression-1) */
		double d1 = Tp;
		double d2 = ((double)8.0) * theta_p;
		Tc = (d1>d2) ? d1 * agression_scaler : d2 * agression_scaler;
	}

		switch (FB_current->mode) {
		case FB_P:	/* P-only ("ITAE" type) */	
			Kc = agression_scaler * ((double)0.2)/Kprocess * pow(Tp/theta_p,(double)1.22) ;
			break;
		case FB_PI:	/* PI-only */
			Ti = Tp;
			Kc = Tp / (Kprocess * (Tc + theta_p) );
			break;
		case FB_PID:	/* PID, "Dependant, Ideal form" */
			Ti = Tp + ((double)0.5) * theta_p;
			Td = (Tp * theta_p) / (((double)2.0) * Tp + theta_p);
			Kc = (Tp + ((double)0.5) * theta_p ) / (Kprocess * (Tc + ((double)0.5) * theta_p) );
			break;
		case FB_PID_IND:	/* PID, Independant form */			
			/* effective K constants must be scaled inversely relative to actual % of potentiometer range used */
			Kp_scale =  (long signed)( (float)Kp * 2047.0 / ((float)calibration.ADC_stroke [0]) );
			Ki_scale =  (long signed)( (float)Ki * 2047.0 / ((float)calibration.ADC_stroke [0]) );
			Kd_scale =  (long signed)( (float)Kd * 2047.0 / ((float)calibration.ADC_stroke [0]) );
			break;
		default:
			/* error. default to P-only, moderate tune */
			chprintf(chp, "%s [FB mode: %d is undefined].\r\n",error1,FB_current->mode);
			chprintf(chp, "%s",error2);
			FB_current = &FB_mode [FB_P];
			FB_current->agression = FB_MODERATE;
			agression_scaler = (double)FB_current->scaler [FB_current->agression];
			Kc = agression_scaler * ((double)0.2)/Kprocess * pow(Tp/theta_p,(double)1.22) ;
			break;
		}
		
		/* 100x pre-calc on Kc to reflect that 100% is represented 
		   as 10,000 in PWM_PERCENTAGE_TO_WIDTH function */
		Kc = ((double)1000.0) * Kc;
		
		pot_noise_floor_10000 = pot_noise_floor * 10000L * 2 / 2048L;	/* double the noise amplitude to be safe */
		
}

/* ADC conversion group configuration
 * Mode:        Linear buffer, 4 samples of 4 channels, SW triggered.
 * Channels:    IN14,IN15,IN6,IN7   (56 cycles sample time)
 */
static const ADCConversionGroup adcgrpcfg = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  adccb,
  NULL,
  /* HW dependent part.*/
  0,
  ADC_CR2_SWSTART,
  ADC_SMPR2_SMP_AN7(ADC_SAMPLE_56) | ADC_SMPR2_SMP_AN6(ADC_SAMPLE_56) | ADC_SMPR1_SMP_AN15(ADC_SAMPLE_56) | ADC_SMPR1_SMP_AN14(ADC_SAMPLE_56),
  0,
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,
  ADC_SQR3_SQ4_N(ADC_CHANNEL_IN7) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN6) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN15) | ADC_SQR3_SQ1_N(ADC_CHANNEL_IN14)
};

/* PWM 21.5 KHz configuration
 * Mode:			Active high
 * Channels:	2
 */
static PWMConfig pwmcfg = {
	11000000,	/* 11Mhz PWM clock frequency  (i.e. not the actual PWM freq)*/
	512,			/* PWM "period" in ChibiOS (i.e. ticks for one PWM cycle). */
						/* Yields a 21.5 KHz PWM signal (0.047 ms signal period */
  NULL,
  {
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
  },
  /* HW dependent part.*/
  0
};

/*	GP2 callback (green flasher)
 */
static void gpt2cb(GPTDriver *gptp) {

  (void)gptp;
	
//DEBUG  palTogglePad(GPIOD, GPIOD_LED4);	/* flash green */
}

/* GPT2 configuration
 * 10,000 Hz with callback.
 */
static const GPTConfig gpt2cfg = {
  10000,  /* 10 KHz timer clock.*/
  gpt2cb    /* Timer callback.*/
};

/* GPT3 callback (ADC conversion trigger & PID feedback loop)
 *
 */
static void gpt3cb(GPTDriver *gptp) {
  (void)gptp;


	static long signed derivative [NUM_AXIIS];
	static long signed new_derivative [NUM_AXIIS];

	static bool_t forward [NUM_AXIIS];									/* direction fwd=TRUE */
	static long signed error [NUM_AXIIS];	/* signal error (=setpoint - measured) */
	static long signed PWM [NUM_AXIIS];			/* PWM% to the hundredth, expressed as 0-10000 */
																	  		/* example: PWM [0] = 625 means 6.25% */
	static long signed PWM_abs [NUM_AXIIS];
	static uint16_t pwm_width [NUM_AXIIS];	/* PWM width (ticks) for axis*/	

	
	palTogglePad(GPIOD, GPIOD_LED4);		/* green toggle */		
	
	/* some precalculation done here to save time (explained further below) */
	static const long signed preCalc1 = (long signed) (  (float)VDD5 * (float)ACS );
	static const long signed preCalc2 = (long signed) (( (float)VDD5 * (float)ACS ) / 200.0);

	
	/* Starts an asynchronous ADC conversion operation, the conversion
		 will be executed in parallel to the current PWM cycle and will
		 terminate before the next PWM cycle.*/
  chSysLockFromIsr();
  adcStartConversionI(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
  chSysUnlockFromIsr();
	
	/* calculate motor currents (based on Pololu ACS715, Vcurrent_sense = V5/2 + 0.066*A) */
	/* (may need this in future for velocity/current limiting */
	current [0] = (uint16_t) (( ((long signed)avg_ch [2]) * preCalc1 / 4095L / 100L ) - preCalc2);
	current [1] = (uint16_t) (( ((long signed)avg_ch [3]) * preCalc1 / 4095L / 100L ) - preCalc2);
	
	/* main PID block starts here
	 * PWM duty cycles continuously calculated based on latest potentiometer &
 	 * position commands available.	*/
	
	
		/* Simple Proportional Feedback = K * ERROR where ERROR = COMMAND - CURRENT POSITION */
		/* Calculated as: COMMAND = (((axis << 4) - 2048L) * ADC_stroke * valet_scale) / 2048L / 400L + ADC_centre */
		/* So, ERROR = COMMAND - raw ADC value for axis potentiometer reading */
		
		/* (8-bit motion commands converted to 12-bit first, XsimCTRL has 127 as center. */
		/*	*** MOVED TO DATA STREAM LOOP ***  */
		
		
		/* axis 1 */
		error [0] = axis_command [0] - (long signed) avg_ch [0];										/* straight positional error */
		error_i [0] = error_i [0] + (10000L*error [0])/(long signed)PID_FREQ;				/* this is 10,000*(integral error) */
		if ( FB_current->derivative_alt )
			derivative [0] = ( avg_ch_prev [0] - avg_ch [0] ) * (long signed)PID_FREQ;		/* derivative error ALT form (-dPV/dt) */	
		else derivative [0] = (error [0] - error_prev [0]) * (long signed)PID_FREQ;		/* derivative error, regular (de/dt) */
	
		error_prev [0] = error [0];		
		avg_ch_prev [0] = avg_ch [0];	
		
		/* Deritivate noise filter applied: (median filter) reduce noise/impulse amplification */			
		if (derivative [0] > max_d [0])
			new_derivative [0] = max_d [0]; // median found
		else if (derivative [0] < min_d [0])
			new_derivative [0] = min_d [0]; // median found
		// median found
		else
			new_derivative [0] = derivative [0];
		if (derivative [0] > previous_derivative [0]) { // for next cycle
			max_d [0] = derivative [0];
			min_d [0] = previous_derivative [0];
		} 
		else {
			max_d [0] = previous_derivative [0];
			min_d [0] = derivative [0];
		}
		previous_derivative [0] = derivative [0];		
	
		/* axis 2 */
		/* ..... repeat axis 1 above ...... */

			
	/*	PID notes:
	 *	another way to ti t is output = Kc * [ e + 1/I * SUM_e * dt + D * DELTA_e / dt ], where
	 *	Kc is "controller gain", and I and D are integral and derivative constants choosen.
	 *  This is known as the "Ideal Algorithm" here http://www.expertune.com/artCE87.aspx.
	 *	It is better than the form I'm using (which they call "Parallel" form) in that it is more stable.
	 *	Having independant Kp, Ki and Kd values tends to cause multiple ways for the system to become unstable.
	 */	
	
	/* full PID PWM as a 0-10,000 i.e. 100ths of a percent
	 * (The formulas below, as written, are misleading because of necessary scaling to prevent integer math overflow.
	 * In reality, what's being done is still the "standard" discretised  PID of Kp*e + Ki*integral_e*dt + Kd*delta_e/dt).
	 */

	/* relace with Kp_scales... eventually again	
	PWM [0] = (Kp * error [0]) + (Ki * error_i [0])/PID_FREQ + (Kd * new_derivative [0]);
	PWM [1] = (Kp * error [1]) + (Ki * error_i [0])/PID_FREQ + (Kd * error_d [0]);
	 */

	
	switch ( FB_current->mode ) {
	case FB_PID_IND:
		PWM [0] = (Kp * error [0]) + (Ki * error_i [0] / 10000L) + (Kd * new_derivative [0] );
		break;
	case FB_PID:
		PWM [0] = (long signed)( (Kc * (double)error [0]) + (Integral_scaler * Kc/Ti * (double)error_i [0] / (double)10000.0) + (Kc * Td * (double)new_derivative [0] ) );
		break;
	case FB_PI:
		PWM [0] = (long signed)( (Kc * (double)error [0]) + (Integral_scaler * Kc/Ti * (double)error_i [0] / (double)10000.0) );
		break;
	case FB_P:
		PWM [0] = (long signed)( Kc * (double)error [0] );
		break;
	}
				
	/* make sure Kc was pre-multiplied by 1000, so the above calcs result in a -100,000 to 100,000 range (1000x PWM%) */	
	if ( PWM [0] > 100000L )
		PWM [0] = 100000L;
	else if ( PWM [0] < -100000L )
		PWM [0] = -100000L;
	
	if ( PWM [1] > 100000L )
		PWM [1] = 100000L;
	else if ( PWM [1] < -100000L )
		PWM [1] = -100000L;
		
	if ( PWM [0] < 0L ) {
		forward [0] = FALSE;
		PWM_abs [0] = -PWM [0]/10L;
		/* reduce deadband effect (a minimal PWM output is required to cause any motion (i.e. overcome friction) */
		if ( (PWM_abs [0] < deadband) && (PWM_abs [0] > deadband>>2) )
			PWM_abs [0] = deadband;
		pwm_display [0] = -(long signed)PWM_abs [0];
	}
	else {
		forward [0] = TRUE;
		PWM_abs [0] = PWM [0]/10L;
		/* reduce deadband effect (a minimal PWM output is required to cause any motion (i.e. overcome friction) */
		if ( (PWM_abs [0] < deadband) && (PWM_abs [0] > deadband>>2) )
			PWM_abs [0] = deadband;
		pwm_display [0] = (long signed)PWM_abs [0];
	}
	
			
	/* set new PWM tick widths (is relative to PWM driver's defined Period) */
	/* (this function requires the input argument expressed as 10,000 for 100%) */
	pwm_width [0] = pwm_gain * PWM_PERCENTAGE_TO_WIDTH( &PWMD4, PWM_abs [0] ) / 100;
	pwm_width [1] = pwm_gain * PWM_PERCENTAGE_TO_WIDTH( &PWMD4, PWM_abs [1] ) / 100;
	
	chSysLockFromIsr();
	
	/* Pololu Motor Driver protocol used:  "Sign-Magnitude (drive-brake)"
	 *		-PWM input sets speed
	 *		-DIR pin 0:  forward
	 *		-DIR pin 1:  backward */
	
  /* Changes the channels pulse width, the change will be effective
     starting from the next cycle. */
	/* AXIS 1: */
	pwmEnableChannelI(&PWMD4, 3, pwm_width [0]);
	if (forward [0]) {
		palSetPad(GPIOB, 13);		/* CW direction,PB.13 ON */
	}
	else {
		palClearPad(GPIOB, 13);			/* CCW direction,PB.13 OFF*/
	}

	/* AXIS 2: */
	pwmEnableChannelI(&PWMD4, 1, pwm_width [1]);
	if (forward [1]) {
		palSetPad(GPIOE, 13);		/* CW direction,PE.13 ON */
	}
	else {
		palClearPad(GPIOE, 13);			/* CCW direction,PE.13 OFF */
	}
		
  chSysUnlockFromIsr();
	
	palTogglePad(GPIOD, GPIOD_LED4);		/* green toggle */			

}

/* GPT3 configuration
 * 10,000 Hz with callback.
 */
static const GPTConfig gpt3cfg = {
  10000,  /* 10 KHz timer clock.*/
  gpt3cb    /* Timer callback.*/
};

/* ADC end conversion callback (samples averaged).
 * The samples are averaged per channel.
 */
static void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

  (void) buffer; (void) n;
	int adc_chan_num, i;
	long sum;
	
	
  /* Note, only in the ADC_COMPLETE state because the ADC driver fires an
     intermediate callback when the buffer is half full.*/
  if (adcp->state == ADC_COMPLETE) {
    /* Calculates the average values from the ADC samples.*/
			
		palTogglePad(GPIOD, GPIOD_LED5);		/* RED toggle */		
			
		for ( adc_chan_num = 0; adc_chan_num < ADC_GRP1_NUM_CHANNELS ; adc_chan_num++ ) {
			sum = 0;
			for ( i = 0; i < ADC_GRP1_BUF_DEPTH; i++ )				
				sum += samples [adc_chan_num + i*ADC_GRP1_NUM_CHANNELS];
			avg_ch [adc_chan_num] = sum / ADC_GRP1_BUF_DEPTH;
		}
	}
		
/*		
    avg_ch [0] = (samples[0] + samples[4] + samples[8] + samples[12]) / 4;
    avg_ch [1] = (samples[1] + samples[5] + samples[9] + samples[13]) / 4;
		avg_ch [2] = (samples[2] + samples[6] + samples[10] + samples[14]) / 4;
		avg_ch [3] = (samples[3] + samples[7] + samples[11] + samples[15]) / 4;
  } */
}

/* Display output **Thread**
 * XsimCTRL protocol
 */
static WORKING_AREA(waThread2, 512);
static msg_t Thread2(void *arg) {

  (void)arg;
  chRegSetThreadName("disp_out");
		
	static char stateBlock[30];		/* display output string */
	

	/* won't work, always zero, weird, even though pwm_width [0] <> 0 and everything globals!
	pwidth512 = (((long signed)pwm_width [0]) * 1000L) / 512L;                 */

	/* displayed values in XsimCTRL:
	axis 1 section:
			axis 1 set position
			axis 1 actual position (unaltered potentiometer reading)
			axis 1 PWM% output / pwm_gain (so always in 1-100% range), in tenths of percent (
			current feedback mode (0, 1, 2, etc.)
	axis 2 section:
			axis 2 set position
			axis 2 actual position (unaltered potentiometer reading)
			axis 1 motor input current in tenths of amps (Pololu outout)
			current feedback agression setting (0, 1, 2, 3, 4, etc.)
*/	
	while(TRUE) {
		sprintf(stateBlock,"[%02X%04X%04X%02X%02X%04X%04X%02X]",axis [0],avg_ch [0] >> 4,(uint16_t)(pwm_display [0] * pwm_gain / 1000),(uint16_t)FB_current->mode+1,axis [1],avg_ch [1] >> 4,current [0],(uint16_t)FB_current->agression+1);
//WORKS		sprintf(stateBlock,"[%02X%04X%04X%02X%02X%04X%04X%02X]",axis [0],avg_ch [0] >> 4,(uint16_t)(pwm_display [0] * pwm_gain / 1000),0,axis [1],avg_ch [1] >> 4,current [0],0);
		sdAsynchronousWrite(&SD2, (uint8_t*) stateBlock,26);
		
		
//		chprintf(chp,"stateBlock: [%26s]\n\r",stateBlock);
	
		chThdSleepMilliseconds(20);	/* limit display update to 50 Hz */
	}
	return 0;
}
	
/* Serial stream collection **Thread**
 * Tight loop queries for data byte reception.
 * Upon reception, data parsed for delimiters to find axis command frame.
 */
static WORKING_AREA(waThread1, 256);
static msg_t Thread1(void *arg) {

  (void)arg;
  chRegSetThreadName("ser_in");

	static const long signed ADC_half_x_400 = 2032L * 400L;		/* pre-calc  */
	static const char delimiter[3] = {0x41,0x42,0xFF};
	const int DELIM_LEN = 3;
	const int DATA_LEN = 2;	/* 1 byte of motion data in stream per axis per sent update */
	bool_t delimiterFound = FALSE;
	int bytesAvail, i=0;
	char b;
	
	/* THIS USED TO BE IN PID LOOP
	 *	command_1 = ((((long signed)axis[0] << 4) - 2032L) * calibration.ADC_stroke [0] * valet_scale) / ADC_half_x_400 + calibration.ADC_centre [0];
	 *	command_2 = ((((long signed)axis[1] << 4) - 2032L) * calibration.ADC_stroke [1] * valet_scale) / ADC_half_x_400 + calibration.ADC_centre [1];
	 */
	
	
	listening_active = FALSE;		/* until activated externally */
		
	while (TRUE) {			
		
		while ( !listening_active )	
			chThdSleepMilliseconds(500);		/* check every half second if I should start listening or not */
		
		chThdSleepMilliseconds(1);	/* seems to work good with this small delay @57600 baud */		
		bytesAvail = chQSpaceI(&SD2.iqueue);		/* number of bytes avail to read */		
		
		if (bytesAvail > 0) {
			
//		b = (char) chIOGet((BaseChannel *)&SD2);	/* grab a byte */
  		b = (char) chIQGet(&SD2.iqueue);					/* this works too, but still 65 spikes */
//			b = (char) chSequentialStreamGet(&SD2);							/* how 'bout this one? */
			
			if (delimiterFound) {		/* save next DATA_LEN bytes as axiis values */
				axis[i] = b; 
				axis_command [i] = ((((long signed) b << 4) - 2032L) * calibration.ADC_stroke[i] * valet_scale) / ADC_half_x_400 + calibration.ADC_centre [i];
				++i;
				if (i == DATA_LEN) {		/* restart search */
					delimiterFound = FALSE;
					i=0;
				}
			}
			else
			if ( b == delimiter[i] ) {	/* next delimiter token found */
				++i;
				if ( i==DELIM_LEN ) {
					delimiterFound = TRUE;	/* entire delimiter found */					
					i=0;
				}
			}
			else
				i=0;
		}
		
	}
	return 0;
}
	
/*===========================================================================*/
/*	Helpers																																	 */
/*===========================================================================*/

/*	pwm_set
 *	PWM output started or stopped.
 *	@params: chp			stream for status messages.
 *	@params: enabling	TRUE or FALSE
 */			
void pwm_set( BaseSequentialStream *chp, bool_t enabling ) {		
		
	int waitOK;

	if ( enabling == TRUE ) {
		/* pwm on stuff here */
		if ( PWMD4.state == PWM_READY )
			chprintf(chp,"  PWM channels already running.\r\n");	
		else {
			pwmStart(&PWMD4, &pwmcfg);
			chThdSleepMilliseconds(10);	/* small delay */
			pwmEnableChannelI(&PWMD4, 1, 0);	/* start ORANGE channel off */
			pwmEnableChannelI(&PWMD4, 3, 0);	/* start BLUE channel off */
			waitOK=5;
			while ( (PWMD4.state != PWM_READY) && --waitOK  ) {
					chprintf(chp,"  Starting all PWM channels...\r\n");
					chThdSleepMilliseconds(1000);
			}
			if (!waitOK)
				chprintf(chp,"  >>> ERROR: Could not start PWM channels.\r\n");
			else
			chprintf(chp,"  PWM channels successfully started.\r\n");
		}
	}
	else {
		/* pwm off stuff here */
		if ( PWMD4.state == PWM_STOP )
			chprintf(chp,"  PWM channels already stopped.\r\n");	
		else {
			pwmDisableChannel(&PWMD4, 1);	// ORANGE channel off
			pwmDisableChannel(&PWMD4, 3);	// BLUE channel off
			pwmStop(&PWMD4);
			waitOK=5;
			while ( (PWMD4.state != PWM_STOP) && --waitOK  ) {
					chprintf(chp,"  Stopping all PWM channels...\r\n");
					chThdSleepMilliseconds(1000);
			}
			if (!waitOK)
				chprintf(chp,"  >>> ERROR: Could not stop PWM channels.\r\n");
			else
				chprintf(chp,"  PWM channels successfully stopped.\r\n");					
		}		
	}

}

/*	pid_set
 *	Start/Stop PID motion processing loop.
 *	@params: chp			stream for status messages.
 *	@params: enabling	TRUE or FALSE
 */			
void pid_set( BaseSequentialStream *chp, bool_t enabling ) {
		
	int waitOK;

	if ( enabling == TRUE ) {
		/* pid on stuff here */
		if ( GPTD3.state == GPT_CONTINUOUS )
				chprintf(chp,"  Motion processing already started.\r\n");
		else {
			gptStart(&GPTD3, &gpt3cfg);		
			gptPolledDelay(&GPTD3, 10); /* Small delay.*/
			gptStartContinuous(&GPTD3, 10000/PID_FREQ);		/* PID loop interval (1000 Hz)*/	
			waitOK = 5;
			while ( (GPTD3.state != GPT_CONTINUOUS) && --waitOK  ) {
				chprintf(chp,"  Enabling motion processing...\r\n");
				chThdSleepMilliseconds(1000);
			}
			if (!waitOK)
				chprintf(chp,"  >>> ERROR: Could not startup motion processing loop timer.\r\n");
			else
				chprintf(chp,"  Motion processing successfully started.\r\n");			
		}
	}
	else {
		/* pid off stuff here */
		if ( GPTD3.state == GPT_STOP )
			chprintf(chp,"  Motion processing already stopped.\r\n");
		else {
			gptStopTimer(&GPTD3);
			gptStop(&GPTD3);
			waitOK = 5;
			while ( (GPTD3.state != GPT_STOP) && --waitOK ) {
				chprintf(chp,"  Stopping motion processing...\r\n");
				chThdSleepMilliseconds(1000);
			}
			if (!waitOK)
				chprintf(chp,"  >>> ERROR: Could not stop motion processing loop timer.\r\n");
			else
				chprintf(chp,"  Motion processing successfully stopped.\r\n");
		}
	}
	
}

/*	command_arm
 *	Activates PWM output at specified gain and starts motion processing (sensor/PID loop).
 *	Centers all axiis.
 *	@params: chp		stream for status messages.
 */		
void command_arm( BaseSequentialStream *chp, bool_t arming ) {

	char line[12];
	
	if ( arming == TRUE ) {
		/* arming stuff here */
		/* configure all motion-processing parameters first, including those where system must be at a standstill */
		pwm_gain = pwm_gain_set;
		PID_set_tune(chp);
		
		chprintf(chp,"  Arming system with settings:\r\n");
		chprintf(chp,"    Feedback mode: %d (%s)\r\n",1 + FB_current->mode, FB_current->desc);
		chprintf(chp,"    Agression: %d (%s)\r\n",1 + FB_current->agression,FB_agression_desc[FB_current->agression]);
		chprintf(chp,"    agr_multiplier: %f\r\n\r\n",FB_current->scaler [FB_current->agression]);
		if ( FB_current->mode == FB_PID )
			chprintf(chp,"    Derivative type: %s\r\n",(char*) (FB_current->derivative_alt == TRUE) ? "alternate" : "regular");
		chprintf(chp,"    Valet mode: %s\r\n",(valet_mode==0)?"OFF":itoa(valet_mode,line));
		chprintf(chp,"    PWM gain:   %d\r\n", pwm_gain);
		
		error_prev [0] = error_prev [1] = avg_ch_prev [0] = avg_ch_prev [1] =  \
		error_i [0] = error_i [1] = max_d [0] = min_d [0] = previous_derivative [0] = 0L;		/* reduce startup jerks */
		pwm_set ( chp,TRUE);
		pid_set (chp,TRUE);
	}
	else {
		/*disarming stuff here */		
		chprintf(chp,"  Disarming system.\r\n");
		pid_set (chp,FALSE);
		chThdSleepMilliseconds(100);
		pwm_gain = 0;
		chThdSleepMilliseconds(100);
		pwm_set (chp,FALSE);		
	}
	
}

/*	command_stop
 *	Fully and safely stops all motion processing and and PWM output.
 *	Serial stream listening disabled.
 *	@params: chp		stream for status messages.
 */			
void command_stop( BaseSequentialStream *chp ) {		
	
	command_arm ( chp, FALSE );
	if ( listening_active )	chprintf(chp,"  Serial stream listening stopped.\r\n");
	else chprintf(chp,"  Serial stream listening already stopped.\r\n");
	listening_active = FALSE;	
}	

/*	command_center
 *	Safely returns all axiis to their calibrated center positions
 *	and holds position there (systems remain activated afterwards).
 *	@params: chp		stream for status messages.
*/		
void command_center( BaseSequentialStream *chp ) {	

	int pwm_gain_set_temp = pwm_gain_set;
	int valet_mode_temp = valet_mode;
	
	if ( listening_active )	chprintf(chp,"  Serial stream listening stopped.\r\n");
	else chprintf(chp,"  Serial stream listening already stopped.\r\n");
	listening_active = FALSE;
	chThdSleepMilliseconds(100);
	chprintf(chp,"  Centering all axiis.\r\n");
	/* slowly move axis back to center point */
	pwm_gain_set = 20;
	valet_mode = 0;	/* otherwise the small gain may not move motor */
	chprintf(chp,"  Motor speeds temporarily reduced to 20%%.\r\n");
	command_arm ( chp, TRUE );
	axis_command [0] = 127 << 4;		/* centering commands sent */
	axis_command [1] = 127 << 4;
	chprintf(chp,"  Axis 1 commanded to home position: %d.\r\n",calibration.ADC_centre [0]);
	chprintf(chp,"  Axis 2 commanded to home position: %d.\r\n",calibration.ADC_centre [1]);
	/* hack here until figure out how to monitor when have arrived at home position */
	chprintf(chp,"  Waiting 5 seconds...\r\n");
	chThdSleepMilliseconds(5000);
	pwm_gain_set = pwm_gain_set_temp;
	pwm_gain = 0;
	valet_mode = valet_mode_temp;
}

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WA_SIZE(2048)
#define TEST_WA_SIZE    THD_WA_SIZE(256)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, size;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
  chprintf(chp, "heap fragments   : %u\r\n", n);
  chprintf(chp, "heap free total  : %u bytes\r\n", size);
}


void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[])
{
  static const char *states[] = {THD_STATE_NAMES};
  Thread *tp;

  (void)argv;
  if (argc > 0)
  {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "%10s %10s %10s %6s %6s %11s %7s\r\n",
           "name", "add", "stack", "prio", "refs", "state", "time");
  tp = chRegFirstThread();
  do
  {
    chprintf(chp, "%10s %.10lx %.10lx %6lu %6lu %11s %7lu\r\n",
             (uint32_t)tp->p_name, (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
             (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
             states[tp->p_state], (uint32_t)tp->p_time);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

/*	cmd_parameters
 *	Display and/or change all system parameters
 *	@params: chp		stream for status messages.
*/		
static void cmd_parameters(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: params\r\n");
    return;
  }

	char line[12];
	int temp;
	long signed Kptemp;
	long signed Kitemp;
	long signed Kdtemp;
	bool_t redo, redo2, redo3, recalcPID;
	redo=redo2=redo3=recalcPID=FALSE;

	PID_set_tune(chp);

	chprintf(chp,"\r\nPARAMETERS command entered ========\r\n");
	/* display current parameters */
	chprintf(chp,"\r\n  Current  Parameters:\r\n");	
	if ( FB_current->mode == FB_PID_IND ) {
		chprintf(chp,"    Kp: %d\r\n",Kp);
		chprintf(chp,"    Ki: %d\r\n",Ki);
		chprintf(chp,"    Kd: %d\r\n",Kd);
	}
	chprintf(chp,"    PWM gain: %d%%\r\n",pwm_gain_set);
	chprintf(chp,"    Valet mode: %s\r\n",(valet_mode==0)?"OFF":itoa(valet_mode,line));	
	chprintf(chp,"    Deadband %2.2f%%.\r\n",(float)deadband / 100.0 );
	// chprintf(chp,"    POT noise floor (in raw ADC output) %d.\r\n",pot_noise_floor);
	// chprintf(chp,"    POT noise floor_10000 %d.\r\n",pot_noise_floor_10000 );
	chprintf(chp,"    Feedback mode: %d (%s)\r\n",1 + FB_current->mode, FB_current->desc);
	if ( FB_current->mode != FB_PID_IND ) {
			chprintf(chp,"    Agression: %d (%s)\r\n",1 + FB_current->agression,FB_agression_desc[FB_current->agression]);
			chprintf(chp,"    agr_multiplier: %f\r\n\r\n",FB_current->scaler [FB_current->agression]);	
		if ( FB_current->mode == FB_PID )
			chprintf(chp,"    Derivative type: %s\r\n",(char*) (FB_current->derivative_alt == TRUE) ? "alternate" : "regular");
			//	chprintf(chp,"    Integral_scaler: %f\r\n",Integral_scaler);
		chprintf(chp,"    Tp: %f\r\n",Tp);
		chprintf(chp,"    theta_p: %f\r\n",theta_p);
		chprintf(chp,"    Kprocess: %f\r\n",Kprocess);
		chprintf(chp,"    Tc: %f\r\n",Tc);
		chprintf(chp,"    Ti: %f\r\n",Ti);
		chprintf(chp,"    Td: %f\r\n",Td);
	//	chprintf(chp,"    alpha_filter: %f\r\n",alpha_filter);
		chprintf(chp,"    Kc: %f\r\n",Kc/1000.0);
	}
	chprintf(chp,"\r\n  Do you want to change anything (y/n)? ");
	shellGetLine(chp, line, sizeof(line));
	if ( (*line == 'y') || (*line == 'Y') )
		if ( PWMD4.state != PWM_STOP ) {
			chprintf(chp,"\r\n  You cannot change parameters while\r\n");
			chprintf(chp,"  system is running.\r\n");
			chprintf(chp,"  >>> Issue 'stop' command first.\r\n\r\n");
			chprintf(chp,"  Press ENTER to continue. ");
			shellGetLine(chp, line, sizeof(line));	
		}
		else {
		
		
			// start PID Ind. tuning parameters
			chprintf(chp,"\r\n  Change PID Tuning values (y/n)? ");
			shellGetLine(chp, line, sizeof(line));
			if ( (*line == 'y') || (*line == 'Y') ) {
				redo = TRUE;
				recalcPID=TRUE;
				Kptemp = Kp;
				Kitemp = Ki;
				Kdtemp = Kd;

				chprintf(chp,"\r\n\r\n  Enter new PID Tuning values.\r\n");					
			}
			else
				redo = FALSE;
			while ( redo ) {
				chprintf(chp,"    Enter Kp (currently %d): ",Kp);
				shellGetLine(chp, line, sizeof(line));
				Kptemp = atoi (line);
				chprintf(chp,"    Enter Ki (currently %d): ",Ki);
				shellGetLine(chp, line, sizeof(line));
				Kitemp = atoi (line);
				chprintf(chp,"    Enter Kd (currently %d): ",Kd);
				shellGetLine(chp, line, sizeof(line));
				Kdtemp = atoi (line);
				chprintf(chp,"\r\n  The new values entered were:\r\n");
				chprintf(chp,"    Kp: %d\r\n",Kptemp);
				chprintf(chp,"    Ki: %d\r\n",Kitemp);
				chprintf(chp,"    Kd: %d\r\n",Kdtemp);												
				chprintf(chp,"\r\n  Are these correct (y/n)? ");
				shellGetLine(chp, line, sizeof(line));
				if ( (*line == 'y') || (*line == 'Y') ) {
					redo = FALSE;
					Kp = Kptemp;
					Ki = Kitemp;
					Kd = Kdtemp;
				}
				else {
					chprintf(chp,"\r\n  Re-enter PID constants.\r\n");
					redo = TRUE;
				}
			}  //END PID-ind tuning values
			
			// start IMC tuning parameters
			chprintf(chp,"\r\n  Change IMC Tuning values (y/n)? ");
			shellGetLine(chp, line, sizeof(line));
			if ( (*line == 'y') || (*line == 'Y') ) {
				redo = TRUE;
				recalcPID=TRUE;
				chprintf(chp,"\r\n\r\n  Enter new IMC Tuning values.\r\n");					
			}
			else
				redo = FALSE;
			while ( redo ) {
				chprintf(chp,"    Enter Tp: ");
				shellGetLine(chp, line, sizeof(line));
				Tp = (double)atof (line);
				chprintf(chp,"    Enter theta_p: ");
				shellGetLine(chp, line, sizeof(line));
				theta_p = (double)atof (line);
				chprintf(chp,"    Enter Kprocess: ");
				shellGetLine(chp, line, sizeof(line));
				Kprocess = (double)atof (line);
//				chprintf(chp,"    Enter Integral_scaler: ");
//				shellGetLine(chp, line, sizeof(line));
//				Integral_scaler = (double)atof (line);		
				chprintf(chp,"\r\n  The new values entered were:\r\n");
				chprintf(chp,"    Tp: %f\r\n",Tp);
				chprintf(chp,"    theta_p: %f\r\n",theta_p);
				chprintf(chp,"    Kprocess: %f\r\n",Kprocess);
//				chprintf(chp,"    Integral_scaler: %f\r\n",Integral_scaler);
				chprintf(chp,"\r\n  Are these correct (y/n)? ");
				shellGetLine(chp, line, sizeof(line));
				if ( (*line == 'y') || (*line == 'Y') ) {
					redo = FALSE;
				}
				else {
					chprintf(chp,"\r\n  Re-enter PID constants.\r\n");
					redo = TRUE;
				}
			}  //END IMC tuning values
			
			
			//start FEEDBACK settings
			chprintf(chp,"\r\n  Change feedback modes/settings (y/n)? ");
			shellGetLine(chp, line, sizeof(line));
			if ( (*line == 'y') || (*line == 'Y') ) {
				redo = TRUE;
				recalcPID = TRUE;
				chprintf(chp,"\r\n\r\n  Enter new feedback settings.\r\n");					
			}
			else
				redo = FALSE;
			while ( redo ) {
				redo2 = TRUE;
				while (redo2) {
					chprintf(chp,"    Current feedback mode: %d (%s)\r\n",1 + FB_current->mode, FB_current->desc);
					chprintf(chp,"      1 = P only\r\n");
					chprintf(chp,"      2 = PI only\r\n");
					chprintf(chp,"      3 = PID dependant\r\n");
					chprintf(chp,"      4 = PID independant\r\n");
					chprintf(chp,"    Enter new feedback mode (1-%d): ",MAX_FB_MODE+1);
					shellGetLine(chp, line, sizeof(line));
					temp = atoi (line);
					if ( (temp<1) || (temp>MAX_FB_MODE + 1) ) {
						redo2 = TRUE;
						chprintf(chp,"\r\n  >>> ERROR: Feedback mode must be 1-%d.\r\n",MAX_FB_MODE + 1);
						chprintf(chp,"      Press ENTER to try again.\r\n\r\n");
					}
					else {
						redo2 = FALSE;
						FB_current = FB_mode + (temp - 1);
					}
				}
								
				if ( FB_current->mode == FB_PID ) {
					redo2 = TRUE;
					while (redo2) {
						chprintf(chp,"    Current PID derivative type: %s\r\n",(char*) (FB_current->derivative_alt == TRUE) ? "alternate" : "regular");
						chprintf(chp,"      1 = Regular (de/dt)\r\n");
						chprintf(chp,"      2 = Alternate (dPV/dt)\r\n");
						chprintf(chp,"    Enter new derivative mode (1-2): ");
						shellGetLine(chp, line, sizeof(line));
						temp = atoi (line);
						if ( (temp<1) || (temp>2) ) {
							redo2 = TRUE;
							chprintf(chp,"\r\n  >>> ERROR: Derivative type must be 1 or 2.\r\n");
							chprintf(chp,"      Press ENTER to try again.\r\n\r\n");
						}
						else {
							redo2 = FALSE;
							FB_current->derivative_alt = (bool_t)(temp-1);
						}
					}
				}				

				if ( FB_current->mode != FB_PID_IND ) {
					redo2 = TRUE;				
					while (redo2) {
						chprintf(chp,"    Current agression: %d - %s\r\n",1 + FB_current->agression,FB_agression_desc[FB_current->agression]);
						chprintf(chp,"      1 = Slow        \r\n");
						chprintf(chp,"      2 = Moderate    \r\n");
						chprintf(chp,"      3 = Normal      \r\n");
						chprintf(chp,"      4 = Fast        \r\n");
						chprintf(chp,"      5 = BoneCrusher \r\n");
						chprintf(chp,"    Enter new agression (1-5): ");
						shellGetLine(chp, line, sizeof(line));
						temp = atoi (line);
						if ( (temp<1) || (temp>5) ) {
							redo2 = TRUE;
							chprintf(chp,"\r\n  >>> ERROR: Agression mode must be 1-%d.\r\n",MAX_FB_AGRESSION + 1);
							chprintf(chp,"      Press ENTER to try again.\r\n\r\n");
						}
						else {
							redo2 = FALSE;
							FB_current->agression = temp-1;
						}
					}
					
					if ( (FB_current->mode == FB_PI)  || (FB_current->mode == FB_PID) ) {
						/* agr multiplier overide */
						chprintf(chp,"    Change agr_multiplier (currently %f) (y/n)? ",FB_current->scaler [FB_current->agression]);					
						shellGetLine(chp, line, sizeof(line));
						if ( (*line == 'y') || (*line == 'Y') ) {
							chprintf(chp,"    Enter new agr_multiplier: ");						
							shellGetLine(chp, line, sizeof(line));
							FB_current->scaler [FB_current->agression] = (double)atof (line);						
						}				
					} /* END agr_multipler overide */
				}					
				
				chprintf(chp,"\r\n  New fedback settings entered were:\r\n");
				chprintf(chp,"    Feedback mode: %d (%s)\r\n",1 + FB_current->mode, FB_current->desc);
				if ( FB_current->mode != FB_PID_IND ) {
					chprintf(chp,"    Agression: %d (%s)\r\n",1 + FB_current->agression,FB_agression_desc[FB_current->agression]);
					chprintf(chp,"    agr_multiplier: %f\r\n\r\n",FB_current->scaler [FB_current->agression]);
					if ( FB_current->mode == FB_PID )
						chprintf(chp,"    Derivative type: %s\r\n",(char*) (FB_current->derivative_alt == TRUE) ? "alternate" : "regular");				
				}
				chprintf(chp,"\r\n  Are these correct (y/n)? ");			
				shellGetLine(chp, line, sizeof(line));
				if ( (*line == 'y') || (*line == 'Y') ) {
					redo = FALSE;					
				}
				else {
					chprintf(chp,"\r\n  Re-enter feedback settings.\r\n");
					redo = TRUE;
				}
			}  			
			//END FEEDBACK settings
			
			if (recalcPID) {
				PID_set_tune(chp);
				if ( FB_current->mode != FB_PID_IND ) {
					chprintf(chp,"\r\n  New calculated IMC control values are:\r\n");
					chprintf(chp,"    Tc: %f\r\n",Tc);
					chprintf(chp,"    Ti: %f\r\n",Ti);					
					chprintf(chp,"    Td: %f\r\n",Td);
	//				chprintf(chp,"    alpha_filter: %f\r\n",alpha_filter);
					chprintf(chp,"    Kc: %f\r\n",Kc/1000.0);
				}
			}

			chprintf(chp,"  Change PWM gain (y/n)? ");
			shellGetLine(chp, line, sizeof(line));
			if ( (*line == 'y') || (*line == 'Y') ) {
				redo = TRUE;
				while (redo) {
					chprintf(chp,"  Enter new PWM gain: ");
					shellGetLine(chp, line, sizeof(line));
					temp = atoi (line);
					if ( temp>100 || temp<0 ) {
						redo = TRUE;
						chprintf(chp,"  >>> ERROR: PWM gain must be 0-100.\r\n");
						chprintf(chp,"      Press ENTER to try again.\r\n\r\n");
					}
					else {
						redo = FALSE;
						pwm_gain_set = temp;
						chprintf(chp,"  New PWM gain: %d.\r\n",pwm_gain_set);
					}
				}
			}
						
						
			chprintf(chp,"  Change deadband (currently %5.2f%%) (y/n)? ",(float)deadband / 100.0 );
			shellGetLine(chp, line, sizeof(line));
			if ( (*line == 'y') || (*line == 'Y') ) {
				redo = TRUE;
				while (redo) {
					chprintf(chp,"  Enter new deadband %%: ");
					shellGetLine(chp, line, sizeof(line));
					temp = (int)(atof (line) * 100.0);
					if ( temp<0 ) {
						redo = TRUE;
						chprintf(chp,"  >>> ERROR: deadband can't be negative.\r\n");
						chprintf(chp,"      Press ENTER to try again.\r\n\r\n");
					}
					else {
						redo = FALSE;
						deadband = temp;
						chprintf(chp,"  New deadband %5.2f%%.\r\n",(float)deadband / 100.0 );
					}
				}
			}
						
			// chprintf(chp,"  Change POT noise floor (currently %d) (y/n)? ",pot_noise_floor);
			// shellGetLine(chp, line, sizeof(line));
			// if ( (*line == 'y') || (*line == 'Y') ) {
				// redo = TRUE;
				// while (redo) {
					// chprintf(chp,"  Enter new POT noise floor (in raw ADC output): ");
					// shellGetLine(chp, line, sizeof(line));
					// temp = atoi (line);
					// if ( temp<0 ) {
						// redo = TRUE;
						// chprintf(chp,"  >>> ERROR: POT noise floor can't be negative.\r\n");
						// chprintf(chp,"      Press ENTER to try again.\r\n\r\n");
					// }
					// else {
						// redo = FALSE;
						// pot_noise_floor = temp;
						// chprintf(chp,"  New POT noise floor %d.\r\n",pot_noise_floor );
						// pot_noise_floor_10000 = pot_noise_floor * 10000L * 2 / 2048L;	/* double the noise amplitude to be safe */
						// chprintf(chp,"  New POT noise floor_10000 %d.\r\n",pot_noise_floor_10000 );
					// }
				// }
			// }
						
						
			chprintf(chp,"  Change Vale mode (y/n)? ");
			shellGetLine(chp, line, sizeof(line));
			if ( (*line == 'y') || (*line == 'Y') ) {
				redo = TRUE;
				while (redo) {
					chprintf(chp,"\r\n    0 = OFF (no reduction)\r\n");
					chprintf(chp,"    1 = 25%% reduction\r\n");
					chprintf(chp,"    2 = 50%% reduction\r\n");
					chprintf(chp,"    3 = 75%% reduction\r\n");
					chprintf(chp,"  Enter new Valet Mode (0-3): ");
					shellGetLine(chp, line, sizeof(line));
					temp = atoi (line);
					if ( !(temp==0 || temp==1 || temp==2 || temp==3) ) {
						redo = TRUE;
						chprintf(chp,"\r\n  >>> ERROR: Valet mode must be 0-3.\r\n");
						chprintf(chp,"      Press ENTER to try again.\r\n\r\n");
					}
					else {
						redo = FALSE;
						valet_mode = temp;
						chprintf(chp,"  New Valet mode: %s\r\n",(valet_mode==0)?"OFF":itoa(valet_mode,line));
						}
					}
			}					
		}
	else
		chprintf(chp,"  No changes made.\r\n");
	chprintf(chp,"Command complete ==========\r\n");
	
}

/*	cmd_calibrate
 *	Wizard to guide user through calibration steps.
 *	Center and end points set for each axis.
 *	@params: chp		stream for status messages.
*/	
static void cmd_calibrate(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: calibrate\r\n");
    return;
  }

	char line[12];
	int center,ep1,ep2,stroke,temp1,temp2;
	bool_t good_capture = FALSE;
	bool_t redo_wanted = TRUE;
	
	chprintf(chp,"\r\nCALIBRATE command entered ========\r\n");	
		/* display current calibration */
	chprintf(chp,"\r\n  Current Axis 1 Calibration:\r\n");	
	chprintf(chp,"    Stroke: %d%%\r\n",(int)(100.0*((float)calibration.ADC_stroke [0])/2047.0));
	chprintf(chp,"    End Points: (%d - %d)\r\n",(int)(calibration.ADC_centre [0] - calibration.ADC_stroke [0]), \
																										 (int)(calibration.ADC_centre [0] + calibration.ADC_stroke [0]) );
	chprintf(chp,"\r\n  Current Axis 2 Calibration:\r\n");	
	chprintf(chp,"    Stroke: %d%%\r\n",(int)(100.0*((float)calibration.ADC_stroke [1])/2047.0));
	chprintf(chp,"    End Points: (%d - %d)\r\n",(int)(calibration.ADC_centre [1] - calibration.ADC_stroke [1]), \
																										 (int)(calibration.ADC_centre [1] + calibration.ADC_stroke [1]) );
	chprintf(chp,"\r\n  Do you want to change anything (y/n)? ");
	shellGetLine(chp, line, sizeof(line));
	if ( (*line == 'y') || (*line == 'Y') )
		if ( PWMD4.state != PWM_STOP ) {
			chprintf(chp,"\r\n  You cannot calibrate system while\r\n");
			chprintf(chp,"  simulator is running.\r\n");
			chprintf(chp,"  >>> Issue 'stop' command first.\r\n\r\n");
			chprintf(chp,"  Press ENTER to continue. ");
		}
		else {
		
			chprintf(chp,"  >>>>\r\n");
			chprintf(chp,"  >>>> WARNING:\r\n");
			chprintf(chp,"  >>>>\r\n");
			chprintf(chp,"  >>>> For safety reasons switch off all motor power\r\n");
			chprintf(chp,"  >>>> before proceeding. Leave all other sensors and\r\n");
			chprintf(chp,"  >>>> electronics powered on.\r\n");
			chprintf(chp,"  >>>>\r\n");
			chprintf(chp,"  >>>> TURN OFF ALL MOTOR POWER BEFORE PROCEEDING.\r\n");
			chprintf(chp,"  >>>>\r\n");
			chprintf(chp,"\r\n  Press ENTER to begin. ");
			shellGetLine(chp, line, sizeof(line));	

			chprintf(chp,"\r\n  Setting motor output: OFF.\r\n");	
			chprintf(chp,"  Setting motion processing and sensors: ON.\r\n\r\n");
			pwm_gain = 0;
			pwm_set( chp,TRUE);
			pid_set( chp,TRUE);

			while ( redo_wanted ) {
				while ( !good_capture ) {
					good_capture = FALSE;
					chprintf(chp,"\r\n  Move axis 1 to center, then press ENTER. ");
					shellGetLine(chp, line, sizeof(line));
					chSysLockFromIsr();
					adcStartConversionI(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
					chSysUnlockFromIsr();
					center = avg_ch [0];	
					chprintf(chp,"\r\nCenter captured as: %d.\r\n",center);
					chprintf(chp,"\r\n  Move axis 1 all the way in one direction. Hold it there and press ENTER. ");
					shellGetLine(chp, line, sizeof(line));
					chSysLockFromIsr();
					adcStartConversionI(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
					ep1 = avg_ch [0];				
					chSysUnlockFromIsr();
					temp1=ep1-center; temp1= (temp1<0) ? -temp1 : temp1;
					chprintf(chp,"\r\nEndpoint #1 captured as: %d (%d from center).\r\n",ep1,temp1);
					chprintf(chp,"\r\n  Move axis 1 all the way in the opposite direction. Hold it there and press ENTER. ");
					shellGetLine(chp, line, sizeof(line));
					chSysLockFromIsr();
					adcStartConversionI(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
					ep2 = avg_ch [0];				
					chSysUnlockFromIsr();
					temp2=ep2-center; temp2= (temp2<0) ? -temp2 : temp2;
					chprintf(chp,"\r\nEndpoint #2 captured as: %d (%d from center).\r\n",ep2,temp2);
					if ( (center>ep1 && center<ep2) || (center>ep2 && center<ep1) )
						good_capture=TRUE;
					else {
						good_capture = FALSE;
						chprintf(chp,"  >>> ERROR: Your center point does not lie between\r\n");
						chprintf(chp,"             the endpoints. Repeat the captures.\r\n\r\n");
					}
				}
				/* whichever distance from centre to endpoint was smallest, that will be our chosen stroke value */
				if ( temp1<temp2 ) {
					chprintf(chp,"\r\n  Endpoint #1 has the smallest stroke (%d).\r\n",temp1);
					stroke = temp1;
				}
				else {
					chprintf(chp,"\r\n  Endpoint #2 has the smallest stroke (%d).\r\n",temp2);
					stroke = temp2;
				}
				chprintf(chp,"  Calibrated motion set to: %d +/- %d.\r\n",center,stroke);
				chprintf(chp,"  This is a %d%% stroke (%d-%d), centered at %d.\r\n",(int)(100.0*((float)stroke)/2047.0), \
																														(int)(center-stroke),
																														(int)(center+stroke),
																														(int)center);
				chprintf(chp,"  Do you accept this calibration (y/n)? ");
				shellGetLine(chp, line, sizeof(line));
				if ( (*line == 'y') || (*line == 'Y') )
					redo_wanted = FALSE;
				else {
					chprintf(chp,"  New calibration discarded. Try again.\r\n\r\n");
					good_capture = FALSE;
				}
			}
			chprintf(chp,"  Calibration complete.\r\n");
			calibration.ADC_centre [0] = center;
			calibration.ADC_stroke [0] = stroke;
			command_arm( chp, FALSE );
		}
	else
		chprintf(chp,"  No changes made.\r\n");
	chprintf(chp,"Command complete ==========\r\n");
}

/*	cmd_go
 *	Centers all axiis, arms and enables all systems.
 *	@params: chp		stream for status messages.
*/	
static void cmd_go(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: go\r\n");
    return;
  }

	int waitOK;
	
	chprintf(chp,"\r\nGO command entered ========\r\n");
	if ( PWMD4.state != PWM_STOP ) {
		chprintf(chp,"  >>> ERROR: System is already running.\r\n");
		chprintf(chp,"      Issue 'stop' command first.\r\n");
	}
	else {
		listening_active = FALSE;
		command_center(chp);
		chprintf(chp,"\r\n  System Ready.\r\n\r\n");
		waitOK=10;
		while ( !chQSpaceI(&SDU1.iqueue) && --waitOK ) {	/* wait keypress */
			chprintf(chp,"  >>> Press ENTER to begin <<<\r\n");
			chThdSleepMilliseconds(1000);
		}
		if (!waitOK) {
			chprintf(chp,"\r\n  >>> Timeout period elapsed. Disabling motion systems.\r\n");
			command_stop(chp);
		}
		else {
			chprintf(chp,"\r\n  User confirmation received.\r\n\r\n");
			command_arm ( chp, TRUE);
			listening_active = TRUE;
			chprintf(chp,"  Serial stream listening started.\r\n");
		}
	}
	chprintf(chp,"Command complete ==========\r\n");	
}

/*	cmd_stop
 *	Immediately stops all motion and disables all systems.
 *	@params: chp		stream for status messages.
*/	
static void cmd_stop(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: stop\r\n");
    return;
  }

	chprintf(chp,"\r\nSTOP command entered ========\r\n");	
	command_stop( chp );
	chprintf(chp,"Command complete ==========\r\n");	
}

/*	cmd_center
 *	Slowly sends all axxis to home positions.
 *	@params: chp		stream for status messages.
*/	
static void cmd_center(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: center\r\n");
    return;
  }

	chprintf(chp,"\r\nCENTER command entered ========\r\n");	
	if ( PWMD4.state != PWM_STOP ) {
		chprintf(chp,"  >>> ERROR: System is already running.\r\n");
		chprintf(chp,"      Issue 'stop' command first.\r\n");
	}
	else {	
		command_center( chp );
		command_arm (chp,FALSE);
		chprintf(chp,"Command complete ==========\r\n");	
	}
}

/*	cmd_about
 *	Show firmware information.
 *	@params: chp		stream for status messages.
*/	
static void cmd_about(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: about\r\n");
    return;
  }

	chprintf(chp,"\r\nABOUT command entered ========\r\n\r\n");	
	chprintf(chp,"  Firmware version: v%d.%d.%03d\r\n",VERSION_MAJOR,VERSION_MINOR,VERSION_SUB);	
	chprintf(chp,"  (%s)\r\n\r\n",VERSION_ABOUT);

	chprintf(chp,"Command complete ==========\r\n");	
}


static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"threads", cmd_threads},
	{"params", cmd_parameters},
	{"calibrate", cmd_calibrate},
	{"go", cmd_go},
  {"stop", cmd_stop},
	{"center", cmd_center},
	{"about", cmd_about},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};


	/* Application entry point (main)
	 *
	 */
int main(void) {

  Thread *shelltp = NULL;
		
  /* System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

	/* Activates the serial driver 2.
 	 * PA2(TX) and PA3(RX) are routed to USART2.
	 */
	sdStart(&SD2, NULL);          /* Default set to 115200-8-N-1 in conf files. */
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

	calibrate_load_defaults ( &calibration );	/* load default calibrations for machine */
	
	/* Start GPT 2 heartbeat timer
	 */
  gptStart(&GPTD2, &gpt2cfg);
  gptPolledDelay(&GPTD2, 10); /* Small delay.*/

  /* Initializes the ADC driver 1.
	* Potentiometer inputs: pins PC4 (axis1) and PC5 (axis2) on the port GPIOC is programmed as analog inputs.
	* Current Sensor inputs: pins PA6 (axis1) and PA7 (axis2) on port GPIOA as analog inputs.
  */
  adcStart(&ADCD1, NULL);
  palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);	
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);	
	palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_ANALOG);	

	/* PWM output mode set for each axis' pins (of TIM4) on...
	 * BLUE LED (PD.15, axis0) and ORANGE LED (PD.13, axis1).
	 * Channels are still disabled, though.
	 */
	palSetPadMode(GPIOD, GPIOD_LED6, PAL_MODE_ALTERNATE(2));
	palSetPadMode(GPIOD, GPIOD_LED3, PAL_MODE_ALTERNATE(2));
		
	/* PB.13 and PB.14 configured for direction logic output (axis0). */
	palSetPadMode(GPIOB, 13, PAL_MODE_OUTPUT_OPENDRAIN);	/* pulled to 5V through 10K resistor */
	/* PE.13 and PE.14 configured for direction logic output (axis1). */
	palSetPadMode(GPIOE, 13, PAL_MODE_OUTPUT_OPENDRAIN);	/* pulled to 5V through 10K resistor */

  /* Create display output and serial stream listener threads */
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
	chThdSleepMilliseconds(1000);
	chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);
		
	gptStartContinuous(&GPTD2, 10000);	/* start heartbeat green flash 1 Hz */

  /*
   * Shell manager initialization.
   */
  shellInit();

	/*
   * Initializes a serial-over-USB CDC driver.
   */
	sduObjectInit(&SDU1);
	sduStart(&SDU1, &serusbcfg);
	
	 /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
	usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

	
	chThdSleepMilliseconds(5000);

	
	/*
     * Normal main() thread activity, in this case it just performs
     * a shell respawn upon its termination.
     */
  while (TRUE) {
   if (!shelltp) {
     if (SDU1.config->usbp->state == USB_ACTIVE) {
       /* Spawns a new shell.*/
       shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
     }
   }
   else {
      /* If the previous shell exited.*/
     if (chThdTerminated(shelltp)) {
     /* Recovers memory of the previous shell.*/
       chThdRelease(shelltp);
      shelltp = NULL;
     }
   }
    chThdSleepMilliseconds(1000);
  }
}
