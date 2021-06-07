/*
 * ff.c
 *
 *  Created on: 23 May 2021
 *      Author: rob
 */

#include "stm32f10x.h"

typedef int32_t fx_t;

#define FX_FRAC 16

#define fx_float(a) (a / (float)((int)1<<FX_FRAC))
#define fx_make(a)  ((fx_t)((a) * ((int)1<<FX_FRAC)))
#define fx_add(a,b) ((a) + (b))
#define fx_sub(a,b) ((a) - (b))
#define fx_mul(a,b) ((fx_t)(((int64_t)(a) * (b)) >> FX_FRAC))
#define fx_div(a,b) ((fx_t)(((int64_t)(a) << FX_FRAC) / (b)))
#define fx_fx2int(a) ((int)((a)>>FX_FRAC))

#define TSHIFT 7
#define TSIZE (1<<TSHIFT)

// convert HZ to skip value
#define HZ_TO_SKIP(F) (((F)*TSIZE*4)/1000)

extern uint16_t adc_buf[8];

// sin table, 16 fractional bits
// used to synthesise vibration
const uint16_t stab[] = {
		0x0000,0x0324,0x0648,0x096c,0x0c8f,0x0fb2,0x12d5,0x15f6,
		0x1917,0x1c37,0x1f56,0x2273,0x2590,0x28aa,0x2bc4,0x2edb,
		0x31f1,0x3505,0x3817,0x3b26,0x3e33,0x413e,0x4447,0x474d,
		0x4a50,0x4d50,0x504d,0x5347,0x563e,0x5931,0x5c22,0x5f0e,
		0x61f7,0x64dc,0x67bd,0x6a9b,0x6d74,0x7049,0x7319,0x75e5,
		0x78ad,0x7b70,0x7e2e,0x80e7,0x839c,0x864b,0x88f5,0x8b9a,
		0x8e39,0x90d3,0x9368,0x95f6,0x987f,0x9b02,0x9d7f,0x9ff6,
		0xa267,0xa4d2,0xa736,0xa994,0xabeb,0xae3b,0xb085,0xb2c8,
		0xb504,0xb73a,0xb968,0xbb8f,0xbdae,0xbfc7,0xc1d8,0xc3e1,
		0xc5e3,0xc7de,0xc9d1,0xcbbb,0xcd9e,0xcf7a,0xd14d,0xd318,
		0xd4db,0xd695,0xd848,0xd9f2,0xdb94,0xdd2d,0xdebd,0xe046,
		0xe1c5,0xe33c,0xe4aa,0xe60f,0xe76b,0xe8bf,0xea09,0xeb4b,
		0xec83,0xedb2,0xeed8,0xeff5,0xf109,0xf213,0xf314,0xf40b,
		0xf4fa,0xf5de,0xf6ba,0xf78b,0xf853,0xf912,0xf9c7,0xfa72,
		0xfb14,0xfbac,0xfc3b,0xfcbf,0xfd3a,0xfdab,0xfe13,0xfe70,
		0xfec4,0xff0e,0xff4e,0xff84,0xffb1,0xffd3,0xffec,0xfffb
};

fx_t fast_sin(int f_pos) {
	int p = f_pos&(TSIZE-1);
	int q = f_pos>>TSHIFT & 0x3;
	fx_t l = 0;
	switch(q) {
	case 0: l = stab[p]; break;
	case 1: l = stab[TSIZE-1-p]; break;
	case 2: l = -stab[p]; break;
	case 3: l = -stab[TSIZE-1-p]; break;
	}
	return(l);
}

/*
 * 1st order low pass, W at 100Hz
 * W = tan(2Pi*100 rad/s / (1kHz * 2))
 * A1 = (W - 1)/(W + 1)
 * B0 = W / (W + 1)
 */

const fx_t A1 = fx_make(0.93906);
const fx_t B0 = fx_make(0.03047);

fx_t axis_filt[5];

uint32_t f_pos;

int set_axis_torque(int axis, int torque) {
	if ((torque > 5)) torque = 25 + torque;
	if ((torque < -5)) torque = -25 + torque;
	if (torque > 250) torque = 250;
	if (torque < -250) torque = -250;

	if (axis == 1) {
		// elevator
		if (torque < 0) {
			TIM_SetCompare3(TIM4, -torque);
			TIM_SetCompare4(TIM4, 0);
			GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_SET);
			GPIO_WriteBit(GPIOB, GPIO_Pin_11, Bit_SET);
		} else {
			TIM_SetCompare4(TIM4, torque);
			TIM_SetCompare3(TIM4, 0);
			GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_SET);
			GPIO_WriteBit(GPIOB, GPIO_Pin_11, Bit_SET);
		}
	} else if (axis == 0) {
		// aileron
		if (torque < 0) {
			TIM_SetCompare2(TIM4, -torque);
			TIM_SetCompare1(TIM4, 0);
			GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_SET);
			GPIO_WriteBit(GPIOB, GPIO_Pin_11, Bit_SET);
		} else {
			TIM_SetCompare1(TIM4, torque);
			TIM_SetCompare2(TIM4, 0);
			GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_SET);
			GPIO_WriteBit(GPIOB, GPIO_Pin_11, Bit_SET);
		}
	}
	return 0;
}

// 1kHz force feedback control cycle
int ff_cycle(void) {
	int i;

	// Low pass filter analogue inputs
	// y[n] = y[n-1] * a1 + x[n] * b0
	for (i=0; i<5; i++) {
		// ADC range is 12-bit, unsigned
		// Subtracting 2048 centres it around zero
		axis_filt[i] = fx_add(fx_mul(axis_filt[i], A1), fx_mul(fx_make(((int32_t)adc_buf[i])-2048), B0));
	}

	/*
	 * TODO
	 * Currently torque is just set to oppose motion with a fixed spring constant.
	 * There is a single sin wave synthesiser for engine noise.
	 * The plan is for each axis to have the following configured by the driver on the host:
	 * - Centering force/offset
	 * - Damping
	 * - Inertia
	 * - 2x cos and 2x sin noise synthesisers (engine noise/vibration)
	 */

	// set motor aileron
	int torque = -fx_fx2int(axis_filt[0])*2;

	// currently there is just one sin wave synthesizer on one axis to test the principle
	// plan is to have 2x cos and 2x sin per axis
	int vibe = fx_fx2int(fx_mul(fast_sin(f_pos), fx_make(100)));

	// position in the (single) sine wave
	f_pos += 10;

	torque += vibe;

	set_axis_torque(0, torque);

	// set motor elevator
	int torquer = fx_fx2int(axis_filt[1])/5;

	set_axis_torque(1, torquer);

	return 0;
}
