

#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nordic_common.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ARM_MATH_CM4

#include "arm_math.h"
#include "arm_const_structs.h"

#include "ping_config.h"

/* ----------------------------------------------------------------------
* Copyright (C) 2010-2012 ARM Limited. All rights reserved.
*
* $Date:         17. January 2013
* $Revision:     V1.4.0
*
* Project:       CMSIS DSP Library
* Title:             arm_fft_bin_example_f32.c
*
* Description:   Example code demonstrating calculation of Max energy bin of
*                frequency domain of input signal.
*
* Target Processor: Cortex-M4/Cortex-M3
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------- */



char cOutbuf[128];
float fFFTin[COMPLEX_FFT_SAMPLE_SIZE];

float fFFTout[COMPLEX_FFT_SAMPLE_SIZE+2];

float32_t maxvalue;

uint32_t maxindex;

arm_rfft_fast_instance_f32 S;

/* -------------------------------------------------------------------
* External Input and Output buffer Declarations for FFT Bin Example
* ------------------------------------------------------------------- */

/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */
uint32_t fftSize = COMPLEX_FFT_SAMPLE_SIZE;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
/* Reference index at which max energy of bin ocuurs */
uint32_t refIndex = 213, testIndex = 0;
/* ----------------------------------------------------------------------
* Max magnitude FFT Bin test
* ------------------------------------------------------------------- */
void sana_fft(float fBinSize)
{
	arm_status status;
	float32_t maxValue;
	float Sampling_Window;
	uint32_t nIdx, nJdx;

	status = ARM_MATH_SUCCESS;
	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_f32(&arm_cfft_sR_f32_len256, fFFTin, ifftFlag, doBitReverse);
	/* Process the data through the Complex Magnitude Module for
	calculating the magnitude at each bin */
	arm_cmplx_mag_f32(fFFTin, fFFTout, fftSize);
	/* Calculates maxValue and returns corresponding BIN value */
	arm_max_f32(fFFTout, fftSize, &maxValue, &testIndex);


	// arm_max_f32(fFFTout+1, FFT_SAMPLE_SIZE/2, &maxvalue, &maxindex);

	testIndex++;

	Sampling_Window = 1000.0 * (float) FFT_SAMPLE_SIZE / 31250.0;

	sprintf(cOutbuf,"Sampling Window = %f\tMax Value:[%ld]:%f Output=\r\n",Sampling_Window, ( uint32_t)(fBinSize * testIndex) ,maxvalue);
	NRF_LOG_RAW_INFO("%s", (uint32_t) cOutbuf);

	nJdx = 0;
	for(nIdx=0; nIdx<FFT_SAMPLE_SIZE; nIdx += 2)
	{
		sprintf(cOutbuf, "[%6d]", ( uint32_t)(fBinSize * nJdx++));
		NRF_LOG_RAW_INFO("%s", (uint32_t) cOutbuf);

		sprintf(cOutbuf, "  %f", fFFTout[nIdx]);
		NRF_LOG_RAW_INFO("%s", (uint32_t) cOutbuf);

		sprintf(cOutbuf, "  %f \r\n", fFFTout[nIdx+1]);
		NRF_LOG_RAW_INFO("%s", (uint32_t) cOutbuf);

	}


}



