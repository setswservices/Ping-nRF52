
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ARM_MATH_CM4

#include "arm_math.h"

extern float fFFTin[MONO_FFT_SAMPLE_SIZE];
extern uint16_t nNumFFTinputs = 0;
extern void sana_fft(void);
