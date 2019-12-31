#define TIMER1_REPEAT_RATE 		(1000) 	// 5 millisecond repeating timer in microseconds

// Number of stereo pairs per I2S access
#define AUDIO_FRAME_NUM_SAMPLES                   			256

// Size of the Rx buffer in terms of samples, or 32-bit stereo pairs
#define I2S_BUFFER_SIZE_WORDS               					AUDIO_FRAME_NUM_SAMPLES * 2   // Double buffered, with AUDIO_FRAME_NUM_SAMPLES

#define FFT_SAMPLE_SIZE 						(AUDIO_FRAME_NUM_SAMPLES)
#define COMPLEX_FFT_SAMPLE_SIZE				(FFT_SAMPLE_SIZE * 2)

extern void Timer1_Init(uint32_t repeat_rate);
extern uint32_t ElapsedTimeInMilliseconds(void);
extern void sana_fft(float fBinSize);


extern uint32_t Num_Mic_Samples;
extern float fFFTin[COMPLEX_FFT_SAMPLE_SIZE];
extern float fFFTout[COMPLEX_FFT_SAMPLE_SIZE+2];
extern char cOutbuf[128];

extern int16_t *Current_RX_Buffer;
extern bool bCaptureRx;
extern int16_t Rx_Buffer[FFT_SAMPLE_SIZE * 2];

extern uint32_t RxTimeBeg;
extern uint32_t RxTimeDelta;

extern uint32_t  m_i2s_tx_buffer[I2S_BUFFER_SIZE_WORDS];
extern uint32_t  m_i2s_rx_buffer[I2S_BUFFER_SIZE_WORDS];




