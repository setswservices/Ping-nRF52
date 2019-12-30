#define TIMER1_REPEAT_RATE 		(1000) 	// 5 millisecond repeating timer in microseconds

#define AUDIO_FRAME_WORDS                   	256
#define I2S_BUFFER_SIZE_WORDS               	AUDIO_FRAME_WORDS * 2   // Double buffered - I2S lib will switch between using first and second half

#define FFT_SAMPLE_SIZE 				(AUDIO_FRAME_WORDS * 2)

extern void Timer1_Init(uint32_t repeat_rate);
extern uint32_t ElapsedTimeInMilliseconds(void);
extern void sana_fft(float fBinSize);


extern uint32_t Num_Mic_Samples;
extern float32_t fFFTin[FFT_SAMPLE_SIZE * 2];
extern char cOutbuf[128];

extern int16_t *Current_RX_Buffer;




