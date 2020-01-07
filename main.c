// In 1996, the ANSI and the NFPA recommended a standard evacuation pattern to eliminate confusion. The pattern is uniform without regard to the sound used. 
// This pattern, which is also used for smoke alarms, is named the Temporal-Three alarm signal, often referred to as "T-3" (ISO 8201 and ANSI/ASA S3.41 
// Temporal Pattern) and produces an interrupted four count (three half second pulses, followed by a one and one half second pause, repeated for a minimum of 180 seconds)

#undef ARM_MATH_CM7
 
#include <stdio.h>
#include <stdlib.h> 
#include "nrf_drv_i2s.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"

#include <ble.h>
#include <ble_gap.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ARM_MATH_CM4
#include "arm_math.h"

#include "ping_config.h"
#include "timer.h"

/************************************************************
*   Include support for SGTL5000. 
************************************************************/

#include "drv_sgtl5000.h"


// Each I2S access/interrupt provides AUDIO_FRAME_NUM_SAMPLES of 32-bit stereo pairs
// And, m_i2s_rx_buffer holds two input buffers for double buffering, so twice the size or I2S_BUFFER_SIZE_WORDS long, where "words" are 32-bit pairs
// So, each vector is I2S_BUFFER_SIZE_WORDS * sizeof(uint32_t) bytes long
// And each buffer contains AUDIO_FRAME_NUM_SAMPLES 32-bit pairs


uint32_t  m_i2s_tx_buffer[I2S_BUFFER_SIZE_WORDS];
uint32_t  m_i2s_rx_buffer[I2S_BUFFER_SIZE_WORDS];

int16_t Rx_Buffer[FFT_SAMPLE_SIZE * 2];	// twice the number of stereo pairs

static uint32_t sample_idx          = 0;

#define SAMPLE_LEN                  67200
static uint8_t * p_sample ;



static bool i2s_sgtl5000_driver_evt_handler(drv_sgtl5000_evt_t * p_evt)
{
    bool ret = false;
    //NRF_LOG_INFO("i2s_sgtl5000_driver_evt_handler %d", p_evt->evt);

    switch (p_evt->evt)
    {
        case DRV_SGTL5000_EVT_I2S_RX_BUF_RECEIVED:
            {
                //NRF_LOG_INFO("i2s_sgtl5000_driver_evt_handler RX BUF RECEIVED");
                // TODO: Handle RX as desired - for this example, we dont use RX for anything
                //uint16_t * p_buffer  = (uint16_t *) p_evt->param.rx_buf_received.p_data_received;
            }
            break;
        case DRV_SGTL5000_EVT_I2S_TX_BUF_REQ:
            {
                //NRF_LOG_INFO("i2s_sgtl5000_driver_evt_handler TX BUF REQ");
                
                /* Play sample! 16kHz sample played on 32kHz frequency! If frequency is changed, this approach needs to change. */
                /* Playback of this 16kHz sample depends on I2S MCK, RATIO, Alignment, format, and channels! Needs to be DIV8, RATIO 128X, alignment LEFT, format I2S, channels LEFT. */
                
                uint16_t * p_buffer  = (uint16_t *) p_evt->param.tx_buf_req.p_data_to_send;
                uint32_t i2s_buffer_size_words = p_evt->param.rx_buf_received.number_of_words;
                int16_t pcm_stream[i2s_buffer_size_words];  // int16_t - i2s_buffer_size_words size; means we only cover half of data_to_send_buffer, which is fine since we are only using LEFT channel
                
                /* Clear pcm buffer */
                memset(pcm_stream, 0, sizeof(pcm_stream));

                /* Check if playing the next part of the sample will exceed the sample size, if not, copy over part of sample to be played */
                if (sample_idx < SAMPLE_LEN)
                {
                    /* Copy sample bytes into pcm_stream (or remaining part of sample). This should fill up half the actual I2S transmit buffer. */
                    /* We only want half becuase the sample is a 16kHz sample, and we are running the SGTL500 at 32kHz; see DRV_SGTL5000_FS_31250HZ */
                    uint32_t bytes_to_copy = ((sample_idx + sizeof(pcm_stream)) < SAMPLE_LEN) ? sizeof(pcm_stream) : SAMPLE_LEN - sample_idx - 1;
                    memcpy(pcm_stream, &p_sample[sample_idx], bytes_to_copy);
                    sample_idx += bytes_to_copy;
                    ret = true;
                }
                else 
                {
                    /* End of buffer reached. */
                    sample_idx = 0;
                    ret = false;
                }
                
                /* Upsample the decompressed audio */
                /* i < i2s_buffer_size_words * 2 because we have a uint16_t buffer pointer */
                for (int i = 0, pcm_stream_idx = 0; i < i2s_buffer_size_words * 2; i += 2)
                {
                    for (int j = i; j < (i + 2); ++j)
                    {
                        p_buffer[j] = pcm_stream[pcm_stream_idx];
                    }
                    ++pcm_stream_idx;
                }
            }
            break;
    }

    return ret;
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    nrf_gpio_pin_clear(LED_1);
    nrf_gpio_pin_clear(LED_2);
    nrf_gpio_pin_clear(LED_3);
    app_error_save_and_stop(id, pc, info);
}


void gpio_init(void)
{
    nrf_gpio_cfg_output(LED_1);
    nrf_gpio_cfg_output(LED_2);
    nrf_gpio_cfg_output(LED_3);
}

bool bEraseBonds = true;
bool connectedToBondedDevice = false;

uint16_t currentConnectionInterval = 0;

volatile bool bPingConnected = false;
volatile bool bSendParameters = false;


int main(void)
{
	uint32_t err_code = NRF_SUCCESS;
	uint16_t nIdx, nJdx;
	uint32_t nSeed = 0;


	gpio_init();

	DoBLE();

	// Get MAC Address

	GetMacAddress();

	// Because the NRF_LOG macros only support up to five varargs after the format string, we need to break this into two calls.
	NRF_LOG_RAW_INFO("\r\n---------------------------------------------------------------------\r\n");
	NRF_LOG_RAW_INFO("\r\nMAC Address = %02X:%02X:%02X:", MAC_Address.addr[5], MAC_Address.addr[4], MAC_Address.addr[3]);
	NRF_LOG_RAW_INFO("%02X:%02X:%02X\r\n", MAC_Address.addr[2], MAC_Address.addr[1], MAC_Address.addr[0]);

	// Generate a random seed

	for(nIdx=0; nIdx < 6 ; nIdx++)
	{
		nSeed += ((uint32_t) MAC_Address.addr[nIdx] ) << nIdx;	
	}

	srand(nSeed);

	NRF_LOG_RAW_INFO("Random seed is %d-\r\n", nSeed);

	Timer1_Init(TIMER1_REPEAT_RATE);

	err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_DEFAULT_BACKENDS_INIT();

#ifdef TESTLEDS
	for(nJdx=LED_1; nJdx <= LED_4; nJdx++)
	for(nIdx=0; nIdx < 3 ; nIdx++)
	{
		nrf_gpio_pin_set(nJdx);
		nrf_delay_ms(500);
		nrf_gpio_pin_clear(nJdx);
		nrf_delay_ms(500);
	}
#endif

	// Turn Off LEDs
	for(nJdx=LED_1; nJdx <= LED_4; nJdx++)  
	{
		NRF_LOG_RAW_INFO("LED %d OFF\r\n", nJdx);
         	 nrf_gpio_pin_set(nJdx);
	}

	NRF_LOG_RAW_INFO("Init started\r\n");

	// Enable audio
	drv_sgtl5000_init_t sgtl_drv_params;
	sgtl_drv_params.i2s_tx_buffer           = (void*)m_i2s_tx_buffer;
	sgtl_drv_params.i2s_rx_buffer           = (void*)m_i2s_rx_buffer;
	sgtl_drv_params.i2s_buffer_size_words   =  I2S_BUFFER_SIZE_WORDS; ; //I2S_BUFFER_SIZE_WORDS/2;
	sgtl_drv_params.i2s_evt_handler         = i2s_sgtl5000_driver_evt_handler;
	sgtl_drv_params.fs                      = DRV_SGTL5000_FS_31250HZ;

#ifdef DOING_TX
	m_i2s_tx_buffer[0] = 167;
	m_i2s_tx_buffer[I2S_BUFFER_SIZE_WORDS/2] = 167;
#endif

	NRF_LOG_RAW_INFO("AUDIO_FRAME_NUM_SAMPLES = %d\r\n", AUDIO_FRAME_NUM_SAMPLES);
	NRF_LOG_RAW_INFO("size of  m_i2s_rx_buffer %d =  %d 32-bit samples\r\n", sizeof(m_i2s_rx_buffer) / sizeof(uint32_t), I2S_BUFFER_SIZE_WORDS);
	NRF_LOG_RAW_INFO("i2s_initial_Rx_buffer addr1: %d, addr2: %d\r\n", m_i2s_rx_buffer, m_i2s_rx_buffer + I2S_BUFFER_SIZE_WORDS/2);

	drv_sgtl5000_init(&sgtl_drv_params);
	drv_sgtl5000_stop();
	NRF_LOG_RAW_INFO("Audio initialization done.\r\n");

	/* Demonstrate Mic loopback */
	NRF_LOG_RAW_INFO("Loop in main and loopback MIC data.\r\n");
	drv_sgtl5000_start_mic_listen();

	

	for (;;)
	{
		uint32_t nIdx, nJdx, nKdx;
		static bool bBeenHere = false;
		float fBinSize;
		uint32_t Dominant_Index;

		if(ElapsedTimeInMilliseconds() > 1000)
		{
			// Signal that we want to capture
			bCaptureRx = true;

			//NRF_LOG_RAW_INFO("StartTime = %d\r\n", ElapsedTimeInMilliseconds());

			// Wait for capture
			while(bCaptureRx)   nrf_delay_ms(1);

			//NRF_LOG_RAW_INFO("Copied RX in %d msec\r\n", RxTimeDelta);

				
			//NRF_LOG_RAW_INFO("[%d] Num_Mic_Samples = %d, Mono FFT Sample Size = %d\n\r",ElapsedTimeInMilliseconds(), Num_Mic_Samples, FFT_SAMPLE_SIZE);
			fBinSize = ( 31250.0 /2 ) / (FFT_SAMPLE_SIZE /2 );

			//sprintf(cOutbuf, "fBinSize = %f\n\r", fBinSize); 	NRF_LOG_RAW_INFO("%s", (uint32_t) cOutbuf);

			// Convert stereo 16-bit samples to mono float samples, half as many
			nJdx = 0;
			nKdx = 0;

			for(nIdx=0; nIdx < FFT_SAMPLE_SIZE * 2; nIdx += 2)
			{
				fFFTin[nKdx] = (float)  Rx_Buffer[nIdx];
				nKdx++;

				//NRF_LOG_RAW_INFO("%8d\r\n",Rx_Buffer[nIdx]);
			}

			//NRF_LOG_RAW_INFO("Doing FFT\r\n");

			uint32_t BegTime, EndTime, DeltaTime;
			uint32_t nIterations = 10;

			BegTime = ElapsedTimeInMilliseconds();

			Dominant_Index = ping_fft(fBinSize);
			
			EndTime = ElapsedTimeInMilliseconds();
			
			DeltaTime = EndTime - BegTime;

			if((Dominant_Index >= 51) && (Dominant_Index <= 53))
			{
                            NRF_LOG_RAW_INFO("Dominant_Index = %d\r\n", Dominant_Index);
				nrf_gpio_pin_clear(LED_3);
			}
			else
			{
				nrf_gpio_pin_set(LED_3);
			}

                     //   NRF_LOG_RAW_INFO("BegTime = %d EndTime = %d\r\n", BegTime, EndTime);
			//NRF_LOG_RAW_INFO("For %d Iterations, took %d msec\r\n", nIterations, DeltaTime);


			bBeenHere = true;
		}

		nrf_delay_ms(100);
		NRF_LOG_FLUSH();
	}
	}



