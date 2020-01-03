
/////////////////////////////////////////////////////////////////////////////////////////////
//
//	File Name:		ping_ble.c
//	Author(s):		Jeffery Bahr, Dmitriy Antonets
//	Copyright Notice:	Copyright, 2019, Ping, LLC
//
//	Purpose/Functionality:	Routines for BLE initialization and use
//
/////////////////////////////////////////////////////////////////////////////////////////////


#include "app_config.h"
 
#include <stdio.h>
#include <stdlib.h> 
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"

// Definitions for NRF Logging prototypes, macros and declarations

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include <nrf_error.h>

#include "ping_config.h"

// Definitions for BLE

#include <ble.h>
#include "ble_ping.h"
#include "ping_ble.h"
#include <ble_advdata.h>
#include <ble_advertising.h>
#include <ble_bas.h>
#include <ble_err.h>
#include <ble_conn_params.h>
#include <nrf_ble_gatt.h>
#include <nrf_sdh.h>
#include <nrf_sdh_soc.h>
#include <nrf_sdh_ble.h>
#include <peer_manager.h>
#include <ble_gap.h>
#include <ble_conn_state.h>
#include <ble_nus.h>
#include <ble_dis.h>
#include <nrf_sdm.h>

#include <nrf_sdh.h>
#include <nrf_sdh_soc.h>
#include <nrf_sdh_ble.h>

#include <fds.h>

#include "timer.h"


/////////////////////////////////////////////////////////////////////////////////////////////
//  Defines                                                                                                                                               //
/////////////////////////////////////////////////////////////////////////////////////////////

#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */
#define PING_SERVICE_UUID_TYPE NUS_SERVICE_UUID_TYPE	 /**< UUID type for the Nordic UART Service (vendor specific). */

#define VCFW_PACKET_SIZE 16											   // 16 bytes of data plus checksum
#define APP_FEATURE_NOT_SUPPORTED BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2 /**< Reply when unsupported features are requested. */

#ifndef NOTIMEOUT_THANKS
#define APP_ADV_TIMEOUT_IN_SECONDS 600 // 180                                     /**< The advertising time-out in units of seconds. */
#else
#define APP_ADV_TIMEOUT_IN_SECONDS 0
#endif

#define INVALID_SESSION_ID			-1

#define ENABLE_BLE_SERVICE_DEBUG				1





/////////////////////////////////////////////////////////////////////////////////////////////
//  Variable and Data Structure Declarations                                                                                                                                           //
/////////////////////////////////////////////////////////////////////////////////////////////

BLE_PING_DEF(m_ping);				/**< BLE Ping service instance. */
NRF_BLE_GATT_DEF(m_gatt);			/**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */


#ifdef FREERTOS_ENABLED
xSemaphoreHandle SendDataMutex = NULL;
#endif // FREERTOS_ENABLED

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;			   /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static uint16_t ping_ble_msg_len = 0;

static ble_uuid_t m_adv_uuids[] = /**< Universally unique service identifier. */
	{
		{BLE_UUID_PING_SERVICE, PING_SERVICE_UUID_TYPE}
	};

#define SECURITY_REQUEST_DELAY 400 /**< Delay after connection until Security Request is sent, if necessary (ms). */
pm_peer_id_t m_peer_to_be_deleted = PM_PEER_ID_INVALID;
static void sec_req_timeout_handler(void);

ble_gap_addr_t MAC_Address;

int BLE_Delay = 0;
uint8_t BLE_buffer[21];

uint16_t CheckSumVCFW;

bool bEraseBonds = true;
bool connectedToBondedDevice = false;

volatile bool bBleConnected = false;

 uint16_t currentConnectionInterval = 0;

volatile bool bSanaConnected = false;

volatile bool bSendParameters = false;



/////////////////////////////////////////////////////////////////////////////////////////////
//  Code Begins Here  
//
//  Note that most of the function names here are those used in the Nordic SDK examples.  We have 
//  maintained them to allow comparison with existing examples.
//
//  Those functions that are not Sana-specific have comments imported from the SDK examples.
//
/////////////////////////////////////////////////////////////////////////////////////////////


void GetMacAddress(void)
{

	(void) sd_ble_gap_addr_get(&MAC_Address);
}

//////////////////////////////////////////////////////////////////////////////
//
// The ble_log_mac_address() function retrieves the MAC address and forms a mask name.
//
// Parameter(s):
//
//	devname		string representing  mask name
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

static void ble_log_mac_address(uint8_t *devname)
{
	// Log our BLE address (6 bytes).
	static uint32_t err_code;
	// Get BLE address.

	err_code = sd_ble_gap_addr_get(&MAC_Address);


	APP_ERROR_CHECK(err_code);

	// Because the NRF_LOG macros only support up to five varargs after the format string, we need to break this into two calls.
	NRF_LOG_RAW_INFO("\r\n---------------------------------------------------------------------\r\n");
	NRF_LOG_RAW_INFO("\r\nMAC Address = %02X:%02X:%02X:", MAC_Address.addr[5], MAC_Address.addr[4], MAC_Address.addr[3]);
	NRF_LOG_RAW_INFO("%02X:%02X:%02X\r\n", MAC_Address.addr[2], MAC_Address.addr[1], MAC_Address.addr[0]);

	if (devname != NULL)
		sprintf((char *)devname, "-%02X%02X%02X", MAC_Address.addr[2], MAC_Address.addr[1], MAC_Address.addr[0]);
}


//////////////////////////////////////////////////////////////////////////////
//
// The Ble_ping_send_data() function sends an arbitrary packet with Packet Type over BLE.   
//
// Parameter(s):
//
//	SanaPacketType		string representing  mask name
//	BLEpacket			pointer to packet buffer
//	BLEpacketLen			packet buffer length
//
// Returns SDK error number or NRF_SUCCESS
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

#define	MAX_BLE_RETRIES		20

int Ble_ping_send_data(uint8_t SanaPacketType, uint8_t *BLEpacket, uint8_t BLEpacketLen)
{
	uint32_t err_code;
	uint16_t nIdx, nJdx;

	
	if( BLEpacket == NULL)
	{
		return NRF_ERROR_INVALID_DATA;
	}

	if (m_ping.conn_handle == BLE_CONN_HANDLE_INVALID)
	{
		return NRF_ERROR_INVALID_STATE;
	}	

#ifdef FREERTOS_ENABLED
	if (xSemaphoreTake(SendDataMutex, (portMAX_DELAY - 10)) != pdPASS)
		APP_ERROR_CHECK(NRF_ERROR_MUTEX_LOCK_FAILED);
#endif // FREERTOS_ENABLED

	memset(BLE_buffer, 0, sizeof(BLE_buffer));

	if(BLEpacketLen > (sizeof(BLE_buffer) - 1))
	{
		BLEpacketLen = sizeof(BLE_buffer) - 1;
	}
	
	memcpy(&BLE_buffer[1], BLEpacket, BLEpacketLen);

	
#if ENABLE_BLE_SEND_DATA_DEBUG
	NRF_LOG_RAW_INFO("*** Ble_ping_send_data: SanaPacketType = %d, Len=%d, BLE_PING_MAX_DATA_LEN = %d\r\n", SanaPacketType, BLEpacketLen, BLE_PING_MAX_DATA_LEN);
#endif
	

	BLE_buffer[0] = SanaPacketType;

	if (m_ping.conn_handle == BLE_CONN_HANDLE_INVALID)
	{

#ifdef FREERTOS_ENABLED	
		if (xSemaphoreGive(SendDataMutex) != pdPASS)
			APP_ERROR_CHECK(NRF_ERROR_MUTEX_UNLOCK_FAILED);
#endif // FREERTOS_ENABLED

		return NRF_ERROR_INVALID_STATE;
	}

	// We could also check hvx_sent_count, which is getting tracked in ble_ping.c

	for (nJdx = 0; nJdx < 5; nJdx++)
	{
		if (hvx_sent_count > 0)
#ifdef FREERTOS_ENABLED
			vTaskDelay((TickType_t)(pdMS_TO_TICKS(20))); // wait 20 msec
#else
			nrf_delay_us(20);
#endif
		else
			break;
	}

	for (nIdx = 0; nIdx < MAX_BLE_RETRIES; nIdx++)
	{

#if ENABLE_BLE_SEND_DATA_DEBUG
		NRF_LOG_RAW_INFO("[%8d]Ble_ping_send_data:  PT=%02x, Len=%d, \r\n", global_msec_counter, SanaPacketType, BLEpacketLen);
#endif

		ping_ble_msg_len = (BLEpacketLen + 1);
		err_code = ble_ping_string_send(&m_ping, (uint8_t *)BLE_buffer, &ping_ble_msg_len);

		if (err_code == NRF_ERROR_RESOURCES)
		{
			if (nIdx > 3)
				NRF_LOG_RAW_INFO("[%8d, %d]Ble_ping_send_data:  Insufficient resources, hvx_sent_count=%d, nIdx=%d\r\n", 
					global_msec_counter, nIdx, hvx_sent_count, nIdx);



#ifdef FREERTOS_ENABLED
			vTaskDelay((TickType_t)(pdMS_TO_TICKS(25 * nIdx)));
#else
			nrf_delay_us(25 * nIdx);
#endif
			; // wait 100 msec
		}
		else
			break;
	}

//
// We're disabling this as of Rev1.1.62 as it seems like missing a packet 
// should not stop the therapy session
//

#ifdef DISABLING_16APR2019
	if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) // We'l just miss this data, but don't stop mask
	)
	{
		APP_ERROR_CHECK(err_code);
	}
#endif

#ifdef FREERTOS_ENABLED
	if (xSemaphoreGive(SendDataMutex) != pdPASS)
		APP_ERROR_CHECK(NRF_ERROR_MUTEX_UNLOCK_FAILED);
#endif // FREERTOS_ENABLED

	return err_code;
}

//////////////////////////////////////////////////////////////////////////////
//
// The delete_bonds() function clears bond information from persistent storage.
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

static void delete_bonds(void)
{
	ret_code_t err_code;

	err_code = pm_peers_delete();
	APP_ERROR_CHECK(err_code);
}

//////////////////////////////////////////////////////////////////////////////
//
// The advertising_start() function starts advertising over BLE.   
//
// Parameter(s):
//
//	p_erase_bonds		pointer to boolean bond state
//
// May terminate in APP_ERROR_CHECK
//
// This function cannot be capitalized
//
//////////////////////////////////////////////////////////////////////////////

void advertising_start(void *p_erase_bonds)
{
	bool erase_bonds = *(bool *)p_erase_bonds;

	if (erase_bonds)
	{
		delete_bonds();
		// Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
	}
	else
	{
#if ENABLE_SECURE_BLE_DEBUG
		ret_code_t err_code;

		err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
#else
		(void)ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
#endif

#if ENABLE_SECURE_BLE_DEBUG
		NRF_LOG_RAW_INFO("ble_advertising_start returns %d\r\n", err_code);
		APP_ERROR_CHECK(err_code);
		NRF_LOG_RAW_INFO("advertising_start exiting\r\n");
#endif
	}
}

void StopAdvertisingAndDisconnect(void)
{

	// Stop advertising
	
	sd_ble_gap_adv_stop();

	nrf_delay_ms(100);

	// Disconnect from any central
	if(m_conn_handle) 
		sd_ble_gap_disconnect(m_conn_handle,  BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}

 //////////////////////////////////////////////////////////////////////////////
//
// The pm_evt_handler() function for handling Peer Manager events.
//
// Parameter(s):
//
//	p_evt  		Peer Manager event
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

static void pm_evt_handler(pm_evt_t const *p_evt)
{

	ret_code_t err_code;

#if ENABLE_SECURE_BLE_DEBUG
	NRF_LOG_RAW_INFO("%s :  p_evt->evt_id = %d ..\r\n", (uint32_t) __func__, p_evt->evt_id);
#endif // ENABLE_SECURE_BLE_DEBUG

	switch (p_evt->evt_id)
	{

	case PM_EVT_CONN_SEC_PARAMS_REQ:
	{
		NRF_LOG_RAW_INFO("Unexpected PM_EVT_CONN_SEC_PARAMS_REQ\r\n");
	}
	break;

	case PM_EVT_BONDED_PEER_CONNECTED:
	{
		NRF_LOG_RAW_INFO("Connected to a previously bonded device.");
		connectedToBondedDevice = true;
	}
	break;

	case PM_EVT_CONN_SEC_SUCCEEDED:
	{
		if (bSecureBLE)
		{
			pm_conn_sec_status_t conn_sec_status;

			// Check if the link is authenticated (meaning at least MITM).
			err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
			APP_ERROR_CHECK(err_code);

			if (conn_sec_status.mitm_protected)
			{
				NRF_LOG_RAW_INFO("Link secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
								 ble_conn_state_role(p_evt->conn_handle),
								 p_evt->conn_handle,
								 p_evt->params.conn_sec_succeeded.procedure);
			}
			else
			{
				// The peer did not use MITM, disconnect.
				NRF_LOG_RAW_INFO("Collector did not use MITM, disconnecting\r\n");
				err_code = pm_peer_id_get(m_conn_handle, &m_peer_to_be_deleted);
				APP_ERROR_CHECK(err_code);
				err_code = sd_ble_gap_disconnect(m_conn_handle,
												 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
				APP_ERROR_CHECK(err_code);
			}
		}
		else
		{
			NRF_LOG_RAW_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
							 ble_conn_state_role(p_evt->conn_handle),
							 p_evt->conn_handle,
							 p_evt->params.conn_sec_succeeded.procedure);
		}
	}
	break;

	case PM_EVT_CONN_SEC_FAILED:
	{
		/* Often, when securing fails, it shouldn't be restarted, for security reasons.
			* Other times, it can be restarted directly.
			* Sometimes it can be restarted, but only after changing some Security Parameters.
			* Sometimes, it cannot be restarted until the link is disconnected and reconnected.
			* Sometimes it is impossible, to secure the link, or the peer device does not support it.
			* How to handle this error is highly application dependent. */

		if (bSecureBLE)
		{
			switch (p_evt->params.conn_sec_failed.error)
			{
			case PM_CONN_SEC_ERROR_PIN_OR_KEY_MISSING:
				// Rebond if one party has lost its keys.
				err_code = pm_conn_secure(p_evt->conn_handle, true);
				if (err_code != NRF_ERROR_INVALID_STATE)
				{
					APP_ERROR_CHECK(err_code);
				}
				break;

			default:
				break;
			}
		}
	}
	break;

	case PM_EVT_CONN_SEC_CONFIG_REQ:
	{
		// Reject pairing request from an already bonded peer.
		pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
		pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
	}
	break;

	case PM_EVT_STORAGE_FULL:
	{
		// Run garbage collection on the flash.
		err_code = fds_gc();
		if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
		{
			// Retry.
		}
		else
		{
			APP_ERROR_CHECK(err_code);
		}
	}
	break;

	case PM_EVT_PEERS_DELETE_SUCCEEDED:
	{
		bool bDeleteBonds = false;
		advertising_start(&bDeleteBonds);
	}
	break;

	case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
	{
		// The local database has likely changed, send service changed indications.
		pm_local_database_has_changed();
	}
	break;

	case PM_EVT_PEER_DATA_UPDATE_FAILED:
	{
		// Assert.
		APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
	}
	break;

	case PM_EVT_PEER_DELETE_FAILED:
	{
		// Assert.
		APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
	}
	break;

	case PM_EVT_PEERS_DELETE_FAILED:
	{
		// Assert.
		APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
	}
	break;

	case PM_EVT_ERROR_UNEXPECTED:
	{
		// Assert.
		APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
	}
	break;

	case PM_EVT_CONN_SEC_START:
	case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
	case PM_EVT_PEER_DELETE_SUCCEEDED:
	case PM_EVT_LOCAL_DB_CACHE_APPLIED:
	case PM_EVT_SERVICE_CHANGED_IND_SENT:
	case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
	default:
		break;
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// The SimpleByteCheckSum() function computes a simple checksum used in VCFW update 
//
// Parameter(s):
//
//	p_data			buffer to compute checksum on
//	Len				length of buffer
//
// Returns checksum
//
//////////////////////////////////////////////////////////////////////////////

uint8_t SimpleByteCheckSum(uint8_t *p_data, uint8_t Len)
{
	uint8_t cs = 0, idx;
	for (idx = 0; idx < Len; idx++)
		cs ^= p_data[idx];

	return cs;
}


//////////////////////////////////////////////////////////////////////////////
//
// The SimpleWordCheckSum() function computes a simple checksum used in VCFW update 
//
// Parameter(s):
//
//	OldCheckSum			initial checksum
//	p_data				buffer to compute checksum on
//	Len					length of buffer
//
// Returns checksum
//
//////////////////////////////////////////////////////////////////////////////

uint16_t SimpleWordCheckSum(uint16_t OldCheckSum, uint8_t *p_data, uint8_t Len)
{
	uint16_t NewCheckSum, idx;
	uint16_t word;

	NewCheckSum = OldCheckSum;

	for (idx = 0; idx < Len; idx++)
	{
		word = (uint16_t)p_data[idx];
		NewCheckSum ^= word;
	}

	return NewCheckSum;
}

//////////////////////////////////////////////////////////////////////////////
//
// The OldModeAnnouncement() function prints the old mode message  
//
// Parameter(s):
//
//	mode	    old mode value
//
//////////////////////////////////////////////////////////////////////////////

void OldModeAnnouncement(uint16_t mode)
{
	NRF_LOG_RAW_INFO("Old Mode was ");
	PrintCurrentMaskMode(mode);
}

//////////////////////////////////////////////////////////////////////////////
//
// The NewModeAnnouncement() function prints the new mode message   
//
// Parameter(s):
//
//	mode	    new mode value
//
//////////////////////////////////////////////////////////////////////////////

void NewModeAnnouncement(uint16_t mode)
{
	NRF_LOG_RAW_INFO("New Mode will be ");
	PrintCurrentMaskMode(mode);
}

//////////////////////////////////////////////////////////////////////////////
//
// The ble_ping_data_handler() function handles the data arriving over BLE on the Sana UART
// service.
//
// Parameter(s):
//
//	p_ping    PING Service structure.
//	p_data   	Data received.
// 	length   	Length of the data.
//
//////////////////////////////////////////////////////////////////////////////

static void ble_ping_data_handler(ble_ping_t *p_ping, uint8_t *p_data, uint16_t length)
{

	static uint16_t Eight = 8;
	
#if ENABLE_BLE_SERVICE_DEBUG

	{
		static uint16_t jdx;
		NRF_LOG_RAW_INFO("Got [");
		for (jdx = 0; jdx < length; jdx++)
		{
			NRF_LOG_RAW_INFO("%c", (const char)p_data[jdx]);
		}
		NRF_LOG_RAW_INFO("]\r\n");
	}

#endif // ENABLE_BLE_SERVICE_DEBUG

	if (strncmp((char *)p_data, "SendData", Eight) == 0)
	{
		if(!bEnableAppControl)
		{
			NRF_LOG_RAW_INFO("** Send Data not permitted for Non-App controlled images ***\r\n");
		}
		else
		{
			NRF_LOG_RAW_INFO("** Sending data to flash over BLE ***\r\n");

			int cmdLength = Eight;

			// check the rest of received command for an optional parameters
			if (length >= cmdLength + 2)
			{
				int idx = 0;
				int paramIndex = -1;
				bool skip = false;
				int position = cmdLength;
				char *parameters = (char *)p_data;
				char parsedStr[16];
				int parsedParams[] = {INVALID_SESSION_ID, INVALID_SESSION_ID};

				memset(parsedStr, 0x00, 16 * sizeof(char));

				while (position < length && parameters[position] && paramIndex < 2)
				{
					NRF_LOG_RAW_INFO("position %d, char: [%c] - ", position, parameters[position]);

					if (parameters[position] == ' ')
					{

						if (strlen(parsedStr) && !skip)
						{
							parsedParams[paramIndex] = atoi(parsedStr);
						}

						paramIndex++;
						idx = 0;
						memset(parsedStr, 0x00, 16 * sizeof(char));
						skip = false;
						NRF_LOG_RAW_INFO("starting param %d\n\r", paramIndex);
					}
					else if (parameters[position] >= '0' && parameters[position] <= '9' && !skip && paramIndex >= 0 && idx < 16)
					{
						NRF_LOG_RAW_INFO("continuing param %d, index %d, char [%c]\n\r", paramIndex, idx, parameters[position]);
						parsedStr[idx] = parameters[position];
						idx++;
					}
					else if (parameters[position] < '0' || parameters[position] > '9')
					{
						NRF_LOG_RAW_INFO("wrong char [%c], skipping\n\r", parameters[position]);
						skip = true;
					}

					position++;
				}

				if (strlen(parsedStr) && paramIndex < 2 && !skip)
				{
					parsedParams[paramIndex] = atoi(parsedStr);
				}

				if (parsedParams[0] != INVALID_SESSION_ID && parsedParams[1] != INVALID_SESSION_ID)
				{
					FirstSessionIdForXmit = parsedParams[0];
					LastSessionIdForXmit = parsedParams[1];

					if (FirstSessionIdForXmit > LastSessionIdForXmit)
					{
						NRF_LOG_RAW_INFO("*** Invalid session range from %d to %d ***\r\n", FirstSessionIdForXmit, LastSessionIdForXmit);
						return;
					}
					else
					{
						NRF_LOG_RAW_INFO("*** SendData Range is %d to %d ***\r\n", FirstSessionIdForXmit, LastSessionIdForXmit);
					}
				}

				NRF_LOG_RAW_INFO("*** for session from %d to %d ***\r\n", FirstSessionIdForXmit, LastSessionIdForXmit);
			}
			else
			{
				FirstSessionIdForXmit = INVALID_SESSION_ID;
				LastSessionIdForXmit = INVALID_SESSION_ID;
				NRF_LOG_RAW_INFO("*** for all sessions ***\r\n");
			}

			system_ops.led_ops = END;
			system_ops.mode = OP_INIT; // mode cannot be OP_OFF to send data 
			bSendingData = true;;
			bDataSessionIdSent = false;
			// Set flash read pointer to first session header

			NRF_LOG_RAW_INFO("***Setting  SCToken\r\n");
			bGotSCToken = true;

			//NRF_LOG_RAW_INFO("** Starting Session ***\r\n");
			//SetSessionState(SESSION_STARTED);
		}
	}

	
	if (strncmp((char *)p_data, "SendLog", 7) == 0)	// Ignore parameters
	{
	
		NRF_LOG_RAW_INFO("** Sending log from flash over BLE ***\r\n");

		int cmdLength = Eight;

		// check the rest of received command for an optional parameters
		if (length >= cmdLength + 2)
		{
			int idx = 0;
			int paramIndex = -1;
			bool skip = false;
			int position = cmdLength;
			char *parameters = (char *)p_data;
			char parsedStr[16];
			int parsedParams[] = {INVALID_SESSION_ID, INVALID_SESSION_ID};

			memset(parsedStr, 0x00, 16 * sizeof(char));

			while (position < length && parameters[position] && paramIndex < 2)
			{
				NRF_LOG_RAW_INFO("position %d, char: [%c] - ", position, parameters[position]);

				if (parameters[position] == ' ')
				{

					if (strlen(parsedStr) && !skip)
					{
						parsedParams[paramIndex] = atoi(parsedStr);
					}

					paramIndex++;
					idx = 0;
					memset(parsedStr, 0x00, 16 * sizeof(char));
					skip = false;
					NRF_LOG_RAW_INFO("starting param %d\n\r", paramIndex);
				}
				else if (parameters[position] >= '0' && parameters[position] <= '9' && !skip && paramIndex >= 0 && idx < 16)
				{
					NRF_LOG_RAW_INFO("continuing param %d, index %d, char [%c]\n\r", paramIndex, idx, parameters[position]);
					parsedStr[idx] = parameters[position];
					idx++;
				}
				else if (parameters[position] < '0' || parameters[position] > '9')
				{
					NRF_LOG_RAW_INFO("wrong char [%c], skipping\n\r", parameters[position]);
					skip = true;
				}

				position++;
			}

			if (strlen(parsedStr) && paramIndex < 2 && !skip)
			{
				parsedParams[paramIndex] = atoi(parsedStr);
			}

			if (parsedParams[0] != INVALID_SESSION_ID && parsedParams[1] != INVALID_SESSION_ID)
			{
				FirstSessionIdForXmit = parsedParams[0];
				LastSessionIdForXmit = parsedParams[1];

				if (FirstSessionIdForXmit > LastSessionIdForXmit)
				{
					NRF_LOG_RAW_INFO("*** Invalid session range from %d to %d ***\r\n", FirstSessionIdForXmit, LastSessionIdForXmit);
					return;
				}
				else
				{
					NRF_LOG_RAW_INFO("*** SendLog Range is %d to %d ***\r\n", FirstSessionIdForXmit, LastSessionIdForXmit);
				}
			}

			NRF_LOG_RAW_INFO("*** for session from %d to %d ***\r\n", FirstSessionIdForXmit, LastSessionIdForXmit);
		}
		else
		{
			FirstSessionIdForXmit = INVALID_SESSION_ID;
			LastSessionIdForXmit = INVALID_SESSION_ID;
			NRF_LOG_RAW_INFO("*** for all sessions ***\r\n");
		}

		system_ops.led_ops = END;
		bSendingLog = true;

		NRF_LOG_RAW_INFO("***Setting  SCToken\r\n");
		bGotSCToken = true;

	}
	

	if (bEnableAppControl)
	{
		//
		// Session Control
		//
		if (strncmp((char *)p_data, "StopSession", length) == 0)
		{
			NRF_LOG_RAW_INFO("** Stop Session ***\r\n");
			SetSessionState(SESSION_STOPPED);
		}
		else if (strncmp((char *)p_data, "StartSession", length) == 0)
		{
			NRF_LOG_RAW_INFO("** Start Session ***\r\n");
			SetSessionState(SESSION_STARTED);
		}
		else if (strncmp((char *)p_data, "PauseSession", length) == 0)
		{
			NRF_LOG_RAW_INFO("** Pause Session ***\r\n");
			SetSessionState(SESSION_PAUSED);
		}
		else if (strncmp((char *)p_data, "ResumeSession", length) == 0)
		{
			NRF_LOG_RAW_INFO("** Resume Session ***\r\n");
			SetSessionState(SESSION_STARTED);
		}
	}

	//
	// Older Commands
	//
#if ENABLE_FLASH
	if (strncmp((char *)p_data, "EraseData", length) == 0)
	{
		NRF_LOG_RAW_INFO("** EraseData: Resetting Flash Offset to Zero ***\r\n");
		bSendingData = false;
		bEraseSessionDataFlash = true;
	}

	if (strncmp((char *)p_data, "EraseLog", length) == 0)
	{
		NRF_LOG_RAW_INFO("** EraseLog: Resetting Flash Offset to Zero ***\r\n");
		bSendingLog = false;
		bEraseSessionLogFlash = true;
	}
		
#endif

	// Command from Sana Connect to permit sending HR/HRV data

	else if (strncmp((char *)p_data, "SCToken", length) == 0)
	{
		NRF_LOG_RAW_INFO("***Got SCToken\r\n");
		bGotSCToken = true;
	}

	// Commands from Sana Connect to change stored mode
	// But doesn't happen until "StoreMODE" command is received

	else if (strncmp((char *)p_data, "StoreMODE", length) == 0)
	{
		NRF_LOG_RAW_INFO("*** Setting: Store Mode to Flash and Reboot\r\n");
		OldModeAnnouncement(StoredMaskMode);
		bStoreModeToFlash = true;
		NewModeAnnouncement(StoredMaskMode);
	}

	//
	// Enable mode behavior change from flash or not
	//
	else if (strncmp((char *)p_data, "EnableMODE", length) == 0)
	{
		NRF_LOG_RAW_INFO("*** Setting: Enable Stored Mode\r\n");
		OldModeAnnouncement(StoredMaskMode);
		StoredMaskMode |= STORED_MODE_CHANGEABLE_MASK;
		NewModeAnnouncement(StoredMaskMode);
	}

	else if (strncmp((char *)p_data, "DisableMODE", length) == 0)
	{
		NRF_LOG_RAW_INFO("*** Setting: Disable Stored Mode\r\n");
		OldModeAnnouncement(StoredMaskMode);
		StoredMaskMode &= ~(STORED_MODE_CHANGEABLE_MASK);
		NewModeAnnouncement(StoredMaskMode);
	}

	//
	// Enable standard or secure BLE
	//
	else if (strncmp((char *)p_data, "SetSECBLEMODE", length) == 0)
	{
		NRF_LOG_RAW_INFO("*** Setting: Mask to Secure BLE Mode\r\n");
		OldModeAnnouncement(StoredMaskMode);
		StoredMaskMode |= STORED_STDBLE_OR_SECUREBLE_MASK;
		NewModeAnnouncement(StoredMaskMode);
	}

	else if (strncmp((char *)p_data, "SetSTDBLEMODE", length) == 0)
	{
		NRF_LOG_RAW_INFO("*** Setting: Mask to Standard BLE Mode\r\n");
		OldModeAnnouncement(StoredMaskMode);
		StoredMaskMode &= ~(STORED_STDBLE_OR_SECUREBLE_MASK);
		NewModeAnnouncement(StoredMaskMode);
	}

	//
	// Enable App or No-App Mode
	//
	else if (strncmp((char *)p_data, "SetAPPMODE", length) == 0)
	{
		NRF_LOG_RAW_INFO("*** Setting: Mask to App Mode\r\n");
		OldModeAnnouncement(StoredMaskMode);
		StoredMaskMode |= STORED_APP_OR_NOAPP_MASK;
		NewModeAnnouncement(StoredMaskMode);
	}

	else if (strncmp((char *)p_data, "SetNOAPPMODE", length) == 0)
	{
		NRF_LOG_RAW_INFO("*** Setting: Mask to No App Mode\r\n");
		OldModeAnnouncement(StoredMaskMode);
		StoredMaskMode &= ~(STORED_APP_OR_NOAPP_MASK);
		NewModeAnnouncement(StoredMaskMode);
	}

	//
	// Enable Real or Sham Mode
	//
	else if (strncmp((char *)p_data, "SetREALMODE", length) == 0)
	{
		NRF_LOG_RAW_INFO("*** Setting: Mask to Real Mode\r\n");
		OldModeAnnouncement(StoredMaskMode);
		StoredMaskMode |= STORED_REAL_OR_SHAM_MASK;
		NewModeAnnouncement(StoredMaskMode);
	}

	else if (strncmp((char *)p_data, "SetSHAMMODE", length) == 0)
	{
		NRF_LOG_RAW_INFO("*** Setting: Mask to Sham Mode\r\n");
		OldModeAnnouncement(StoredMaskMode);
		StoredMaskMode &= ~(STORED_REAL_OR_SHAM_MASK);
		NewModeAnnouncement(StoredMaskMode);
	}

	//
	// Reset Volume and Light and Reboot
	//

	else if (strncmp((char *)p_data, "ResetVL", length) == 0)
	{
		ResetVolumeAndLight();
	}
	else if (strncmp((char *)p_data, "SendBattery", length) == 0)
	{
		SendBatteryData();
	}
	else if (strncmp((char *)p_data, "SendParameters", length) == 0)
	{

		bSendParameters = true;
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// The ping_data_handler() function is the event handler for new data on the Sana UART 
// service.
//
// Parameter(s):
//
//	p_evt		pointer to event structure
//
//////////////////////////////////////////////////////////////////////////////

static void ping_data_handler(ble_ping_evt_t *p_evt)
{

	if (p_evt->type == BLE_PING_EVT_RX_DATA)
	{
		NRF_LOG_DEBUG("Received data from BLE PING \r\n");
		ble_ping_data_handler(p_evt->p_ping, (uint8_t *)p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// The gatt_evt_handler() function for handling events from the GATT library   
//
// Parameter(s):
//
//	p_gatt		pointer to GATT structure
//	p_evt		pointer to event structure
//
//////////////////////////////////////////////////////////////////////////////

void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{
	if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
	{
		m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
		NRF_LOG_RAW_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
	}
	NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
				  p_gatt->att_mtu_desired_central,
				  p_gatt->att_mtu_desired_periph);
}


//////////////////////////////////////////////////////////////////////////////
//
// The on_conn_params_evt() function for handling the Connection Parameters Module
//
// Parameter(s):
//
//	p_evt		pointer to event structure
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
	static ret_code_t err_code;

	if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
	{
		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		APP_ERROR_CHECK(err_code);
	}
}

#if ENABLE_ADVERTISING

//////////////////////////////////////////////////////////////////////////////
//
// The on_adv_evt() function will be called for advertising events which are passed to the 
// application. 
//
// Parameter(s):
//
//	ble_adv_evt  		Advertising event
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
	uint32_t err_code;

	switch (ble_adv_evt)
	{
	case BLE_ADV_EVT_FAST:
		err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_ADV_EVT_IDLE:
		NRF_LOG_RAW_INFO("%s(%d)\r\n", (uint32_t *)__func__, global_msec_counter);
		break;

	default:
		break;
	}
}
#endif // ENABLE_ADVERTISING

//////////////////////////////////////////////////////////////////////////////
//
// The ble_evt_handler() function  for handling BLE events.
//
// Parameter(s):
//
//	p_ble_evt   Bluetooth stack event
//	p_context   Unused, but necessary as it is in the prototype
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
	static uint32_t err_code;

#if ENABLE_SECURE_BLE_DEBUG
	NRF_LOG_RAW_INFO("%s :  p_ble_evt->header.evt_id = %d ..\r\n", (uint32_t) __func__, p_ble_evt->header.evt_id);
#endif // ENABLE_SECURE_BLE_DEBUG

	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_DISCONNECTED:
		NRF_LOG_RAW_INFO("Disconnected\r\n");
		m_conn_handle = BLE_CONN_HANDLE_INVALID;
		bSanaConnected = false;
		connectedToBondedDevice = false;

		if (bSecureBLE)
		{
			/*check if the last connected peer had not used MITM, if so, delete its bond information*/
			if (m_peer_to_be_deleted != PM_PEER_ID_INVALID)
			{
				ret_code_t ret_val = pm_peer_delete(m_peer_to_be_deleted);
				APP_ERROR_CHECK(ret_val);
				NRF_LOG_DEBUG("Collector's bond deleted\r\n");
				m_peer_to_be_deleted = PM_PEER_ID_INVALID;
			}
		}

		break;

	case BLE_GAP_EVT_CONNECTED:
		NRF_LOG_RAW_INFO("Connected\r\n");

		NRF_LOG_RAW_INFO("%s: MinCI=%d, MaxCI=%d\r\n", 
		(uint32_t) __func__,
		p_ble_evt->evt.gap_evt.params.connected.conn_params.min_conn_interval, 
		p_ble_evt->evt.gap_evt.params.connected.conn_params.max_conn_interval);

		if (bSecureBLE)
		{
			m_peer_to_be_deleted = PM_PEER_ID_INVALID;

			m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			// Start Security Request timer.

			if (!connectedToBondedDevice)
			{
				Set_Secure_Timeout_Handler(sec_req_timeout_handler, SECURITY_REQUEST_DELAY);
			}
		}

		bSanaConnected = true;
		
		break;
	case BLE_GAP_EVT_CONN_PARAM_UPDATE:
	{
#if ENABLE_SECURE_BLE_DEBUG
		NRF_LOG_RAW_INFO("********************** Connection params changed!\r\n");
		NRF_LOG_RAW_INFO("********************** Interval min: %d\r\n", 
		 	p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval);
#endif // ENABLE_SECURE_BLE_DEBUG

		currentConnectionInterval = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval;
	}
	break;
	case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
	{
		NRF_LOG_DEBUG("PHY update request.");
		ble_gap_phys_t const phys =
			{
				.rx_phys = BLE_GAP_PHY_AUTO,
				.tx_phys = BLE_GAP_PHY_AUTO,
			};
		err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
		APP_ERROR_CHECK(err_code);
	}
	break;

	case BLE_GATTC_EVT_TIMEOUT:

		// Disconnect on GATT Client timeout event.
		NRF_LOG_DEBUG("GATT Client Timeout.");
		err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
										 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTS_EVT_TIMEOUT:

		// Disconnect on GATT Server timeout event.
		NRF_LOG_DEBUG("GATT Server Timeout.");
		err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
										 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GAP_EVT_PASSKEY_DISPLAY:
	{
		if (bSecureBLE)
		{
			char passkey[PASSKEY_LENGTH + 1];
			memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, PASSKEY_LENGTH);
			passkey[PASSKEY_LENGTH] = 0;
			// Don't send delayed Security Request if security procedure is already in progress.

			Clean_Secure_Timeout_Handler();
			NRF_LOG_RAW_INFO("Passkey: %s\r\n", nrf_log_push(passkey));
		}
	}
	break; // BLE_GAP_EVT_PASSKEY_DISPLAY

	case BLE_EVT_USER_MEM_REQUEST:

		err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
		APP_ERROR_CHECK(err_code);
		break;

	case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
	{
		ble_gatts_evt_rw_authorize_request_t req;
		ble_gatts_rw_authorize_reply_params_t auth_reply;

		req = p_ble_evt->evt.gatts_evt.params.authorize_request;

		if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
		{
			if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ) ||
				(req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
				(req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
			{
				if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
				{
					auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
				}
				else
				{
					auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
				}
				auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
				err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
														   &auth_reply);
				APP_ERROR_CHECK(err_code);
			}
		}
	}
	break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

	default:
		// No implementation needed.
		break;
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// The ble_stack_init() function for initializing the BLE stack. 
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

static void ble_stack_init(void)
{
	static ret_code_t err_code;

	err_code = nrf_sdh_enable_request();

	APP_ERROR_CHECK(err_code);

	// Configure the BLE stack using the default settings.
	// Fetch the start address of the application RAM.

	uint32_t ram_start = 0;
	err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);

	APP_ERROR_CHECK(err_code);

	// Enable BLE stack.
	err_code = nrf_sdh_ble_enable(&ram_start);

	APP_ERROR_CHECK(err_code);

	// Register a handler for BLE events.
	NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


//////////////////////////////////////////////////////////////////////////////
//
// The gap_params_init() function sets up all the necessary GAP (Generic Access Profile) parameters 
// of the  device including the device name, appearance, and the preferred connection parameters.  
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

static void gap_params_init(void)
{
	static ret_code_t err_code;
	static ble_gap_conn_params_t gap_conn_params;
	static ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	strcpy((char *)BLE_dev_id, DEVICE_NAME);

	ble_log_mac_address(&BLE_dev_id[strlen(DEVICE_NAME)]);

	err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)BLE_dev_id, strlen(DEVICE_NAME) + 7);

	bGotSanaName = true;

	APP_ERROR_CHECK(err_code);

	err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

	NRF_LOG_RAW_INFO("%s: Setting MinCI=%d, MaxCI=%d\r\n", 
	(uint32_t) __func__,
	gap_conn_params.min_conn_interval, 
	gap_conn_params.max_conn_interval);

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}


//////////////////////////////////////////////////////////////////////////////
//
// The gatt_init() function is for initializing the GATT module.
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

static void gatt_init(void)
{

	ret_code_t err_code;

	err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, BLE_GATT_ATT_MTU_DEFAULT);
	APP_ERROR_CHECK(err_code);
}

//////////////////////////////////////////////////////////////////////////////
//
// The advertising_init() function is for initializing the Advertising functionality. 
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

static void advertising_init(void)
{
	static ret_code_t err_code;
	static ble_advertising_init_t init;

	memset(&init, 0, sizeof(init));

	init.advdata.name_type = BLE_ADVDATA_FULL_NAME;

	init.advdata.include_appearance = false;

	// This has to be general for a timeout of zero (APP_ADV_TIMEOUT_IN_SECONDS)

	init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	init.srdata.uuids_complete.p_uuids = m_adv_uuids;

	init.config.ble_adv_fast_enabled = true;
	init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
	init.config.ble_adv_fast_timeout = APP_ADV_TIMEOUT_IN_SECONDS;

	init.evt_handler = on_adv_evt;

	err_code = ble_advertising_init(&m_advertising, &init);

	APP_ERROR_CHECK(err_code);

	ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


//////////////////////////////////////////////////////////////////////////////
//
// The services_init() function is for initializing services that will be used by the application.
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

static void services_init(void)
{
	ret_code_t err_code;

	ble_ping_init_t ping_init;

	memset(&ping_init, 0, sizeof(ping_init));

	ping_init.data_handler = ping_data_handler;

	err_code = ble_ping_init(&m_ping, &ping_init);
	APP_ERROR_CHECK(err_code);
}

//////////////////////////////////////////////////////////////////////////////
//
// The conn_params_error_handler() function is handling a Connection Parameters error.
//
// Will terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

static void conn_params_error_handler(uint32_t nrf_error)
{
	APP_ERROR_CHECK(nrf_error);
}

//////////////////////////////////////////////////////////////////////////////
//
// The conn_params_init() function is  initializing the Connection Parameters module.
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

static void conn_params_init(void)
{
	ret_code_t err_code;
	ble_conn_params_init_t cp_init;

	memset(&cp_init, 0, sizeof(cp_init));

	cp_init.p_conn_params = NULL;
	cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
	cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;

	cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;

	cp_init.disconnect_on_fail = false;
	cp_init.evt_handler = on_conn_params_evt;
	cp_init.error_handler = conn_params_error_handler;

	err_code = ble_conn_params_init(&cp_init);
	APP_ERROR_CHECK(err_code);

}


//////////////////////////////////////////////////////////////////////////////
//
// The peer_manager_init() function is  for the Peer Manager initialization. 
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

static void peer_manager_init(bool erase_bonds)
{
	ble_gap_sec_params_t sec_param;
	ret_code_t err_code;

	err_code = pm_init();
	APP_ERROR_CHECK(err_code);

	if (erase_bonds)
	{
		err_code = pm_peers_delete();
		APP_ERROR_CHECK(err_code);
	}

	memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

	// Security parameters to be used for all security procedures.
	sec_param.bond = SEC_PARAM_BOND;

	if (bSecureBLE)
	{
		sec_param.mitm = SEC_PARAM_MITM_1;
	}
	else
	{
		sec_param.mitm = SEC_PARAM_MITM_0;
	}
	sec_param.lesc = SEC_PARAM_LESC;
	sec_param.keypress = SEC_PARAM_KEYPRESS;

	if (bSecureBLE)
	{
		sec_param.io_caps = BLE_GAP_IO_CAPS_DISPLAY_ONLY;
	}
	else
	{
		sec_param.io_caps = BLE_GAP_IO_CAPS_NONE;
	}

	sec_param.oob = SEC_PARAM_OOB;
	sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
	sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
	sec_param.kdist_own.enc = 1;
	sec_param.kdist_own.id = 1;
	sec_param.kdist_peer.enc = 1;
	sec_param.kdist_peer.id = 1;

	err_code = pm_sec_params_set(&sec_param);
	APP_ERROR_CHECK(err_code);

	err_code = pm_register(pm_evt_handler);
	APP_ERROR_CHECK(err_code);
}


//////////////////////////////////////////////////////////////////////////////
//
// The sec_req_timeout_handler() function is for handling the Security Request timer timeout,
// and will be called each time the Security Request timer expires.
//
//////////////////////////////////////////////////////////////////////////////

static void sec_req_timeout_handler(void)
{
	uint32_t err_code;

	if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
	{
		// Initiate bonding.
		NRF_LOG_RAW_INFO("Start encryption\r\n");

		err_code = pm_conn_secure(m_conn_handle, true);
		
		if (err_code != NRF_ERROR_INVALID_STATE)
		{
			NRF_LOG_RAW_INFO("Err = %d\r\n", err_code);
		}

		NRF_LOG_RAW_INFO("encryption OK\r\n", err_code);
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// The getPin() function is the  Sana proprietary algorithm for generating 6-digit passcode 
// from the MAC address    
//
// Parameter(s):
//
//	destination		resulting passcode
//	maxLength		maximum length of destination
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

#if ENABLE_GENERATED_PASSCODE == 1
static void getPin(uint8_t *destination, int maxLength)
{
	ble_gap_addr_t addr;
	uint32_t err_code;
	// Get BLE address.
#if (NRF_SD_BLE_API_VERSION >= 3)
	err_code = sd_ble_gap_addr_get(&addr);
#else
	err_code = sd_ble_gap_address_get(&addr);
#endif

	APP_ERROR_CHECK(err_code);

	char deviceAddress[128];
	memset(deviceAddress, 0x00, 128);

	char fillPattern[] = "%mmy#V4%Wum=4^xsAk7fxQzLCBGQqgMzgP=bm*73=%EB9E#r*Acjf%v8c?=_U!Wv";

	sprintf(deviceAddress, "%02X:%02X:%02X:%02X:%02X:%02X", addr.addr[5], addr.addr[4], addr.addr[3], addr.addr[2], addr.addr[1], addr.addr[0]);
	strncat(deviceAddress, fillPattern, 64 - strlen(deviceAddress));

	NRF_LOG_RAW_INFO("*** Address for encode: %s\r\n", deviceAddress);

	MD5_CTX context;
	MD5_Init(&context);
	MD5_Update(&context, deviceAddress, 64);

	unsigned char result[32];
	memset(result, 0x00, 32);
	MD5_Final(result, &context);

	uint8_t passkey[33];
	memset(passkey, 0x00, 33);

	char data[4];

	NRF_LOG_RAW_INFO("*** Address MD5:");

	for (int idx = 0; idx < strlen((char *)result); idx++)
	{
		sprintf(data, "%03d", result[idx]);
		strcat((char *)passkey, data);

		NRF_LOG_RAW_INFO(" %02X", result[idx]);
	}

	NRF_LOG_RAW_INFO("\r\n");

	NRF_LOG_RAW_INFO("*** Generated passkey: %s\r\n", (char *)passkey);

	memcpy(destination, passkey, maxLength);
}

#endif // ENABLE_GENERATED_PASSCODE

//////////////////////////////////////////////////////////////////////////////
//
// The passkey() function sets the passcode for MITM BLE security  
//
//////////////////////////////////////////////////////////////////////////////

static void passkey(void)
{
#if ENABLE_GENERATED_PASSCODE == 1
	uint8_t lclPasskey[7];
	getPin(lclPasskey, 6);
#else
	uint8_t lclPasskey[] = "1234567";
#endif // ENABLE_GENERATED_PASSCODE
	ble_opt_t ble_opt;

	ble_opt.gap_opt.passkey.p_passkey = &lclPasskey[0];
	(void)sd_ble_opt_set((int) BLE_GAP_OPT_PASSKEY, &ble_opt);
}

//////////////////////////////////////////////////////////////////////////////
//
// The ble_stack_stop() function stops BLE activity.   
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

void ble_stack_stop(void)
{
	static uint32_t err_code;

	err_code = nrf_sdh_disable_request();
	APP_ERROR_CHECK(err_code);

	ASSERT(!nrf_sdh_is_enabled());
}


//////////////////////////////////////////////////////////////////////////////
//
// The DoBLE() function collects the BLE startup sequence in one place  
//
//////////////////////////////////////////////////////////////////////////////

void DoBLE(void)
{
	// Configure and initialize the BLE stack.

#if ENABLE_BLE_SERVICE_DEBUG
	DangerWillRobinson(2, 100, 100, NO_LED | NO_LED);
#endif // ENABLE_BLE_SERVICE_DEBUG

	ble_stack_init();
	peer_manager_init(bEraseBonds);
	gap_params_init();
	gatt_init();

	if (bSecureBLE)
	{
		passkey();
	}

	services_init();
	advertising_init();
	conn_params_init();
}

#endif // RESTOFSTUFF

