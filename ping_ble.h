
/////////////////////////////////////////////////////////////////////////////////////////////
//
//	File Name:		pingble.h
//	Author(s):		Jeffery Bahr, Dmitriy Antonets, Steve Elstad, Mike Brashears, James Cannan
//	Copyright Notice:	Copyright, 2019, Ping, LLC
//
//	Purpose/Functionality:	Defines and externs associated with pingble.c
//
//	NOTE:  I'm keeping the defines here as in the SDK examples
//
/////////////////////////////////////////////////////////////////////////////////////////////

#ifndef PINGBLE_H
#define PINGBLE_H

///////////////////////////////////////////////////////////////////////////////////////////////
// Defines
///////////////////////////////////////////////////////////////////////////////////////////////

#define DEVICE_NAME                         "Sana"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(10, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.125 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(650, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                                    /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#if MASK_MODE_SECBLE == 1
#define SEC_PARAM_BOND                     1                                       /**< No Perform bonding. */
#define SEC_PARAM_MITM                  	1       					/**< Man In The Middle protection required (applicable when display module is detected). */
#else
#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#endif // MASK_MODE_SECBLE


#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */

#if SEC_PARAM_MITM == 1 
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_DISPLAY_ONLY                /**< Display I/O capabilities. */
#else
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#endif // SEC_PARAM_MITM == 1 

#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define ENABLE_GENERATED_PASSCODE			0										/** If set, generate passcode out of BLE device address */

#define PASSKEY_TXT                     "Passkey:"                                  /**< Message to be displayed together with the pass-key. */
#define PASSKEY_TXT_LENGTH              8                                           /**< Length of message to be displayed together with the pass-key. */
#define PASSKEY_LENGTH                  6                                           /**< Length of pass-key received by the stack for display. */

#define VCFW_INIT			0
#define VCFW_INITIATED		1
#define VCFW_RECEIVING		3
#define VCFW_XMODEM		4
#define VCFW_DONE			5

///////////////////////////////////////////////////////////////////////////////////////////////
// Global Variable Prototypes and Declarations
///////////////////////////////////////////////////////////////////////////////////////////////

extern bool bEraseBonds;
extern void DoBLE(void);

#define BLE_PING_BLE_OBSERVER_PRIO			2

#define BLE_PING_DEF(_name)                                                                          \
static ble_ping_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_PING_BLE_OBSERVER_PRIO,                                                     \
                     ble_ping_on_ble_evt, &_name)

#endif //  PINGBLE_H