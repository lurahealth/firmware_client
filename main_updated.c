/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the 
 * Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "boards.h"

#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_gatt.h"
#include "nrf_saadc.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#include "nrfx_ppi.h"
#include "nrfx_pwm.h"
#include "nrf_timer.h"
#include "nrfx_saadc.h"

#include "ble_nus.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"

#include "app_timer.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "app_fifo.h"
#include "app_pwm.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// Included for peer manager
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_conn_state.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif



#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Lura_Health_Client"                        /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                200                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (200 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define SAMPLES_IN_BUFFER               11                                          /**< SAADC buffer > */

#define NRF_SAADC_CUSTOM_CHANNEL_CONFIG_SE(PIN_P) \
{                                                   \
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      \
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      \
    .gain       = NRF_SAADC_GAIN1_5,                \
    .reference  = NRF_SAADC_REFERENCE_INTERNAL,     \
    .acq_time   = NRF_SAADC_ACQTIME_10US,           \
    .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      \
    .burst      = NRF_SAADC_BURST_DISABLED,         \
    .pin_p      = (nrf_saadc_input_t)(PIN_P),       \
    .pin_n      = NRF_SAADC_INPUT_DISABLED          \
}

/* UNDEFS FOR DEBUGGING */
#undef RX_PIN_NUMBER
#undef RTS_PIN_NUMBER
#undef LED_4          
#undef LED_STOP       

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
APP_TIMER_DEF(m_timer_id);

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};


/* Lura Health nRF52810 port assignments */
#define ENABLE_ANALOG_PIN 4

/* GLOBALS */
uint32_t AVG_PH_VAL        = 0;
uint32_t AVG_BATT_VAL      = 0;
uint32_t AVG_TEMP_VAL      = 0;
uint16_t total_size 	   = 15;
bool     PH_IS_READ        = false;
bool     BATTERY_IS_READ   = false;
bool     SAADC_CALIBRATED  = false;
bool     CONNECTION_MADE   = false;


static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(1);
static       nrf_saadc_value_t m_buffer_pool[1][SAMPLES_IN_BUFFER];
static       nrf_ppi_channel_t m_ppi_channel;


// Byte array to store total packet
uint8_t total_packet[] = {48,48,48,48,44,    /* pH value, comma */
                          48,48,48,48,44,    /* Temperature, comma */
                          48,48,48,48,10};   /* Battery value, EOL */


// Forward declarations
static inline void enable_pH_voltage_reading  (void);
static inline void enable_switch              (void);
static inline void check_reed_switch          (void);
static inline void disable_pH_voltage_reading (void);
static inline void saadc_init                 (void);
static inline void enable_analog_pin          (void);
static inline void disable_analog_pin         (void);
static        void advertising_start          (bool erase_bonds);
              void send_data_and_disconnect   (void);


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. 
 *          You need to analyse how your product is supposed to react in case of 
 *          Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    NRF_LOG_INFO("ENTERED PM_EVT_HANDLER");
    NRF_LOG_FLUSH();
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
            NRF_LOG_INFO("PM_EVT_CONN_SEC_SUCCEEDED\n");
            NRF_LOG_FLUSH();
            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            NRF_LOG_INFO("PM_EVT_PEERS_DELETED_SUCCEEDED\n");
            NRF_LOG_FLUSH();            
            advertising_start(false);
            break;

        case PM_EVT_BONDED_PEER_CONNECTED:  
            NRF_LOG_INFO("PM_EVT_BONDED_PEER_CONNECTED\n");
            NRF_LOG_FLUSH();
            break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Allow pairing request from an already bonded peer.
            NRF_LOG_INFO("PM_EVT_CONN_SEC_CONFIG_REQ\n");
            NRF_LOG_FLUSH();
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        /* * * * ALL ABOVE CASES ARE NEEDED, BELOW ARE FOR TESTING * * */

        case PM_EVT_CONN_SEC_START:
            NRF_LOG_INFO("PM_EVT_CONN_SEC_START\n");
            NRF_LOG_FLUSH();
            break;

        case PM_EVT_CONN_SEC_FAILED:
            NRF_LOG_INFO("PM_EVT_CONN_SEC_FAILED\n");
            NRF_LOG_FLUSH();
            break;

         case PM_EVT_CONN_SEC_PARAMS_REQ:
            NRF_LOG_INFO("PM_EVT_CONN_SEC_PARAMS_REQ\n");
            NRF_LOG_FLUSH();
            break;

         case PM_EVT_STORAGE_FULL:
            NRF_LOG_INFO("PM_EVT_STORAGE_FULL\n");
            NRF_LOG_FLUSH();
            break;

        case PM_EVT_ERROR_UNEXPECTED:
            NRF_LOG_INFO("PM_EVT_ERROR_UNEXPECTED\n");
            NRF_LOG_FLUSH();
            break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            NRF_LOG_INFO("****** PM_EVT_PEER_DATA_UPDATE_SUCCEEDED *****");
            NRF_LOG_FLUSH();

            NRF_LOG_INFO("data_id: %d\n", p_evt->params.peer_data_update_succeeded.data_id);
            NRF_LOG_FLUSH();

            if (p_evt->params.peer_data_update_succeeded.data_id == 8) {
                send_data_and_disconnect();
            }
            
            break;

       case PM_EVT_PEER_DATA_UPDATE_FAILED:
            NRF_LOG_INFO("PM_EVT_PEER_DATA_UPDATE_FAILED\n");
            NRF_LOG_FLUSH();
            break;

       case PM_EVT_LOCAL_DB_CACHE_APPLIED:
            NRF_LOG_INFO("PM_EVT_LOCAL_DB_CACHE_APPLIED\n");
            NRF_LOG_FLUSH();
            break;

        default:
            NRF_LOG_INFO("PM DEFAULT CASE\n");
            NRF_LOG_FLUSH();
            break;
    }
}



/**@brief Function for initializing the timer module.
 */
static inline void timers_init(void)
{
    uint32_t err_code;
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("app_timer_init finished\n");
    NRF_LOG_FLUSH();
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access 
 *          Profile) parameters of the device. It also sets the permissions 
 *          and appearance.
 */
static inline void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                         (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("gap params init finished\n");
    NRF_LOG_FLUSH();
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may 
 *          need to inform the application about an error.
 *
 * @param[in] nrf_error Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART 
 *          BLE Service and send it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static inline void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, 
                                            p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", 
                                                                    err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("services_init finished\n");
    NRF_LOG_FLUSH();
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection 
 *          Parameters Module which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by 
 *       simply setting the disconnect_on_fail config parameter, but instead 
 *       we use the event handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, 
                                         BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
    NRF_LOG_INFO("on_comm_params_evt entered\n");
    NRF_LOG_FLUSH();
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    NRF_LOG_INFO("conn_params_error_handler entered\n");
    NRF_LOG_FLUSH();
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay  = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("con_params_init finished\n");
    NRF_LOG_FLUSH();
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 * 
 * TO DO: Implement later
 */
static void sleep_mode_enter(void)
{
    // uint32_t err_code; 

    // Go to system-off mode (function will not return; wakeup causes reset).
    //err_code = sd_power_system_off();
    // APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed 
 *          to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    NRF_LOG_INFO("on_adv_event entered\n");
    NRF_LOG_FLUSH();

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    NRF_LOG_INFO("ble_evt_handler entered");
    NRF_LOG_FLUSH();

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            CONNECTION_MADE = true;

            NRF_LOG_INFO("CONNECTION MADE (ble_gap_evt_connected) \n");

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            if(p_ble_evt->evt.gap_evt.params.disconnected.reason  == 
                                                    BLE_HCI_CONNECTION_TIMEOUT)
            {
                //disconnect_reason is BLE_HCI_CONNECTION_TIMEOUT
                NRF_LOG_INFO("connection timeout\n");
            }
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            CONNECTION_MADE = false;
            NRF_LOG_INFO("DISCONNECTED, BLE_GAP_EVT_DISCONNECTED\n");

            err_code = app_timer_stop(m_timer_id);
            APP_ERROR_CHECK(err_code);
            nrfx_timer_uninit(&m_timer);
            nrfx_ppi_channel_free(m_ppi_channel);
            nrfx_saadc_uninit();

            // *** DISABLE ENABLE ***
            disable_analog_pin();

            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };

        do
          {
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            if ((err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != NRF_ERROR_RESOURCES) &&
                (err_code != NRF_ERROR_NOT_FOUND) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                  APP_ERROR_CHECK(err_code);    
            }
      } while (err_code == NRF_ERROR_RESOURCES);

            NRF_LOG_INFO("BLE_GAP_EVT_PHY_UPDATE_REQUEST\n");
            NRF_LOG_FLUSH();
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = 
                sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                      BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            CONNECTION_MADE = false;
            APP_ERROR_CHECK(err_code);

            NRF_LOG_INFO("BLE_GATTC_EVT_TIMEOUT\n");
            NRF_LOG_FLUSH();

            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = 
                sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                      BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            CONNECTION_MADE = false;
            APP_ERROR_CHECK(err_code);

            NRF_LOG_INFO("BLE_GATTC_EVT_TIMEOUT\n");
            NRF_LOG_FLUSH();

            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            NRF_LOG_FLUSH();
            break;

        case BLE_GAP_EVT_PHY_UPDATE:
            NRF_LOG_INFO("BLE_GAP_EVT_PHY_UPDATE, procedure finished\n");
            NRF_LOG_FLUSH();
            break;

        default:
            // No implementation needed.
            NRF_LOG_INFO("ble_evt_handler default case\n");
            NRF_LOG_FLUSH();

            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

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
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, 
                                                ble_evt_handler, NULL);

    NRF_LOG_INFO("ble_stack init finished\n");
    NRF_LOG_FLUSH();                                    
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("peer manager init finished\n");
    NRF_LOG_FLUSH();   
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags               = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

    NRF_LOG_INFO("advertising init finished\n");
    NRF_LOG_FLUSH(); 
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the 
 *          next event occurs.
 */
static void idle_state_handle(void)
{
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        if (err_code == NRF_ERROR_INVALID_STATE) {
            NRF_LOG_INFO("INVALID STATE FAM ******\n");
            NRF_LOG_FLUSH();
        }
        APP_ERROR_CHECK(err_code);
    }
    NRF_LOG_INFO("advertising_start finished\n");
    NRF_LOG_FLUSH(); 
}


/* This function sets enable pin for ISFET circuitry to HIGH
 */
static inline void enable_analog_circuit(void)
{
    nrf_drv_gpiote_out_config_t config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false);
    if(nrf_drv_gpiote_is_init() == false) {
          nrf_drv_gpiote_init();
    }
    nrf_drv_gpiote_out_init(ENABLE_ANALOG_PIN, &config);
    nrf_drv_gpiote_out_set(ENABLE_ANALOG_PIN);

    NRF_LOG_INFO("enable analog circuit finished\n");
    NRF_LOG_FLUSH(); 
}

/* This function sets enable pin for ISFET circuitry to LOW
 */
static inline void disable_analog_pin(void)
{
     // Redundant, but follows design
     nrfx_gpiote_uninit();
    NRF_LOG_INFO("disable_analog_pin finished\n");
    NRF_LOG_FLUSH(); 
}

void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    // To Add Later
    NRF_LOG_INFO("timer handler entered, nrf_drv_timer SAADC\n");
    NRF_LOG_FLUSH(); 
}


static inline void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 15us */
    uint32_t ticks = nrf_drv_timer_us_to_ticks(&m_timer, 35);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = 
                nrf_drv_timer_compare_event_address_get(&m_timer,
                                                        NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event triggers task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("saadc_sampling_event_init finished\n");
    NRF_LOG_FLUSH(); 
}


static inline void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("saadc_samp_event_enable finished\n");
    NRF_LOG_FLUSH(); 
}


static inline void restart_saadc(void)
{
    NRF_LOG_INFO("restart_saadc entered\n");
    NRF_LOG_FLUSH();
    nrfx_timer_uninit(&m_timer);
    nrfx_ppi_channel_free(m_ppi_channel);
    nrfx_saadc_uninit();
    while(nrfx_saadc_is_busy()) {
        // make sure SAADC is not busy
    }
    enable_pH_voltage_reading(); 
    NRF_LOG_INFO("restart_saadc finished\n");
    NRF_LOG_FLUSH();
}


// Pack integer values into byte array to send via bluetooth
void create_bluetooth_packet(uint32_t ph_val,
                             uint32_t batt_val,        
                             uint32_t temp_val, 
                             uint8_t* total_packet)
{
    /*
      {0,0,0,0,44,    pH value arr[0-3], comma arr[4]
       0,0,0,0,44,    temperature arr[5-8], comma arr[9]
       0,0,0,0,10};   battery value arr[10-13], EOL arr[14]
    */

    uint32_t temp = 0;                  // hold intermediate divisions of variables
    uint32_t ASCII_DIG_BASE = 48;

    // Pack ph_val into appropriate location
    temp = ph_val;
    for(int i = 3; i >= 0; i--){
        if (i == 3) total_packet[i] = (uint8_t)(temp % 10 + ASCII_DIG_BASE);
        else {
            temp = temp / 10;
            total_packet[i] = (uint8_t)(temp % 10 + ASCII_DIG_BASE);
        }
    }

    // Pack temp_val into appropriate location
    temp = temp_val;
    for(int i = 8; i >= 5; i--){
        if (i == 8) total_packet[i] = (uint8_t)(temp % 10 + ASCII_DIG_BASE);
        else {
            temp = temp / 10;
            total_packet[i] = (uint8_t)(temp % 10 + ASCII_DIG_BASE);
        }
    }

    // Pack batt_val into appropriate location
    temp = batt_val;

    for(int i = 13; i >= 10; i--){
        if (i == 13) total_packet[i] = (uint8_t)(temp % 10 + ASCII_DIG_BASE);
        else {
            temp = temp / 10;
            total_packet[i] = (uint8_t)(temp % 10 + ASCII_DIG_BASE);
        }
    }
}

static inline uint32_t saadc_result_to_mv(uint32_t saadc_result)
{
    float saadc_denom   = 4095.0;
    float saadc_vref_mv = 3000.0;
    float saadc_res_in_mv = ((float)saadc_result/saadc_denom) * saadc_vref_mv;

    return (uint32_t)saadc_res_in_mv;
}

/**
 * Function is called when SAADC reading event is done. First done event
 * reads pH input, stores in global variable. Second reading stores
 * pH data, combines pH and temp data into a comma-seperated string,
 * then transmits via BLE.
 *
 * BUG: p_buffer[0] is always '0' when reading pH at high frequency.
 *      Workaround is to average values besides 1, divide by 
 *      samples_in_buffer -1 .
 */
static inline void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) 
    {
        NRF_LOG_INFO("SAADC_CALLBACK ENTERED\n");
        NRF_LOG_FLUSH();

        ret_code_t err_code;
        uint32_t   avg_saadc_reading = 0;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 
                                                            SAMPLES_IN_BUFFER); 
        APP_ERROR_CHECK(err_code);
        // Sum and average SAADC values
        for (int i = 1; i < SAMPLES_IN_BUFFER; i++)
        {
            if (p_event->data.done.p_buffer[i] < 0) {
                avg_saadc_reading += 0;
            } 
            else {
                avg_saadc_reading += p_event->data.done.p_buffer[i];
            }
        }
        avg_saadc_reading = avg_saadc_reading/(SAMPLES_IN_BUFFER - 1); 
        // If ph has not been read, read it then restart SAADC to read temp
        if (!PH_IS_READ) {
            AVG_PH_VAL = saadc_result_to_mv(avg_saadc_reading);
            PH_IS_READ = true;
            // Uninit saadc peripheral, restart saadc, enable sampling event
            NRF_LOG_INFO("read pH val, restarting: %d", AVG_PH_VAL);
            NRF_LOG_FLUSH();
            restart_saadc();
        } 
        // If pH has been read but not battery, read battery then restart
        else if (!(PH_IS_READ && BATTERY_IS_READ)) {
            AVG_BATT_VAL = saadc_result_to_mv(avg_saadc_reading);
            NRF_LOG_INFO("read batt val, restarting: %d", AVG_BATT_VAL);
            NRF_LOG_FLUSH();
            BATTERY_IS_READ = true;
            restart_saadc();
        }
        // Once temp, batter and ph have been read, create and send data in packet
        else {
            AVG_TEMP_VAL = saadc_result_to_mv(avg_saadc_reading);
            NRF_LOG_INFO("read temp val: %d", AVG_TEMP_VAL);
            NRF_LOG_FLUSH();
  
            // Create bluetooth data
            create_bluetooth_packet(AVG_PH_VAL, AVG_BATT_VAL, 
                                    AVG_TEMP_VAL, total_packet);
            
            // Disable pH voltage reading
            disable_pH_voltage_reading();
 
            NRF_LOG_INFO("SAADC_CALLBACK finished\n");
            NRF_LOG_FLUSH();


            // Begin advertising
            advertising_start(false);
        }
    }
}


/* Reads pH transducer output
 */
void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_input_t ANALOG_INPUT;
    // Change pin depending on global control boolean
    if (!PH_IS_READ) {
        NRF_LOG_INFO("Setting saadc input to AIN1\n");
        NRF_LOG_FLUSH();
        ANALOG_INPUT = NRF_SAADC_INPUT_AIN1;
    }
    else if (!(PH_IS_READ && BATTERY_IS_READ)) {
        NRF_LOG_INFO("Setting saadc input to AIN3\n");
        NRF_LOG_FLUSH();
        ANALOG_INPUT = NRF_SAADC_INPUT_AIN3;
    }
    else {
        NRF_LOG_INFO("Setting saadc input to AIN0\n");
        NRF_LOG_FLUSH();
        ANALOG_INPUT = NRF_SAADC_INPUT_AIN0;        
    }

    nrf_saadc_channel_config_t channel_config =
            NRF_SAADC_CUSTOM_CHANNEL_CONFIG_SE(ANALOG_INPUT);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    /*
     * BUG: calibration never completes
     */

    // Calibrate offset
    // if(!SAADC_CALIBRATED) {
    //     while (nrfx_saadc_calibrate_offset() != NRFX_SUCCESS) {
    //         NRF_LOG_INFO("calibration does not equal success\n");
    //         NRF_LOG_FLUSH();
    //         nrf_delay_us(10);
    //     }
    //     SAADC_CALIBRATED = true;
    //     while (nrfx_saadc_is_busy()) {
    //         NRF_LOG_INFO("saadc busy while restarting\n");
    //         NRF_LOG_FLUSH();
    //         nrf_delay_ms(1000);
    //     }
    // }   

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("saadc_init finished\n");
    NRF_LOG_FLUSH();
}


/* This function initializes and enables SAADC sampling
 */
static inline void enable_pH_voltage_reading(void)
{
    NRF_LOG_INFO("enable_ph_voltage_reading entered\n");
    NRF_LOG_FLUSH();

    saadc_init();
    saadc_sampling_event_init();
    saadc_sampling_event_enable();
    nrf_pwr_mgmt_run();

    NRF_LOG_INFO("enable_ph_voltage_reading finished\n");
    NRF_LOG_FLUSH();
}

/* Function unitializes and disables SAADC sampling, restarts 1 second timer
 */
static inline void disable_pH_voltage_reading(void)
{
    NRF_LOG_INFO("disable_ph_voltage_reading entered\n");
    NRF_LOG_FLUSH();

    nrfx_timer_uninit(&m_timer);
    nrfx_ppi_channel_free(m_ppi_channel);
    nrfx_saadc_uninit();

    NRF_LOG_INFO("m_timer nrfx_timer uninit, inside disable_ph_voltage_reading\n");
    NRF_LOG_FLUSH();

    // *** DISABLE ENABLE ***
    disable_analog_pin();
}

static inline void single_shot_timer_handler()
{
    NRF_LOG_INFO("single shot timer handler entered\n");
    NRF_LOG_FLUSH();
    // disable timer
    ret_code_t err_code;
    err_code = app_timer_stop(m_timer_id);
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_INFO("m_timer_id stopped, inside single shot timer handler\n");
    NRF_LOG_FLUSH();

    // Delay to ensure appropriate timing between
    enable_analog_circuit();       
    // PWM output, ISFET capacitor, etc
    nrf_delay_ms(10);              
    // Begin SAADC initialization/start
    enable_pH_voltage_reading();

    NRF_LOG_INFO("single_shot_timer_handler finished\n");
    NRF_LOG_FLUSH();
}

void reset_bluetooth_packet()
{
    // Byte array to store total packet
    uint8_t total_packet[] = {48,48,48,48,44,    /* pH value, comma */
                              48,48,48,48,44,    /* Temperature, comma */
                              48,48,48,48,10};   /* Battery value, EOL */
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && 
        (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH 
                                                                - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, 
                                                    m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 
                                               NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

void init_and_start_app_timer()
{
    ret_code_t err_code;

    // Create application timer
    err_code = app_timer_create(&m_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                single_shot_timer_handler);
    APP_ERROR_CHECK(err_code);
        
    // 1 second timer intervals
    err_code = app_timer_start(m_timer_id, APP_TIMER_TICKS(10), NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("APP TIMER STARTED, m_timer_id (gatt_evt_handler) \n");
    NRF_LOG_FLUSH();
}

void send_data_and_disconnect(void)
{
    // Send data
    ret_code_t err_code;
    int debug_ctr = 0;
  
    // Send data
    do
      {
         err_code = ble_nus_data_send(&m_nus, total_packet, 
                                      &total_size, m_conn_handle);
         debug_ctr++;
         if ((err_code != NRF_ERROR_INVALID_STATE) &&
             (err_code != NRF_ERROR_RESOURCES) &&
             (err_code != NRF_ERROR_NOT_FOUND) &&
             (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
         {
                APP_ERROR_CHECK(err_code);    
         }
      } while (err_code == NRF_ERROR_RESOURCES);

    NRF_LOG_INFO("***\nnus_data_send called %d times\n***\n", debug_ctr);
    NRF_LOG_FLUSH();

    // reset global control boolean
    PH_IS_READ = false;
    BATTERY_IS_READ = false;

    for (int i = 0; i < 15; i++) {
        NRF_LOG_INFO("packet[%d]: %u", i, total_packet[i]);
    }
  
    NRF_LOG_FLUSH();
    // Reset bluetooth packet to all 0's
    reset_bluetooth_packet();
 
    // Turn off peripherals
    NRF_LOG_INFO("BLUETOOTH DATA SENT\n");
    NRF_LOG_FLUSH();
    
    nrf_delay_ms(2000);

    // Disconnect from central device
    err_code = sd_ble_gap_disconnect(m_conn_handle, 
                                     BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    APP_ERROR_CHECK(err_code);

    // Restart timer
    err_code = app_timer_start(m_timer_id, APP_TIMER_TICKS(10000), NULL);
    APP_ERROR_CHECK(err_code);
    nrf_pwr_mgmt_run();

    NRF_LOG_INFO("APP TIMER RESTARTED\n");
    NRF_LOG_FLUSH();
}

/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds = false;

    log_init();
    timers_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();

    init_and_start_app_timer();
}

/*
 * @}
 */
