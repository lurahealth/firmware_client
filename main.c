/** @file
 *
 * This file contains the main application code to interface with the Microsens
 * or Wellinq Na+ or K+ single ion ISFET sensors, and the Lura Health mobile
 * application.
 */

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


#include <stdint.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
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

// Included for persisent flash reads/writes
#include "fds.h"
#include "nrf_fstorage.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif



#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "LuraHealth_Dan"                             /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                400                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                1000                                        /**< The advertising duration in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(40, UNIT_1_25_MS)             /**< Minimum acceptable connection interval, Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(80, UNIT_1_25_MS)             /**< Maximum acceptable connection interval, Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(5000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(10000)                      /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
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

#define DATA_INTERVAL                   5000

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

#define PACKET_FLOAT_MARKER "%s%d.%1d"
#define PACKET_RVAL_MARKER "%s%d.%3d"
#define LOG_PACKET_FLOAT(val, dec) (uint32_t)(((val) < 0 && (val) > -1.0) ? "-" : ""),   \
                                  (int32_t)(val),                                       \
                                  (int32_t)((((val) > 0) ? (val) - (int32_t)(val)       \
                                              : (int32_t)(val) - (val))*pow(10,dec))


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

// Timer and control flag to enable delay before disconnecting
APP_TIMER_DEF(m_timer_disconn_delay);
bool   DISCONN_DELAY    = true;
#define DISCONN_DELAY_MS   15000 


static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};


/* Lura Health nRF52810 port assignments */
#define ENABLE_ISFET_PIN 8
#define CHIP_POWER_PIN   12

/* GLOBALS */
uint32_t AVG_PX_VAL   = 0;
uint32_t AVG_BATT_VAL = 0;
uint32_t AVG_TEMP_VAL = 0;
uint32_t HVN_COUNTER  = 0;
uint32_t PACK_CTR     = 0;
bool     SEND_BUFFERED_DATA  = false;
bool     HVN_TX_EVT_COMPLETE = false;
bool     PX_IS_READ          = false;
bool     BATTERY_IS_READ     = false;
bool     SAADC_CALIBRATED    = false;
bool     CONNECTION_MADE     = false;
bool     CAL_MODE            = false;
bool     PT1_READ            = false;
bool     PT2_READ            = false;
bool     PT3_READ            = false;
float     PT1_PX_VAL        = 0;
float     PT1_MV_VAL        = 0;
float     PT2_PX_VAL        = 0;
float     PT2_MV_VAL        = 0;
float     PT3_PX_VAL        = 0;
float     PT3_MV_VAL        = 0;
float     REF_TEMP          = 0;
float     CURR_TEMP         = 0;
int      NUM_CAL_PTS       = 0;
float     MVAL_CALIBRATION  = 0;
float     BVAL_CALIBRATION  = 0;
float     RVAL_CALIBRATION  = 0;
float     CAL_PERFORMED     = 0;
static volatile uint8_t write_flag=0;

// Byte array to store total packet
uint8_t total_packet[] = {48,48,48,48,44,    /* real px value, comma */
                          48,48,48,48,44,    /* Temperature, comma */
                          48,48,48,48,44,    /* Battery value, comma */
                          48,48,48,48,10};   /* raw px value, EOL */
// Total size of bluetooth packet
uint16_t   total_size = 20;

/* Used for reading/writing calibration values to flash */
#define MVAL_FILE_ID      0x1110
#define MVAL_REC_KEY      0x1111
#define BVAL_FILE_ID      0x2220
#define BVAL_REC_KEY      0x2221
#define RVAL_FILE_ID      0x3330
#define RVAL_REC_KEY      0x3331
#define CAL_DONE_FILE_ID  0x4440
#define CAL_DONE_REC_KEY  0x4441


// Forward declarations
void create_bluetooth_packet(uint32_t px_val, uint32_t batt_val,        
                             uint32_t temp_val, float px_val_cal,
                             uint8_t* total_packet);
void init_and_start_app_timer   (void);
void send_data_and_restart_timer(void);
void enable_px_voltage_reading  (void);
void disable_px_voltage_reading (void);
void saadc_init                 (void);
void enable_isfet_circuit       (void);
void disable_isfet_circuit      (void);
void turn_chip_power_on         (void);
void turn_chip_power_off         (void);
void restart_saadc              (void);
void write_cal_values_to_flash   (void);
void check_for_buffer_done_signal(char **packet);
void linreg                     (int num, float x[], float y[]);
void perform_calibration        (uint8_t cal_pts);
float calculate_px_from_mV       (uint32_t px_val);
float calculate_celsius_from_mv  (uint32_t mv);
float validate_float_range        (float val);
static void advertising_start   (bool erase_bonds);
static void idle_state_handle   (void);
uint32_t saadc_result_to_mv     (uint32_t saadc_result);
uint32_t sensor_temp_comp       (uint32_t raw_analyte_mv, uint32_t temp_mv);

/* 
 * Buffers for storing data through System ON sleep periods
 *
 * Data will be appended to this buffer every time an advertising
 * timeout occurs. If there is data stored in the buffer when a 
 * connection is made, a "primer" packet containing total number
 * of packets (one packet per data set in buffer) will be sent as
 * one packet, and then the data will be sent in FIFO order. I.E.
 * buffer[0] will contain the "oldest" data, buffer[n-1] will contain
 * the "newest" data, and packets will be sent buffer[0] to buffer [n-1]
 *
 * Buffers can store five days worth of data. Data is collected once 
 * every 15 minutes; 96 readings per day * 5 days = 480 readings.
 * Each reading requires 10 bytes (3x uint16_t, 1x float), so this 
 * buffer system can store 4.8kB of data.
 */
 #define DATA_BUFF_SIZE 480
 uint16_t TOTAL_DATA_IN_BUFFERS = 0;

 uint16_t px_mv  [DATA_BUFF_SIZE];
 uint16_t temp_mv[DATA_BUFF_SIZE];
 uint16_t batt_mv[DATA_BUFF_SIZE];
 float     px_cal [DATA_BUFF_SIZE];

// Function to initialize all buffers with values of 0
 void init_data_buffers(void)
 {
    for(int i = 0; i < DATA_BUFF_SIZE; i++) {
        px_mv  [i] = 0;
        temp_mv[i] = 0;
        batt_mv[i] = 0;
        px_cal [i] = 0;
    }
 }

 void reset_data_buffers(void)
 {
    NRF_LOG_INFO("RESETTING px BUFFERS");
    int i = 0;
    while(px_mv[i] != 0) {
        px_mv  [i] = 0;
        temp_mv[i] = 0;
        batt_mv[i] = 0;
        px_cal [i] = 0;
        i++;
    }
    TOTAL_DATA_IN_BUFFERS = 0;
    PACK_CTR = 0;
 }

/* Function to store data in buffer in case of advertising timeout
 *
 * On the next connection after timeouts causing data to be buffered,
 * first add the most recent data to the buffer then call send_buffered_data()
 */
void add_data_to_buffers(void)
{
    // Values stored in px_cal[i] may be zero while other values are
    // non-zero. Iterate through px_mv to find first "empty" buffer index
    // and store data at the same index for other buffers
    int i = 0;
    while(px_mv[i] != 0) { i++; }
    px_mv[i]   = (uint16_t) AVG_PX_VAL;
    temp_mv[i] = (uint16_t) AVG_TEMP_VAL;
    batt_mv[i] = (uint16_t) AVG_BATT_VAL;
    if(CAL_PERFORMED) {
        float px_cal_val = calculate_px_from_mV(sensor_temp_comp(AVG_PX_VAL, AVG_TEMP_VAL));
        px_cal[i] = validate_float_range(px_cal_val);
    }
    TOTAL_DATA_IN_BUFFERS++;
}

void create_and_send_buffer_primer_packet(void)
{
    uint32_t err_code;
    uint32_t ASCII_DIG_BASE = 48;
    uint16_t primer_len = 10;
    // Format and send primer packet first
    uint8_t primer[] = {'T','O','T','A','L','_',48,48,48,10};
    uint32_t temp = TOTAL_DATA_IN_BUFFERS;
    NRF_LOG_INFO("Total data in buffers: %d", TOTAL_DATA_IN_BUFFERS);
    // Packing protocol for number abc: 
    // [... 0, 0, c] --> [... 0, b, c] --> [... a, b, c]
    for(int i = primer_len - 2; i > 5; i--) {
        if (i == primer_len - 2) primer[i] = (uint8_t)(temp % 10 + ASCII_DIG_BASE);
        else {
            temp = temp / 10;
            primer[i] = (uint8_t)(temp % 10 + ASCII_DIG_BASE);
        }
    }
    // Send primer packet
    do{
         err_code = ble_nus_data_send(&m_nus, primer, &primer_len, m_conn_handle);
         NRF_LOG_INFO("error code: %u", err_code);
         if ((err_code != NRF_ERROR_INVALID_STATE) &&
             (err_code != NRF_ERROR_RESOURCES) &&
             (err_code != NRF_ERROR_NOT_FOUND))
         {
                
                APP_ERROR_CHECK(err_code);              
         }
         nrf_delay_us(100);
    } while (err_code == NRF_ERROR_RESOURCES);
}

void send_buffered_data(void)
{
    uint32_t err_code;
    HVN_COUNTER = 0;

    // Create packet
    create_bluetooth_packet((uint32_t)px_mv[PACK_CTR], 
                            (uint32_t)batt_mv[PACK_CTR], 
                            (uint32_t)temp_mv[PACK_CTR], 
                             px_cal[PACK_CTR], total_packet);
    do {
        err_code = ble_nus_data_send(&m_nus, total_packet, 
                                     &total_size, m_conn_handle);
        if ((err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND))
        {   
            APP_ERROR_CHECK(err_code);              
        }
    } while (err_code == NRF_ERROR_RESOURCES);
    NRF_LOG_INFO("%d PACKETS SENT", PACK_CTR+1);
}

void check_for_buffer_done_signal(char **packet)
{
    NRF_LOG_INFO("Checking received packet for done signal...");
    char *DONE  = "DONE";
    if (strstr(*packet, DONE) != NULL){
        NRF_LOG_INFO("Received DONE signal");
        // Disconnect
        uint32_t err_code;
        err_code = sd_ble_gap_disconnect(m_conn_handle, 
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }
}

/*
 * Functions for translating from temperature sensor millivolt output 
 * to real-world temperature values. The thermistor resistance should first
 * be calculated from the temperature millivolt reading. Then, A simplified 
 * version of the Steinhart and Hart equation can be used to calculate 
 * Kelvins. Data is sent in Celsius for ease of debugging (data sheet values
 * are listed in Celsius), and the mobile application may convert to Fahrenheit
 */

// Helper function to convert millivolts to thermistor resistance
float mv_to_therm_resistance(uint32_t mv)
{
    float therm_res = 0;
    float Vtemp     = 0;
    float R1        = 10000.0;
    float Vin       = 1800;

    Vtemp = (float) mv;
    therm_res = (Vtemp * R1) / (Vin - Vtemp);

    // Catch invalid resistance values, set to 500 ohm if negative
    if(therm_res < 500)
        therm_res = 500;

    return therm_res;
}

// Helper function to convert thermistor resistance to Kelvins
float therm_resistance_to_kelvins(float therm_res)
{
    float kelvins_constant = 273.15;
    float ref_temp         = 25.0 + kelvins_constant;
    float ref_resistance   = 10000.0;
    float beta_constant    = 3380.0;
    float real_kelvins     = 0;

    real_kelvins = (beta_constant * ref_temp) / 
                      (beta_constant + (ref_temp * log(therm_res/ref_resistance)));

    return real_kelvins;
}

// Helper function to convert Kelvins to Celsius
float kelvins_to_celsius(float kelvins)
{
    float kelvins_constant = 273.15;
    return kelvins - kelvins_constant;
}

// Function to fully convert temperature millivolt output to degrees celsius
float calculate_celsius_from_mv(uint32_t mv)
{
    float therm_res = 0;
    float kelvins   = 0;
    float celsius   = 0;
    therm_res = mv_to_therm_resistance(mv);
    kelvins   = therm_resistance_to_kelvins(therm_res);
    celsius   = kelvins_to_celsius(kelvins);

    // Round celsius to nearest 0.1
    celsius = celsius * 100.0;
    if ((uint32_t)celsius % 10 > 5)
        celsius = celsius + 10.0;
    celsius = celsius / 10.0;
    celsius = round(celsius);
    celsius = celsius / 10.0 + 0.000000001; // Workaround for floating point error

    return celsius;
}

/* Adjusts analyte sensor mV output (raw_mv) based on temperature dependance.
 * The analyte sensor output is linearly dependant on temperature, i.e. 1400mV
 * @ 25C, 1405mV @ 30C, and 1410mV @ 35C may all represent the same pH value.
 *
 * Adjustment is made using the difference between reference temp (REF_TEMP)
 * recorded during calibration, and current temp reading taken at the same
 * time as the analyte sensor mV output.
 */
uint32_t sensor_temp_comp(uint32_t raw_analyte_mv, uint32_t temp_mv)
{
    float TEMP_DEPENDANCE = 1.10;   // Reasonable avg. mV/C dependance between sensors
    float curr_temp = calculate_celsius_from_mv(temp_mv);
    float temp_diff  = REF_TEMP - curr_temp;
    return raw_analyte_mv + round((temp_diff * TEMP_DEPENDANCE));
}

/* Takes SAADC mV reading as input and returns the actual battery
 * voltage, recorded at time of sensor reading. Battery voltage
 * is connected to a voltage divider with constant resistors
 * R1 = 3M and R2 = 1M. 
 */
uint32_t calculate_battvoltage_from_mv(uint32_t batt_mv)
{
    // batt_mv = real_batt_voltage * (R2 / (R1 + R2))
    // real_batt_voltage = batt_mv * ((R1 + R2) / R2)
    uint32_t R1 = 3000000;
    uint32_t R2 = 1000000;
    return batt_mv * ((R1 + R2) / R2);
}


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
    NRF_LOG_INFO("ENTERED PM_EVT_HANDLER\n");
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
            NRF_LOG_INFO("PM_EVT_CONN_SEC_SUCCEEDED");
            NRF_LOG_FLUSH();
            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            NRF_LOG_INFO("PM_EVT_PEERS_DELETE_SUCCEEDED");
            NRF_LOG_FLUSH();            
            advertising_start(false);
            break;

        case PM_EVT_BONDED_PEER_CONNECTED:  
            NRF_LOG_INFO("PM_EVT_BONDED_PEER_CONNECTED");
            NRF_LOG_FLUSH();
            break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Allow pairing request from an already bonded peer.
            NRF_LOG_INFO("PM_EVT_CONN_SEC_CONFIG_REQ");
            NRF_LOG_FLUSH();
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        default:
            break;
    }
}





/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access 
 *          Profile) parameters of the device. It also sets the permissions 
 *          and appearance.
 */
void gap_params_init(void)
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

// Helper function
void substring(char s[], char sub[], int p, int l) {
   int c = 0;
   
   while (c < l) {
      sub[c] = s[p+c-1];
      c++;
   }
   sub[c] = '\0';
}

/* Takes mmol/L value and converts to pK or pNa
 */
float mmol_per_L_to_px(float mmol_L)
{
    // Convert mmol/L to mol/L
    float mol_L = mmol_L / 1000;
    return -log10(mol_L);
}

/* Takes pK or pNa value and converts to mmol/L
 */
float px_to_mmol_per_L(float px)
{
    return pow(10.0, -px) * 1000;
}

void read_saadc_and_store_avg_in_cal_pt(int samples)
{
    uint32_t AVG_MV_VAL = 0;
    nrf_saadc_value_t temp_val = 0;
    ret_code_t err_code;
    // Average readings
    for (int i = 0; i < samples; i++) {
      err_code = nrfx_saadc_sample_convert(0, &temp_val);
      APP_ERROR_CHECK(err_code);
      if (temp_val < 0)
          AVG_MV_VAL += 0;
      else
          AVG_MV_VAL += saadc_result_to_mv(temp_val);
      nrf_delay_us(10);
    }
    AVG_MV_VAL = AVG_MV_VAL / samples;
    // Assign averaged readings to the correct calibration point
    if(!PT1_READ){
      PT1_MV_VAL = (float)AVG_MV_VAL;
    }
    else if (PT1_READ && !PT2_READ){
      PT2_MV_VAL = (float)AVG_MV_VAL;
    }
    else if (PT1_READ && PT2_READ && !PT3_READ){
       PT3_MV_VAL = (float)AVG_MV_VAL;
    }  
}

void read_saadc_and_set_ref_temp(int samples)
{
    uint32_t AVG_MV_VAL = 0;
    nrf_saadc_value_t temp_val = 0;
    ret_code_t err_code;
    // Average readings
    for (int i = 0; i < samples; i++) {
      err_code = nrfx_saadc_sample_convert(0, &temp_val);
      APP_ERROR_CHECK(err_code);
      if (temp_val < 0)
          AVG_MV_VAL += 0;
      else
          AVG_MV_VAL += saadc_result_to_mv(temp_val);
      nrf_delay_us(10);
    }
    AVG_MV_VAL = AVG_MV_VAL / samples;
    if (!PT1_READ) {
        CURR_TEMP = calculate_celsius_from_mv(AVG_MV_VAL);
        CURR_TEMP = validate_float_range(CURR_TEMP); 
        REF_TEMP  = CURR_TEMP;
        PT1_READ = true;
    }
    else if (PT1_READ && !PT2_READ) {
        CURR_TEMP = calculate_celsius_from_mv(AVG_MV_VAL);
        CURR_TEMP = validate_float_range(CURR_TEMP); 
        REF_TEMP = REF_TEMP + CURR_TEMP;
        if (NUM_CAL_PTS == 2) {REF_TEMP = REF_TEMP / 2.0;}
        PT2_READ = true;
    }
    else if (PT1_READ && PT2_READ && !PT3_READ) {
       CURR_TEMP = calculate_celsius_from_mv(AVG_MV_VAL);
       CURR_TEMP = validate_float_range(CURR_TEMP); 
       REF_TEMP = REF_TEMP + CURR_TEMP;
       REF_TEMP = REF_TEMP / 3.0;
       PT3_READ = true;
    }
    NRF_LOG_INFO("ref temp: " NRF_LOG_FLOAT_MARKER " ", NRF_LOG_FLOAT(REF_TEMP));
}

// Read saadc values for temperature and px to store for calibration
void read_saadc_for_calibration(void) 
{
    int NUM_SAMPLES = 150;
    PX_IS_READ      = false;
    BATTERY_IS_READ = false;

    // Reset SAADC state before taking first calibration point
    if (!PT1_READ) {disable_px_voltage_reading();}
    enable_px_voltage_reading();
    read_saadc_and_store_avg_in_cal_pt(NUM_SAMPLES);   
    // Reset saadc to read temperature value
    disable_px_voltage_reading();
    PX_IS_READ = true;
    BATTERY_IS_READ = true; // Work around to read temperature values
    enable_px_voltage_reading();
    read_saadc_and_set_ref_temp(NUM_SAMPLES);
    disable_px_voltage_reading();
}

/* Helper function to clear calibration global state variables
 */
void reset_calibration_state()
{
    CAL_MODE        = false;
    CAL_PERFORMED   = 1.0;
    PT1_READ        = false;
    PT2_READ        = false;
    PT3_READ        = false;
    PX_IS_READ      = false;
    BATTERY_IS_READ = false;
}

/*
 * Use the values read from read_saadc_for_calibration to reset the M, B and R values
 * to recalibrate accuracy of ISFET voltage output to px value conversions
 */
void perform_calibration(uint8_t cal_pts)
{
  if (cal_pts == 1) {
    // Compare mV for px value to mV calculated for same px with current M & B values,
    // then adjust B value by the difference in mV values (shift intercept of line)
    float incorrect_px  = calculate_px_from_mV((uint32_t)PT1_MV_VAL);
    float cal_adjustment = PT1_PX_VAL - incorrect_px;
    BVAL_CALIBRATION = BVAL_CALIBRATION + cal_adjustment;
  }
  else if (cal_pts == 2) {
    // Create arrays of px value and corresponding mV values (change all line properties)
    float x_vals[] = {PT1_MV_VAL, PT2_MV_VAL};
    float y_vals[] = {PT1_PX_VAL, PT2_PX_VAL};
    linreg(cal_pts, x_vals, y_vals);
  }
  else if (cal_pts == 3) {
    // Create arrays of px value and corresponding mV values (change all line properties)
    float x_vals[] = {PT1_MV_VAL, PT2_MV_VAL, PT3_MV_VAL};
    float y_vals[] = {PT1_PX_VAL, PT2_PX_VAL, PT3_PX_VAL};
    linreg(cal_pts, x_vals, y_vals);
  }
}

void stop_disconn_delay_timer()
{
    ret_code_t err_code;
    err_code = app_timer_stop(m_timer_disconn_delay);
    APP_ERROR_CHECK(err_code);
}

void disconnect_from_central()
{
    uint32_t err_code;
    err_code = sd_ble_gap_disconnect(m_conn_handle, 
                                     BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
}

/* Packs the mV value and temperature reading recorded for the current
 * calibration point into the confirmation packet
 */
void pack_cal_values_into_confirm_packet(char confirm_packet[3][24], int cal_pt) {
      // Pack packet depending on calibration point
      if (cal_pt == 1)
          sprintf(confirm_packet[cal_pt-1], "PT1CONF %u mV, " PACKET_FLOAT_MARKER " C\n", 
                                           (uint32_t)PT1_MV_VAL, LOG_PACKET_FLOAT(CURR_TEMP,1));
      else if (cal_pt == 2)
          sprintf(confirm_packet[cal_pt-1], "PT2CONF %u mV, " PACKET_FLOAT_MARKER " C\n", 
                                           (uint32_t)PT2_MV_VAL, LOG_PACKET_FLOAT(CURR_TEMP,1));
      else if (cal_pt == 3)
          sprintf(confirm_packet[cal_pt-1], "PT3CONF %u mV, " PACKET_FLOAT_MARKER " C\n", 
                                           (uint32_t)PT3_MV_VAL, LOG_PACKET_FLOAT(CURR_TEMP,1));
}

/* Packs the results of calibration into a packet, including M and B values from
 * Y = Mx + B, the R^2 accuracy of the linear regression, and the reference temp.
 *
 * When converting mV to analyte values, this application uses an calibration 
 * with units such that M is "pX / mV" and B is pX. This packing function coverts 
 * M to "mV / pX" and B to mV, as these are the most useful values to the user
 */
void pack_lin_reg_values_into_packet(char report_packet[45], uint16_t *pack_len)
{
    uint16_t len = 0;
    for (int i = 0; i < 45; i++) {report_packet[i] = 0;}
    sprintf(report_packet, "M=" PACKET_FLOAT_MARKER ", B=" PACKET_FLOAT_MARKER ", "
                           "R=" PACKET_RVAL_MARKER ", C=" PACKET_FLOAT_MARKER " \n", 
                                    LOG_PACKET_FLOAT(1.0/MVAL_CALIBRATION,1),        
                                    LOG_PACKET_FLOAT(BVAL_CALIBRATION/MVAL_CALIBRATION*-1,1),   
                                    LOG_PACKET_FLOAT(fabs(RVAL_CALIBRATION),3),      
                                    LOG_PACKET_FLOAT(REF_TEMP,1));
   for (int i = 0; i < 45; i++) {
      if (report_packet[i] > 0)
        len++;
   }
   report_packet[len] = '\n';
   *pack_len = len+2;
}

/*
 * Checks packet contents to appropriately perform calibration
 */
void check_for_calibration(char **packet)
{
    // Possible Strings to be received by px device
    char *STARTCAL  = "STARTCAL";
    char *PWROFF    = "PWROFF";
    char *PT        = "PT";
    // Possible strings to send to mobile application
    char CALBEGIN[9] = {"CALBEGIN\n"};   
    char CALRESULTS[45];
    char PT_CONFS[3][24];
    // Variables to hold sizes of strings for ble_nus_send function
    uint16_t SIZE_BEGIN = 9;
    uint16_t SIZE_CONF  = 24;
    uint16_t SIZE_RESULTS;
    uint32_t err_code;

    if (strstr(*packet, PWROFF) != NULL){
        NRF_LOG_INFO("received pwroff\n");
        disconnect_from_central();
        nrfx_gpiote_out_clear(CHIP_POWER_PIN);
    }

    if (strstr(*packet, STARTCAL) != NULL) {
        char cal_pts_str[1];
        CAL_MODE = true;
        stop_disconn_delay_timer();
        disable_px_voltage_reading();
        // Parse integer from STARTCALX packet, where X is 1, 2 or 3
        substring(*packet, cal_pts_str, 9, 1);
        NUM_CAL_PTS = atoi(cal_pts_str);
        err_code = ble_nus_data_send(&m_nus, CALBEGIN, &SIZE_BEGIN, m_conn_handle);
        APP_ERROR_CHECK(err_code);
    }

    if (strstr(*packet, PT) != NULL) {
        char px_val_substring[5];
        char cal_pt_str[1];
        int cal_pt = 0;
        // Parse out calibration point from packet
        substring(*packet, cal_pt_str, 3, 1);
        cal_pt = atoi(cal_pt_str);
        // Parse out px value from packet
        substring(*packet, px_val_substring, 5, 4);
        // Assign px value to appropriate variable
        if (cal_pt == 1)
            PT1_PX_VAL = mmol_per_L_to_px(atof(px_val_substring)); 
        else if (cal_pt == 2)
            PT2_PX_VAL = mmol_per_L_to_px(atof(px_val_substring)); 
        else if (cal_pt == 3)
            PT3_PX_VAL = mmol_per_L_to_px(atof(px_val_substring)); 
        // Read calibration data and send confirmation packet
        read_saadc_for_calibration();
        pack_cal_values_into_confirm_packet(PT_CONFS, cal_pt);
        err_code = ble_nus_data_send(&m_nus, PT_CONFS[cal_pt - 1], 
                                     &SIZE_CONF, m_conn_handle);
        APP_ERROR_CHECK(err_code);
        // Restart normal data transmission if calibration is complete
        if (NUM_CAL_PTS == cal_pt) {
          perform_calibration(cal_pt);
          pack_lin_reg_values_into_packet(CALRESULTS, &SIZE_RESULTS);
          NRF_LOG_INFO("length: %u", SIZE_RESULTS);
          err_code = ble_nus_data_send(&m_nus, CALRESULTS, &SIZE_RESULTS, m_conn_handle);
          write_cal_values_to_flash();
          reset_calibration_state();
          // Delay before disconnecting from central
          err_code = app_timer_start(m_timer_disconn_delay, 
                                     APP_TIMER_TICKS(10000), NULL);
          APP_ERROR_CHECK(err_code);
          //disconnect_from_central();
        }
    }
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART 
 *          BLE Service and send it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        NRF_LOG_INFO("RECEIVED DATA FROM NUS DATA HANDLER");
        // Array to store data received by smartphone will never exceed 9 characters
        char data[10];
        // Pointer to array
        char *data_ptr = data;
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS.\n");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, 
                                            p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                // Parse data into array
                data[i] = p_evt->params.rx_data.p_data[i];
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", 
                                                                    err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        // Check pack for calibration protocol details
        check_for_calibration(&data_ptr);
        check_for_buffer_done_signal(&data_ptr);
    }

    if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
    {
        send_data_and_restart_timer();
    }

}


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

    NRF_LOG_INFO("INSIDE CONN PARAMS EVT\n");

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, 
                                         BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
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
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code; 

    // Go to system-off mode (function will not return; wakeup causes reset).
    //err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
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
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("FAST ADVERTISING STARTED");
            break;
        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("on_adv_evt IDLE EVENT");
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

    NRF_LOG_INFO("INSIDE BLE EVT HANDLER\n");
    NRF_LOG_INFO("evt id: %u\n", p_ble_evt->header.evt_id);

    switch (p_ble_evt->header.evt_id)
    {
        case 38:
            NRF_LOG_INFO("CASE 38\n");
            // Store data in buffer if there is space remaining
            if (TOTAL_DATA_IN_BUFFERS < DATA_BUFF_SIZE - 1)
                add_data_to_buffers();
                /* 
                 * TO DO: Figure out how to compress data if buffer
                 *        fills up to 480. Unlikely, but application
                 *        should be able to handle this.
                 */
            init_and_start_app_timer();
            break;

        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            CONNECTION_MADE = true;
            // Set TX power to highest setting
            sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_conn_handle, 4);

            NRF_LOG_INFO("CONNECTION MADE (ble_gap_evt) \n");

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
            NRF_LOG_INFO("DISCONNECTED\n");
            HVN_COUNTER = 0;
            init_and_start_app_timer();

            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, 
                                                                         &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            NRF_LOG_INFO("TIMEOUT GATTC EVT\n");
            // Disconnect on GATT Client timeout event.
            err_code = 
                sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                      BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            // restart timer
            init_and_start_app_timer();
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            NRF_LOG_INFO("TIMEOUT GATTS EVT\n");
            // Disconnect on GATT Server timeout event.
            err_code = 
                sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                      BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            APP_ERROR_CHECK(err_code);
            // restart timer
            init_and_start_app_timer();
            break;

         case BLE_GAP_EVT_TIMEOUT:
            NRF_LOG_INFO("TIMEOUT GAP EVT\n");
            // Restart timer 
            init_and_start_app_timer();
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
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            HVN_COUNTER++;   
            NRF_LOG_INFO("EVT HVN TX COMPLETED, total evts: %d\n", HVN_COUNTER);
            HVN_TX_EVT_COMPLETE = true;
            break;

        default:
            // No implementation needed.
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

    // Set TX power to highest setting
    sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, 4);
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

        APP_ERROR_CHECK(err_code);
    }
    NRF_LOG_INFO("Advertising started");
    NRF_LOG_FLUSH();
}


/* This function sets enable pin for ISFET circuitry to HIGH
 */
void enable_isfet_circuit(void)
{
    ret_code_t err_code;
    nrf_drv_gpiote_out_config_t config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false);
    if(nrf_drv_gpiote_is_init() == false) {
          err_code = nrf_drv_gpiote_init();
          APP_ERROR_CHECK(err_code);
    }
    err_code = nrf_drv_gpiote_out_init(ENABLE_ISFET_PIN, &config);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_out_set(ENABLE_ISFET_PIN);
}

/* This function holds POWER ON line HIGH to keep chip turned on
 */
void turn_chip_power_on(void)
{
    nrf_drv_gpiote_out_config_t config = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(false);
    if(nrf_drv_gpiote_is_init() == false) {
          nrf_drv_gpiote_init();
    }
    nrf_drv_gpiote_out_init(CHIP_POWER_PIN, &config);
    nrf_drv_gpiote_out_set(CHIP_POWER_PIN);
}

/* This functions turns POWER ON line LOW to turn the chip completely off
 */
void turn_chip_power_off(void)
{
    nrfx_gpiote_out_clear(CHIP_POWER_PIN);
}

/* This function sets enable pin for ISFET circuitry to LOW
 */
void disable_isfet_circuit(void)
{
     nrfx_gpiote_out_clear(ENABLE_ISFET_PIN);
     nrfx_gpiote_out_uninit(ENABLE_ISFET_PIN);
}

/* This function uninits saadc, clear interrupt for power optimization, and
 * restarts SAADC immediately while changing analog input pin to sense px,
 * temperature or battery level values accordingly
 */
void restart_saadc(void)
{
    nrfx_saadc_uninit();
    NVIC_ClearPendingIRQ(SAADC_IRQn);
    while(nrfx_saadc_is_busy()) {
        // make sure SAADC is not busy
    }
    enable_px_voltage_reading(); 
}

float calculate_px_from_mV(uint32_t px_val)
{
    // px = (px_val - BVAL_CALIBRATION) / (MVAL_CALIBRATION)
    return ((float)px_val * MVAL_CALIBRATION) + BVAL_CALIBRATION;
}

// Returns 99.9 if val is >= 100.0, returns 0.1 if val is < 0
float validate_float_range(float val)
{
    if (val > 99.9) {
        return 99.9;
    } else if (val < 0.0) {
        return 0.1;
    } else {
        return val;
    }
}

// Returns 3000 (max mV range) if val is > 3000
uint32_t validate_uint_range(uint32_t val)
{
    if (val > 3000)
        return 3000;
    else
        return val;
}

/* Calculates the pK value from stored mv value, then converts from pK to mmol/L.
 * Data is rounded to nearest 0.1 mmol/L decimal place. Stores each digit from 
 * packet[p] to packet[p+3]
 */
void pack_calibrated_px_val(uint32_t px_mv_val, uint32_t temp_val,
                                        uint8_t* total_packet, uint8_t p)
{
    uint32_t ASCII_DIG_BASE = 48;
    float pX_value = calculate_px_from_mV(sensor_temp_comp(px_mv_val, temp_val));
    float x_mmol_L = px_to_mmol_per_L(pX_value);
    x_mmol_L = validate_float_range(x_mmol_L);
    float x_decimal_vals = (x_mmol_L - floor(x_mmol_L)) * 100;
    // Round K decimal values to nearest 0.1 accuracy
    x_decimal_vals = round(x_decimal_vals / 10) * 10;
    // If decimals rounds to 100, increment k_mmol_l and set decimals to 0
    if (x_decimal_vals == 100) {
        x_mmol_L = x_mmol_L + 1.0;
        x_decimal_vals = 0.00;
    }
    total_packet[p]   = (uint8_t) ((uint8_t)floor(x_mmol_L / 10) + ASCII_DIG_BASE);
    total_packet[p+1] = (uint8_t) ((uint8_t)floor((uint8_t)x_mmol_L % 10) + ASCII_DIG_BASE);
    total_packet[p+2] = 46;
    total_packet[p+3] = (uint8_t) (((uint8_t)x_decimal_vals / 10) + ASCII_DIG_BASE);
}

/* Packs temperature value into total_packet[5-8], as degrees Celsius.
 * Stores each digit from packet[p] to packet[p+3], with 0.1 C accuracy
 */
void pack_temperature_val(uint32_t temp_val, uint8_t* total_packet, uint8_t p)
{
    uint32_t ASCII_DIG_BASE = 48;
    float real_temp = calculate_celsius_from_mv(temp_val);
    real_temp = validate_float_range(real_temp);
    float temp_decimal_vals = (real_temp - floor(real_temp)) * 100;
    total_packet[p]   = (uint8_t) ((uint8_t)floor(real_temp / 10) + ASCII_DIG_BASE);
    total_packet[p+1] = (uint8_t) ((uint8_t)floor((uint8_t)real_temp % 10) + ASCII_DIG_BASE);
    total_packet[p+2] = 46;
    total_packet[p+3] = (uint8_t) (((uint8_t)temp_decimal_vals / 10) + ASCII_DIG_BASE);
}

/* Packs a mV value between 0 - 3000 into char string total_packet, from 
 * total_packet[p] to total_packet[p + l - 1]. Ex: 1234 --> ['1','2','3','4']
 *
 * uint8_t p is the position of most significat digit, i.e. '1' in 1234
 * uint8_t l is the length of the number to be stored
 */
void pack_mv_value(uint32_t mv_val, uint8_t* total_packet, uint8_t p, uint8_t l)
{
    uint32_t ASCII_DIG_BASE = 48;
    uint32_t temp = 0;            // hold intermediate divisions of variables
    // Packing protocol for number abcd: 
    //  [... 0, 0, 0, d] --> [... 0, 0, c, d] --> ... --> [... a, b, c, d]
    temp = mv_val;
    for(int i = p+l-1; i >= p; i--){
        if (i == p+l-1) total_packet[i] = (uint8_t)(temp % 10 + ASCII_DIG_BASE);
        else {
            temp = temp / 10;
            total_packet[i] = (uint8_t)(temp % 10 + ASCII_DIG_BASE);
        }
    }
}

/* Pack values into byte array to send via bluetooth
 */
void create_bluetooth_packet(uint32_t px_val,   uint32_t batt_val,        
                             uint32_t temp_val, float px_val_cal,
                             uint8_t* total_packet)
{
    /*
        {0,0,0,0,44,    calibrated px value arr[0-3], comma arr[4]
         0,0,0,0,44,    temperature arr[5-8], comma arr[9]
         0,0,0,0,44,    battery value arr[10-13], commar arr[14]
         0 0 0 0,10};   raw px value arr[15-18], EOL arr[19]
    */
      
    if (CAL_PERFORMED)  
        pack_calibrated_px_val(px_val, temp_val, total_packet, 0);
    pack_temperature_val(temp_val, total_packet, 5);
    // Convert battery saadc mV to real battery voltage from voltage divider
    batt_val = calculate_battvoltage_from_mv(validate_uint_range(batt_val));
    pack_mv_value(batt_val, total_packet, 10, 4);
    pack_mv_value(validate_uint_range(px_val), total_packet, 15, 4);  
}

/* Converts SAADC result to mV value using gain, prescaler and resolution values
 */
uint32_t saadc_result_to_mv(uint32_t saadc_result)
{
    float adc_denom     = 4096.0;
    float adc_ref_mv    = 600.0;
    float adc_prescale  = 5.0;
    float adc_res_in_mv = (((float)saadc_result*adc_ref_mv)/adc_denom) * adc_prescale;

    return (uint32_t)adc_res_in_mv;
}

/* Read saadc values for temperature, battery level, and px, then store as 
 * respective mV values in global variables.
 */
void read_saadc_for_regular_protocol(void) 
{
    int NUM_SAMPLES = 150;
    nrf_saadc_value_t temp_val = 0;
    ret_code_t err_code;
    uint32_t AVG_MV_VAL = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
      err_code = nrfx_saadc_sample_convert(0, &temp_val);
      APP_ERROR_CHECK(err_code);
      if (temp_val < 0) {
          AVG_MV_VAL += 0;
      } else {
          AVG_MV_VAL += saadc_result_to_mv(temp_val);
      }
      nrf_delay_us(10);
    }
    AVG_MV_VAL = AVG_MV_VAL / NUM_SAMPLES;
    // Assign averaged readings to the correct calibration point
    if(!PX_IS_READ){
      AVG_PX_VAL = AVG_MV_VAL;
      NRF_LOG_INFO("read px val, restarting: %d", AVG_PX_VAL);
      NRF_LOG_FLUSH();
      PX_IS_READ = true;
      restart_saadc();
    }
    else if (!(PX_IS_READ && BATTERY_IS_READ)){
      AVG_BATT_VAL = AVG_MV_VAL;
      NRF_LOG_INFO("read batt val, restarting: %d", AVG_BATT_VAL);
      NRF_LOG_FLUSH();
      BATTERY_IS_READ = true;
      restart_saadc();
    }
    else {
       AVG_TEMP_VAL = AVG_MV_VAL;
       NRF_LOG_INFO("read temp val, restarting: %d", AVG_TEMP_VAL);
       PX_IS_READ = false;
       BATTERY_IS_READ = false;
       disable_px_voltage_reading();
       advertising_start(false);
    }    
}


void saadc_blocking_callback(nrf_drv_saadc_evt_t const * p_event)
{
    // Don't need to do anything
}



void init_saadc_for_blocking_sample_conversion(nrf_saadc_channel_config_t channel_config)
{
    ret_code_t err_code;
    err_code = nrf_drv_saadc_init(NULL, saadc_blocking_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
}

/* Initialize saadc with appropriate analog input pin
 */
void saadc_init(void)
{
    nrf_saadc_input_t ANALOG_INPUT;
    // Change pin depending on global control boolean
    if (!PX_IS_READ) {
        ANALOG_INPUT = NRF_SAADC_INPUT_AIN2;
    }
    else if (!(PX_IS_READ && BATTERY_IS_READ)) {
        ANALOG_INPUT = NRF_SAADC_INPUT_AIN3;
    }
    else {
        ANALOG_INPUT = NRF_SAADC_INPUT_AIN1;        
    }

    nrf_saadc_channel_config_t channel_config =
            NRF_SAADC_CUSTOM_CHANNEL_CONFIG_SE(ANALOG_INPUT);
    
    init_saadc_for_blocking_sample_conversion(channel_config);
}


/* This function initializes and enables SAADC sampling
 */
void enable_px_voltage_reading(void)
{
    saadc_init();
    if (!CAL_MODE) 
        read_saadc_for_regular_protocol();
}


/* Function unitializes and disables SAADC sampling, restarts timer
 */
void disable_px_voltage_reading(void)
{
    NRF_LOG_INFO("\n*** Disabling px voltage reading ***\n\n");
    NRF_LOG_FLUSH();
    nrfx_saadc_uninit();
    NVIC_ClearPendingIRQ(SAADC_IRQn);
    while(nrfx_saadc_is_busy()) {}

    // *** DISABLE BIASING CIRCUITRY ***
    disable_isfet_circuit();
}

/* Callback for app timer. Eables ISFET circuit by driving LDO logic HIGH,
 * and calling function to init and prepare SAADC.
 */
void single_shot_timer_handler()
{
    // disable timer
    ret_code_t err_code;
    err_code = app_timer_stop(m_timer_id);
    APP_ERROR_CHECK(err_code);

    // Delay to ensure appropriate timing 
    enable_isfet_circuit();       
    // Slight delay for ISFET to achieve optimal ISFET reading
    nrf_delay_ms(100);              
    // Begin SAADC initialization/start

    /* * * * * * * * * * * * * * *
     *  UNCOMMENT TO SEND DATA
     */

    enable_px_voltage_reading();

    /*
     *  COMMENT TO NOT SEND DATA
     * * * * * * * * * * * * * * */
}

/* Timer handler used by app timer if disconnection delay is enabled
 */
void disconn_delay_timer_handler()
{
    // disable timer
    ret_code_t err_code;
    err_code = app_timer_stop(m_timer_disconn_delay);
    APP_ERROR_CHECK(err_code);

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
      // Disconnect from central
      err_code = sd_ble_gap_disconnect(m_conn_handle, 
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing the timer module.
 */
void timers_init(void)
{
    uint32_t err_code;
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                single_shot_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_timer_disconn_delay,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                disconn_delay_timer_handler);
    APP_ERROR_CHECK(err_code);
}

/* Creates bluetooth packet by converting mV values to real-world values (px, 
 * degrees C) where appropriate and packing into char array. Sends data to
 * central, including buffered data if applicable. If buffered data is present,
 * the most recent data will first be appended to the buffer and then the entire
 * buffer will be sent.
 */
void send_data_and_restart_timer()
{
    uint32_t err_code;
    
    // Send data normally if there is no buffered data
    if (TOTAL_DATA_IN_BUFFERS == 0){
        // Create packet
        create_bluetooth_packet(AVG_PX_VAL, AVG_BATT_VAL, 
                                AVG_TEMP_VAL, NULL, total_packet);
        // Send data
        do
          {
             err_code = ble_nus_data_send(&m_nus, total_packet, 
                                          &total_size, m_conn_handle);
             NRF_LOG_INFO("error code: %u", err_code);
             if ((err_code != NRF_ERROR_INVALID_STATE) &&
                 (err_code != NRF_ERROR_RESOURCES) &&
                 (err_code != NRF_ERROR_NOT_FOUND))
             {
                
                    APP_ERROR_CHECK(err_code);              
             }
          } while (err_code == NRF_ERROR_RESOURCES);
          
          if (DISCONN_DELAY) {
              // Delay before disconnecting from central
              err_code = app_timer_start(m_timer_disconn_delay, 
                                         APP_TIMER_TICKS(DISCONN_DELAY_MS), NULL);
              APP_ERROR_CHECK(err_code);
          }
          else {
              err_code = 
                  sd_ble_gap_disconnect(m_conn_handle, 
                                        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
              APP_ERROR_CHECK(err_code);
          }
    }
    // Add most recent data to buffer, then send all data in buffer
    else {
        add_data_to_buffers();
        create_and_send_buffer_primer_packet();
        SEND_BUFFERED_DATA = true;
    }

    
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    uint32_t err_code;

    NRF_LOG_INFO("INSIDE GATT EVT HANDLER");

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

/* Starts app timer used for lower power System ON sleep mode
 */
void init_and_start_app_timer()
{
    sd_ble_gap_adv_stop(m_conn_handle);
    ret_code_t err_code;

    err_code = app_timer_start(m_timer_id, APP_TIMER_TICKS(DATA_INTERVAL), NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("TIMER STARTED (single shot) \n");
    NRF_LOG_FLUSH();
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

/* Helper function for linreg */
inline static float sqr(float x) {
    return x*x;
}

/* Function for running linear regression on two and three point calibration data
 */
void linreg(int num, float x[], float y[])
{
    float sumx  = 0.0;             /* sum of x     */
    float sumx2 = 0.0;             /* sum of x**2  */
    float sumxy = 0.0;             /* sum of x * y */
    float sumy  = 0.0;             /* sum of y     */
    float sumy2 = 0.0;             /* sum of y**2  */

    for (int i=0;i<num;i++){ 
        sumx  += x[i];       
        sumx2 += sqr(x[i]);  
        sumxy += x[i] * y[i];
        sumy  += y[i];      
        sumy2 += sqr(y[i]); 
    } 

    float denom = (num * sumx2 - sqr(sumx));

    if (denom == 0) {
        // singular matrix. can't solve the problem.
        NRF_LOG_INFO("singular matrix, cannot solve regression\n");
    }

    MVAL_CALIBRATION = (num * sumxy  -  sumx * sumy) / denom;
    BVAL_CALIBRATION = (sumy * sumx2  -  sumx * sumxy) / denom;
    RVAL_CALIBRATION = (sumxy - sumx * sumy / num) /    
                        sqrt((sumx2 - sqr(sumx)/num) *
                       (sumy2 - sqr(sumy)/num));

    NRF_LOG_INFO("MVAL **CAL**: " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(MVAL_CALIBRATION));
    NRF_LOG_INFO("BVAL **CAL**: " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(BVAL_CALIBRATION));
    NRF_LOG_INFO("RVAL **CAL**: " NRF_LOG_FLOAT_MARKER " \n", NRF_LOG_FLOAT(RVAL_CALIBRATION));
}

/* Handler for FDS module events */
void my_fds_evt_handler(fds_evt_t const * const p_fds_evt)
{
    switch (p_fds_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_fds_evt->result != FDS_SUCCESS)
            {
                NRF_LOG_INFO("ERROR IN EVENT HANDLER\n");
                NRF_LOG_FLUSH();
            }
            break;
        case FDS_EVT_WRITE:
            if (p_fds_evt->result == FDS_SUCCESS)
            {
                write_flag=1;
            }
            break;
        default:
            break;
    }
}


static void fds_write(float value, uint16_t FILE_ID, uint16_t REC_KEY)
{
    fds_record_t       record;
    fds_record_desc_t  record_desc;

    // Set up record.
    record.file_id              = FILE_ID;
    record.key                 = REC_KEY;
    record.data.length_words   = 1;

    if(FILE_ID == MVAL_FILE_ID && REC_KEY == MVAL_REC_KEY){
      record.data.p_data = &MVAL_CALIBRATION;
    }
    else if(FILE_ID == BVAL_FILE_ID && REC_KEY == BVAL_REC_KEY){
      record.data.p_data = &BVAL_CALIBRATION;
    }
    else if(FILE_ID == RVAL_FILE_ID && REC_KEY == RVAL_REC_KEY){
      record.data.p_data = &RVAL_CALIBRATION;
    }
    else if(FILE_ID == CAL_DONE_FILE_ID && REC_KEY == CAL_DONE_REC_KEY) {
      record.data.p_data = &CAL_PERFORMED;
    }
                    
    ret_code_t ret = fds_record_write(&record_desc, &record);
    if (ret != FDS_SUCCESS)
    {
        NRF_LOG_INFO("ERROR WRITING TO FLASH\n");
    }
    NRF_LOG_INFO("SUCCESS WRITING TO FLASH\n");
    NRF_LOG_FLUSH();
}


static void fds_update(float value, uint16_t FILE_ID, uint16_t REC_KEY)
{
    fds_record_t       record;
    fds_record_desc_t  record_desc;
    fds_find_token_t    ftok ={0};

    // Set up record.
    record.file_id              = FILE_ID;
    record.key                 = REC_KEY;
    record.data.length_words   = 1;

    if(FILE_ID == MVAL_FILE_ID && REC_KEY == MVAL_REC_KEY){
      record.data.p_data = &MVAL_CALIBRATION;
      fds_record_find(MVAL_FILE_ID, MVAL_REC_KEY, &record_desc, &ftok);
    }
    else if(FILE_ID == BVAL_FILE_ID && REC_KEY == BVAL_REC_KEY){
      record.data.p_data = &BVAL_CALIBRATION;
      fds_record_find(BVAL_FILE_ID, BVAL_REC_KEY, &record_desc, &ftok);
    }
    else if(FILE_ID == RVAL_FILE_ID && REC_KEY == RVAL_REC_KEY){
      record.data.p_data = &RVAL_CALIBRATION;
      fds_record_find(RVAL_FILE_ID, RVAL_REC_KEY, &record_desc, &ftok);
    }
    else if(FILE_ID == CAL_DONE_FILE_ID && REC_KEY == CAL_DONE_REC_KEY) {
      record.data.p_data = &CAL_PERFORMED;
      fds_record_find(CAL_DONE_FILE_ID, CAL_DONE_REC_KEY, &record_desc, &ftok);
    }
                    
    ret_code_t ret = fds_record_update(&record_desc, &record);
    if (ret != FDS_SUCCESS)
    {
        NRF_LOG_INFO("ERROR WRITING UPDATE TO FLASH\n");
    }
    NRF_LOG_INFO("SUCCESS WRITING UPDATE TO FLASH");
    NRF_LOG_FLUSH();
}


float fds_read(uint16_t FILE_ID, uint16_t REC_KEY)
{
    fds_flash_record_t  flash_record;
    fds_record_desc_t  record_desc;
    fds_find_token_t    ftok ={0};   //Important, make sure to zero init the ftok token
    float *p_data;
    float data;
    uint32_t err_code;
    
    NRF_LOG_INFO("Start flash search... \r\n");
    // Loop until all records with the given key and file ID have been found.
    while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == FDS_SUCCESS)
    {
        err_code = fds_record_open(&record_desc, &flash_record);
        if ( err_code != FDS_SUCCESS)
        {
                NRF_LOG_INFO("COULD NOT FIND OR OPEN RECORD\n");	
                return 0.0;
        }

        p_data = (float *) flash_record.p_data;
        data = *p_data;
        for (uint8_t i=0;i<flash_record.p_header->length_words;i++)
        {
                NRF_LOG_INFO("Data read: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(data));
        }
        NRF_LOG_INFO("\r\n");
        // Close the record when done.
        err_code = fds_record_close(&record_desc);
        if (err_code != FDS_SUCCESS)
        {
                NRF_LOG_INFO("ERROR CLOSING RECORD\n");
        }
        NRF_LOG_FLUSH();
    }
    NRF_LOG_INFO("SUCCESS CLOSING RECORD\n");
    return data;
}


static void fds_find_and_delete(uint16_t FILE_ID, uint16_t REC_KEY)
{
    fds_record_desc_t  record_desc;
    fds_find_token_t    ftok;
	
    ftok.page=0;
    ftok.p_addr=NULL;
    // Loop and find records with same ID and rec key and mark them as deleted. 
    while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == FDS_SUCCESS)
    {
        fds_record_delete(&record_desc);
        NRF_LOG_INFO("Deleted record ID: %d \r\n",record_desc.record_id);
    }
    // call the garbage collector to empty them, don't need to do this all the time, 
    // this is just for demonstration
    ret_code_t ret = fds_gc();
    if (ret != FDS_SUCCESS)
    {
        NRF_LOG_INFO("ERROR DELETING RECORD\n");
    }
    NRF_LOG_INFO("RECORD DELETED SUCCESFULLY\n");
}

static void fds_init_helper(void)
{	
    ret_code_t ret = fds_register(my_fds_evt_handler);
    if (ret != FDS_SUCCESS)
    {
        NRF_LOG_INFO("ERROR, COULD NOT REGISTER FDS\n");
                    
    }
    ret = fds_init();
    if (ret != FDS_SUCCESS)
    {
        NRF_LOG_INFO("ERROR, COULD NOT INIT FDS\n");
    }
    
    NRF_LOG_INFO("FDS INIT\n");	
}

/* Check words used in fds after initialization, and if more than 4 (default)
 * words are used then read MVAL, BVAL and RVAL values stored in flash. Assign
 * stored values to global variables respectively
 */
static void check_calibration_state(void)
{
    fds_stat_t fds_info;
    fds_stat(&fds_info);
    NRF_LOG_INFO("open records: %u, words used: %u\n", fds_info.open_records, 
                                                       fds_info.words_used);
    // fds_read will return 0 if the CAL_DONE record does not exist, 
    // or if the stored value is 0
    if(fds_read(CAL_DONE_FILE_ID, CAL_DONE_REC_KEY)) {
      CAL_PERFORMED = 1.0;
      NRF_LOG_INFO("Setting CAL_PERFORMED to true\n");
      // Read values stored in flash and set to respective global variables
      MVAL_CALIBRATION = fds_read(MVAL_FILE_ID, MVAL_REC_KEY);
      BVAL_CALIBRATION = fds_read(BVAL_FILE_ID, BVAL_REC_KEY);
      RVAL_CALIBRATION = fds_read(RVAL_FILE_ID, RVAL_REC_KEY);
      NRF_LOG_INFO("MVAL: " NRF_LOG_FLOAT_MARKER " \n", NRF_LOG_FLOAT(MVAL_CALIBRATION));
      NRF_LOG_INFO("BVAL: " NRF_LOG_FLOAT_MARKER " \n", NRF_LOG_FLOAT(BVAL_CALIBRATION));
      NRF_LOG_INFO("RVAL: " NRF_LOG_FLOAT_MARKER " \n", NRF_LOG_FLOAT(RVAL_CALIBRATION));
    }
}

/* If calibration has already been performed then update existing records with new 
 * values. If calibration has not already been performed, then write values to
 * new records
 */
void write_cal_values_to_flash(void) 
{
    // Update the existing flash records
    if (CAL_PERFORMED) {
        fds_update(MVAL_CALIBRATION, MVAL_FILE_ID,     MVAL_REC_KEY);
        fds_update(BVAL_CALIBRATION, BVAL_FILE_ID,     BVAL_REC_KEY);
        fds_update(RVAL_CALIBRATION, RVAL_FILE_ID,     RVAL_REC_KEY);
        fds_update(CAL_PERFORMED,    CAL_DONE_FILE_ID, CAL_DONE_REC_KEY);
    }
    // Write values to new records
    else {
        fds_write(MVAL_CALIBRATION, MVAL_FILE_ID,     MVAL_REC_KEY);
        fds_write(BVAL_CALIBRATION, BVAL_FILE_ID,     BVAL_REC_KEY);
        fds_write(RVAL_CALIBRATION, RVAL_FILE_ID,     RVAL_REC_KEY);
        fds_write(CAL_PERFORMED,    CAL_DONE_FILE_ID, CAL_DONE_REC_KEY);
    }
}

/* Sends the next data set stored in data buffer */
void send_next_packet_in_buffer()
{
     uint32_t err_code;
     send_buffered_data();
     PACK_CTR++;
     HVN_TX_EVT_COMPLETE = false;
     // Reset buffers, variables, and disconnect when finished 
     if (PACK_CTR == TOTAL_DATA_IN_BUFFERS) {
         reset_data_buffers();
         SEND_BUFFERED_DATA = false;
         if (DISCONN_DELAY) {
            // Delay before disconnecting from central
            err_code = app_timer_start(m_timer_disconn_delay, 
                                       APP_TIMER_TICKS(DISCONN_DELAY_MS), NULL);
            APP_ERROR_CHECK(err_code);
         }
         else {
             err_code = 
                  sd_ble_gap_disconnect(m_conn_handle, 
                                        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
             APP_ERROR_CHECK(err_code);
        }
      }
}

/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds = false;

    // Call function very first to turn on the chip
    turn_chip_power_on();

    log_init();
    power_management_init();

    // Initialize fds and check for calibration values
    fds_init_helper();
    check_calibration_state();

    // Continue with adjusted calibration state
    ble_stack_init();
    timers_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();

    // Init long-term data storage buffers
    init_data_buffers();
    
    // Start intermittent data reading <> advertising protocol
    enable_isfet_circuit();
    nrf_delay_ms(100);
    enable_px_voltage_reading();

    // Enter main loop for power management
    while (true)
    {
        idle_state_handle();
        // If data has been buffered then send accordingly, 
        // waiting for BLE_GATTS_EVT_HVN_TX_COMPLETE in 
        // between packets so the tx buffer is not filled
        if (SEND_BUFFERED_DATA) {
            while(HVN_TX_EVT_COMPLETE == false) { break; }
            send_next_packet_in_buffer();
        }
    } 
}

/*
 * @}
 */
