/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ADV_START_DELAY_MS 100

#define DEFAULT_TAG_ID 0xA 
#define ID_LED_INDEX_0 BSP_BOARD_LED_3 
#define ID_LED_INDEX_1 BSP_BOARD_LED_2 

#define IDLE_LED BSP_BOARD_LED_1 //LED2


#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                  /**< BLE observer priority of the application. There is no need to modify this value. */


#define ADV_INT_MS                      20
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(ADV_INT_MS, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define NUM_ADV_PACKETS                 100

#define ADV_TIMEOUT                     (ADV_INT_MS * (NUM_ADV_PACKETS + 0.7))/10


#define APP_BEACON_INFO_LENGTH          0x17                               /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
//#define APP_DEVICE_TYPE                 0x03                               /**< 0x02 refers to Beacon. */
//#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0xBAAA                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                         /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                         /**< Minor value used to identify Beacons. */

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                         /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[2][BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */
static uint32_t             m_advdata_index = 0;

APP_TIMER_DEF(m_adv_restart_timer); //Timer for restarting advertising 
APP_TIMER_DEF(m_adv_start_delay_timer); 

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata[0],
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};


static void advdata_index_next(void)
{
    m_advdata_index = (m_advdata_index + 1) & 0x1;
    m_adv_data.adv_data.p_data = m_enc_advdata[m_advdata_index];
}

typedef struct
{  
   uint8_t tag_id; 
   uint8_t tx_power_index; 
   uint8_t measure_num; 
}custom_adv_param_t; 

#define DEFAULT_TX_POWER_INDEX 0 

static int8_t tx_power_dBm[] = {-8, -4, 0, 3, 4}; //possible tx powers in dBm 

static custom_adv_param_t custom_adv_param = {DEFAULT_TAG_ID, DEFAULT_TX_POWER_INDEX, 0}; 

static uint8_t m_beacon_info[] =                    /**< Information advertised by the Beacon. */
{   
    DEFAULT_TAG_ID,
    DEFAULT_TX_POWER_INDEX, 
    0
};


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static ble_advdata_t advdata;
static ble_advdata_manuf_data_t manuf_specific_data;

static void advertising_init(void)
{
    uint32_t      err_code;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = sizeof(m_beacon_info);

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    
    //coded phy 
    m_adv_params.properties.type    = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED; 
    m_adv_params.primary_phy        = BLE_GAP_PHY_CODED; 
    m_adv_params.secondary_phy      = BLE_GAP_PHY_CODED;

    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = (uint16_t)ADV_TIMEOUT;  //  time out in 10ms units.

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
}


static void advertising_start(custom_adv_param_t param)
{   
    ret_code_t err_code;
     
    m_beacon_info[0] = param.tag_id; 
    m_beacon_info[1] = param.tx_power_index; 
    m_beacon_info[2] = param.measure_num; 

    advdata_index_next();

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, tx_power_dBm[param.tx_power_index]); 
    APP_ERROR_CHECK(err_code);
    
    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}

static void update_tag_id(bool increase) {
    if(increase) {
        if(custom_adv_param.tag_id < 0xD){
            custom_adv_param.tag_id++;
        }
    } else {
        if(custom_adv_param.tag_id > 0xA) {
            custom_adv_param.tag_id--; 
        }
    }
    switch(custom_adv_param.tag_id) {
        case 0xA: 
            bsp_board_led_off(ID_LED_INDEX_0); 
            bsp_board_led_off(ID_LED_INDEX_1);
            break; 
        case 0xB:
            bsp_board_led_on(ID_LED_INDEX_0); 
            bsp_board_led_off(ID_LED_INDEX_1);
            break; 
        case 0xC: 
            bsp_board_led_off(ID_LED_INDEX_0); 
            bsp_board_led_on(ID_LED_INDEX_1);
            break; 
        case 0xD: 
            bsp_board_led_on(ID_LED_INDEX_0); 
            bsp_board_led_on(ID_LED_INDEX_1);
            break; 
        default: 
            bsp_board_led_off(ID_LED_INDEX_0); 
            bsp_board_led_off(ID_LED_INDEX_1);
            break; 
     }

    NRF_LOG_INFO("ID %i, measurement %i, START? Btn1 to confirm", custom_adv_param.tag_id, custom_adv_param.measure_num);
}


static bool isTransmitting = false; //Flag to disable button press during transmission 

static void bsp_event_callback(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
        {   
            if(!isTransmitting){
                  isTransmitting = true; 
                  bsp_board_led_off(IDLE_LED);
                  //advertising_start(custom_adv_param);
                  //NRF_LOG_INFO("RUNNING...")
                  APP_ERROR_CHECK(app_timer_start(m_adv_start_delay_timer, APP_TIMER_TICKS(ADV_START_DELAY_MS), NULL));
                  
            }
            break; 
        }
        case BSP_EVENT_KEY_1:
        {
            if(!isTransmitting){
                update_tag_id(true); 
            } 
            break; 
        }
        case BSP_EVENT_KEY_2: 
        {
            //NOP

            break; 
        }
        case BSP_EVENT_KEY_3: 
        {
            if(!isTransmitting){
                update_tag_id(false); 
            }  
            break; 
        }
        default:
        {
            break;
        }
    }
}

/* Function for initializing LEDs and buttons. */
static void leds_n_btns_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_callback);
    APP_ERROR_CHECK(err_code);
}


//Runs automatically when the advertising duration is timed out 
static void adv_restart_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    
    custom_adv_param.tx_power_index++;

    if(custom_adv_param.tx_power_index < sizeof(tx_power_dBm)/sizeof(int8_t)) {
        advertising_start(custom_adv_param);
    } else {
        custom_adv_param.tx_power_index = 0; 
        custom_adv_param.measure_num++;
        NRF_LOG_INFO("ID %i, measurement %i, START? Btn1 to confirm", custom_adv_param.tag_id, custom_adv_param.measure_num);
        isTransmitting = false; 
        bsp_board_led_on(IDLE_LED);
    }
}

//Runs automatically when the delay after adv start is completed 
static void adv_start_delay_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    NRF_LOG_INFO("RUNNING...")
    advertising_start(custom_adv_param);
}


/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_adv_restart_timer, APP_TIMER_MODE_SINGLE_SHOT, adv_restart_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_adv_start_delay_timer, APP_TIMER_MODE_SINGLE_SHOT, adv_start_delay_timer_handler);
    APP_ERROR_CHECK(err_code);
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    if(p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_SET_TERMINATED) 
    {
       APP_ERROR_CHECK(app_timer_start(m_adv_restart_timer, APP_TIMER_TICKS(ADV_INT_MS), NULL));
    }
}

NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);


/* Initializes the SoftDevice and the BLE event interrupt */ 
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
}


/* Function for initializing logging. */
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
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{  

    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }

}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    timers_init();
    leds_n_btns_init();

    power_management_init();
    ble_stack_init();
    advertising_init();

    // Start execution.
    NRF_LOG_INFO("ID %i, measurement %i, START? Btn1 to confirm", custom_adv_param.tag_id, custom_adv_param.measure_num);

    bsp_board_led_on(IDLE_LED);

    // Enter main loop.
    while(1)
    {
        idle_state_handle();
    }
}