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
#include "nrf_ble_qwr.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"

#include "rttr.h"
#include "rttr_helper.h"
#include "rttr_config.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


/*****************************************************************************
 * Macros
 *****************************************************************************/

#define SERIES_LED                      BSP_BOARD_LED_0 //LED1
#define IDLE_LED                        BSP_BOARD_LED_1 //LED2
#define RTTR_LED                        BSP_BOARD_LED_2 //LED3
#define TIMEOUT_LED                     BSP_BOARD_LED_3 //LED4

#define ID_LED_INDEX_0                  BSP_BOARD_LED_2 //LED3
#define ID_LED_INDEX_1                  BSP_BOARD_LED_0 //LED1

#define ADV_START_BSP_EVENT             BSP_EVENT_KEY_2 //BTN 3 
#define ID_INCREMENT_BSP_EVENT          BSP_EVENT_KEY_1
#define ID_DECREMENT_BSP_EVENT          BSP_EVENT_KEY_0

#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                  /**< BLE observer priority of the application. There is no need to modify this value. */

#define APP_BLE_PHY                     BLE_GAP_PHY_CODED                       /**< The primary PHY used for advertising/connections. */
#define APP_BLE_TX_POWER_DEFAULT        3                                  /**< The TX power in dBm used for advertising/connections. */
#define APP_ADV_INTERVAL                MSEC_TO_UNITS(20, UNIT_0_625_MS)                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                MSEC_TO_UNITS(10000, UNIT_10_MS)   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(500, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

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


#define TAG_ID_MIN                      0xA
#define TAG_ID_MAX                      0xD
#define TAG_ID_DEFAULT                  TAG_ID_MIN
#define TX_POWER_INDEX_DEFAULT          0 

/*****************************************************************************
 * Type definitions
 *****************************************************************************/

typedef struct
{
   uint8_t tag_id;
   //uint8_t tx_power_index; 
   uint8_t measure_num; 
}
custom_adv_param_t; 


/*****************************************************************************
 * Forward declarations
 *****************************************************************************/

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);


/*****************************************************************************
 * Static variables
 *****************************************************************************/

/*
 * BLE variables
 *****************************************************************************/

NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t                  m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;      /**< Advertising handle used to identify an advertising set. */
static uint8_t                  m_enc_advdata[2][BLE_GAP_ADV_SET_DATA_SIZE_MAX];    /**< Buffer for storing an encoded advertising set. */
static uint32_t                 m_enc_advdata_idx = 0;

static uint8_t m_beacon_info[] =                    /**< Information advertised by the Beacon. */
{
    TAG_ID_DEFAULT, 
    0
};

static custom_adv_param_t       m_custom_adv_param = {
    .tag_id = TAG_ID_DEFAULT,
    .measure_num = 0
};

static ble_advdata_t            m_advdata;
static ble_advdata_manuf_data_t m_manuf_specific_data;
static ble_gap_adv_params_t     m_adv_params;


/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_gap_adv_data =
{
    .adv_data =
    {
        .p_data = &m_enc_advdata[0][0],
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0
    }
};

//static int8_t tx_power_dBm[] = {-8, -4, 0, 3, 4}; //possible tx powers in dBm
static bool m_adv_ready = false;

/*
 * RTTR/measurement variables
 *****************************************************************************/

RTTR_RESPONDER_HELPER_DEF(m_rttr, APP_BLE_OBSERVER_PRIO);


/*****************************************************************************
 * Static functions
 *****************************************************************************/

/*
 * BLE GAP
 *****************************************************************************/

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/*
 * BLE GATT/services
 *****************************************************************************/

static void services_init(void)
{
    ret_code_t err_code = rttr_helper_responder_services_init(&m_rttr);
    APP_ERROR_CHECK(err_code);
}


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


/*
 * BLE advertising
 *****************************************************************************/

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    m_manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    m_manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    m_manuf_specific_data.data.size   = sizeof(m_beacon_info);

    // Build and set advertising data.
    memset(&m_advdata, 0, sizeof(m_advdata));

    m_advdata.name_type             = BLE_ADVDATA_NO_NAME;
    m_advdata.flags                 = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    m_advdata.p_manuf_specific_data = &m_manuf_specific_data;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.primary_phy        = APP_BLE_PHY;
    m_adv_params.secondary_phy      = APP_BLE_PHY;
#if APP_BLE_PHY == BLE_GAP_PHY_CODED
    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED;
#else
    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_NONSCANNABLE_UNDIRECTED;
#endif
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = APP_ADV_INTERVAL;
    m_adv_params.duration        = APP_ADV_DURATION;
}


static inline void advdata_index_next(void)
{
    m_enc_advdata_idx = (m_enc_advdata_idx + 1) & 0x1;
    m_gap_adv_data.adv_data.p_data = &m_enc_advdata[m_enc_advdata_idx][0];
}


static void advertising_start(custom_adv_param_t * p_param)
{
    ret_code_t err_code;

    m_adv_ready = false;

    m_beacon_info[0] = p_param->tag_id;
    m_beacon_info[1] = p_param->measure_num;

    advdata_index_next();
    
    err_code = ble_advdata_encode(&m_advdata,
                                  m_gap_adv_data.adv_data.p_data,
                                  &m_gap_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_gap_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,
                                       m_adv_handle,
                                       APP_BLE_TX_POWER_DEFAULT);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    bsp_board_led_off(IDLE_LED);
    bsp_board_led_off(TIMEOUT_LED);
    //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


static void tag_id_leds_set(void)
{
#if 1
    switch (m_custom_adv_param.tag_id)
    {
        case 0xA:
        {
            bsp_board_led_off(ID_LED_INDEX_0); 
            bsp_board_led_off(ID_LED_INDEX_1);
            break; 
        }
        case 0xB:
        {
            bsp_board_led_on(ID_LED_INDEX_0); 
            bsp_board_led_off(ID_LED_INDEX_1);
            break; 
        }
        case 0xC:
        {
            bsp_board_led_off(ID_LED_INDEX_0); 
            bsp_board_led_on(ID_LED_INDEX_1);
            break;
        }
        case 0xD:
        { 
            bsp_board_led_on(ID_LED_INDEX_0); 
            bsp_board_led_on(ID_LED_INDEX_1);
            break;
        }
        default:
        {
            APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
        }
    }
#endif
}


static void tag_id_log(void)
{
    NRF_LOG_INFO("TAG ID: 0x%X", m_custom_adv_param.tag_id);
}


static inline void tag_id_increment(void)
{
    if (m_custom_adv_param.tag_id < 0xD)
    {
        m_custom_adv_param.tag_id++;
    }
}


static inline void tag_id_decrement(void)
{
    if (m_custom_adv_param.tag_id > 0xA)
    {
        m_custom_adv_param.tag_id--;
    }
}


static void timeout_handle(void)
{   
    //ret_code_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);
    bsp_board_led_off(SERIES_LED);
    bsp_board_led_on(IDLE_LED);
    bsp_board_led_on(TIMEOUT_LED);
    tag_id_leds_set();
    m_adv_ready = true;
}


/*
 * BLE Queued Write Module
 *****************************************************************************/

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


static void qwr_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
}


static void bsp_event_callback(bsp_event_t event)
{
    switch (event)
    {
        case ADV_START_BSP_EVENT:
        {
            if (m_adv_ready)
            {
                advertising_start(&m_custom_adv_param);
                bsp_board_led_off(ID_LED_INDEX_0);
                bsp_board_led_off(ID_LED_INDEX_1);
                NRF_LOG_INFO("RUNNING...")
            }
            else
            {
                NRF_LOG_INFO("Advertising busy.");
            }
            break;
        }
        case ID_INCREMENT_BSP_EVENT:
        {
            if (m_adv_ready)
            {
                tag_id_increment();
                tag_id_leds_set();
                tag_id_log();
            }
            break; 
        }
        case ID_DECREMENT_BSP_EVENT: 
        {
            if (m_adv_ready)
            {
                tag_id_decrement();
                tag_id_leds_set();
                tag_id_log();
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
#if 0
static void adv_restart_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    
    m_custom_adv_param.tx_power_index++;

    if (m_custom_adv_param.tx_power_index < sizeof(tx_power_dBm)/sizeof(int8_t))
    {
        advertising_start(m_custom_adv_param);
    }
    else
    {
        m_custom_adv_param.tx_power_index = 0; 
        m_custom_adv_param.measure_num++;
        NRF_LOG_INFO("ID %i, measurement %i, START? Btn1 to confirm",
                     m_custom_adv_param.tag_id,
                     m_custom_adv_param.measure_num);
        isTransmitting = false; 
        bsp_board_led_on(IDLE_LED);
    }
}
#endif

/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

#if 0
    err_code = app_timer_create(&m_adv_restart_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                adv_restart_timer_handler);
    APP_ERROR_CHECK(err_code);
#endif
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;
    static bool measurement_series_started = false; 

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN,
                                               m_conn_handle,
                                               APP_BLE_TX_POWER_DEFAULT);
            APP_ERROR_CHECK(err_code);

            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            //bsp_indication_set(BSP_INDICATE_IDLE);
            break;
        }

        case BLE_GAP_EVT_DISCONNECTED:
        {
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            
            if (p_ble_evt->evt.gap_evt.params.disconnected.reason == BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION)
            {
                if (!rttr_helper_ready(&m_rttr))
                {
                    m_adv_ready = true;
                    measurement_series_started = false; 
                    bsp_board_led_off(SERIES_LED);
                    bsp_board_led_on(IDLE_LED);
                    tag_id_leds_set();
                    m_custom_adv_param.measure_num++; 
                    NRF_LOG_INFO("Disconnected, RTTR disabled. (5 x Measurement Series completed)");
                }
                else
                {   
                    if(measurement_series_started == false)
                    {
                        measurement_series_started = true; 
                        bsp_board_led_on(SERIES_LED);
                    }

                    NRF_LOG_INFO("Disconnected, RTTR enabled.");
                }
            }
            else
            {
                NRF_LOG_INFO("Connection improperly closed, restarting advertising...");
                advertising_start(&m_custom_adv_param);
            }
            break;
        }

        case BLE_GAP_EVT_ADV_SET_TERMINATED:
        {
            timeout_handle();
            NRF_LOG_INFO("Advertising timed out.");
            if(measurement_series_started) 
            {
                bsp_board_led_off(SERIES_LED);
                measurement_series_started = false; 
                m_custom_adv_param.measure_num++; 
            }
            break;
        }

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        {
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;
        }

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
            break;
        }

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        {
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
        }

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            timeout_handle();
            break;
        }

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            timeout_handle();
            break;
        }
        
        default:
        {
            // No implementation needed.
            break;
        }
    }
}

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

static void ble_init(void)
{
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
}


static void ble_uninit(void)
{
    uint32_t err_code;

    err_code = nrf_sdh_disable_request();
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


/*
 * RTT Ranging
 *****************************************************************************/

static void rttr_helper_evt_handle(rttr_helper_t * p_helper,
                                   rttr_helper_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case RTTR_HELPER_EVT_ENABLED:
        {
            NRF_LOG_INFO("RTT Ranging enabled!");
            break;
        }

        case RTTR_HELPER_EVT_DISABLED:
        {
            NRF_LOG_INFO("RTT Ranging disabled!");
            break;
        }

        case RTTR_HELPER_EVT_STARTED:
        {
            bsp_board_led_on(RTTR_LED);
            break;
        }
        case RTTR_HELPER_EVT_FINISHED:
        {
            bsp_board_led_off(RTTR_LED);
            NRF_LOG_INFO("RTT Ranging finished!");
            if (p_evt->params.finished.count > 0)
            {   
                NRF_LOG_INFO("Packets recevied: %i",p_evt->params.finished.count)
                advertising_start(&m_custom_adv_param);
            }
            else
            {
                // No packets received is a sign that something is wrong - retry adv.
                NRF_LOG_INFO("No RTTR packets received, retry advertising...")
                advertising_start(&m_custom_adv_param);
            }
            break;
        }
        default:
        {
            break;
        }
    }
}


static void ranging_init(void)
{
    uint32_t err_code;
    rttr_helper_init_t rttr_init;

    rttr_init.phy_init = (rttr_radio_phy_t) APP_BLE_PHY;
    rttr_init.evt_cb = rttr_helper_evt_handle;
    rttr_init.ble_init_cb = ble_init;
    rttr_init.ble_uninit_cb = ble_uninit;

    err_code = rttr_helper_init(&m_rttr, &rttr_init);
    APP_ERROR_CHECK(err_code);
}


/*
 * Main loop
 *****************************************************************************/

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
    qwr_init();
    ble_init();
    ranging_init();

    m_adv_ready = true;

    bsp_board_led_on(IDLE_LED);
    NRF_LOG_INFO("RTTR Tag started. Press Button 3 to start advertising.");
    tag_id_log();

    // Enter main loop.
    while (1)
    {
        rttr_helper_start_check(&m_rttr);
        idle_state_handle();
    }
}