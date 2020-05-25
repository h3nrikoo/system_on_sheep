/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
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
/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "app_timer.h"
#include "ble_rttrs.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
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
#define ADVERTISING_INTERVAL_MS         5000
#define APP_COMPANY_IDENTIFIER          0xBAAA                                              /**< Identifier for sheep tags */
#define DEVICE_ID                       123                                                 /**< ID of specific tag that is advertised in an adv packet */
                                                                                            
#define APP_BLE_PHY                     BLE_GAP_PHY_CODED                                   /**< The primary PHY used for advertising/connections. */
#define APP_BLE_TX_POWER                3                                                   /**< The TX power in dBm used for advertising/connections. */

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(ADVERTISING_INTERVAL_MS, UNIT_0_625_MS)  /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED               /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)                    /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)                    /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                                   /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)                     /**< Connection supervisory time-out (4 seconds). */
                                                                                            
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                              /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                               /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                                   /**< Number of attempts before giving up the connection parameter negotiation. */
                                                                                            
#define APP_BLE_OBSERVER_PRIO           3                                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                                   /**< A tag identifying the SoftDevice BLE configuration. */
                                                                                            
#define RTTR_READY_LED                  BSP_BOARD_LED_2                                     /**< Is on when ranging is enabled but not yet started. */
#define RTTR_ONGOING_LED                BSP_BOARD_LED_3                                     /**< Is on while ranging is ongoing. */
                                                                                            
#define DEAD_BEEF                       0xDEADBEEF                                          /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


/*****************************************************************************
 * Static variables
 *****************************************************************************/

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */

RTTR_RESPONDER_HELPER_DEF(m_rttr, APP_BLE_OBSERVER_PRIO);


/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0
    }
};


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

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


static void services_init(void)
{
    ret_code_t err_code = rttr_helper_responder_services_init(&m_rttr);
    APP_ERROR_CHECK(err_code);
}


/*
 * BLE Advertising
 *****************************************************************************/

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t    err_code;
    static ble_advdata_t advdata;
    static ble_advdata_manuf_data_t manuf_specific_data;
    static uint8_t device_id[] = {DEVICE_ID}; 


    manuf_specific_data.company_identifier  = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = (uint8_t *) device_id;
    manuf_specific_data.data.size   = sizeof(device_id);


    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_NO_NAME;  
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.p_manuf_specific_data   = &manuf_specific_data;



    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = APP_BLE_PHY;
    adv_params.secondary_phy   = APP_BLE_PHY;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,
                                       m_adv_handle,
                                       APP_BLE_TX_POWER);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    bsp_indication_set(BSP_INDICATE_ADVERTISING);
}


static void advertising_start_if_no_ranging(void)
{
    if (!rttr_helper_ready(&m_rttr))
    {   
        for(int i = 0; i <= 0xFFFF; i++) {
            //nop
        }
        advertising_start();
    }
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


/*
 * BLE connection parameters module
 *****************************************************************************/

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
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
    ret_code_t             err_code;
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


/*
 * BLE general
 *****************************************************************************/

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN,
                                               m_conn_handle,
                                               APP_BLE_TX_POWER);
            APP_ERROR_CHECK(err_code);

            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);

            err_code = bsp_buttons_enable();
            APP_ERROR_CHECK(err_code);

            NRF_LOG_INFO("Connected");
            bsp_indication_set(BSP_INDICATE_CONNECTED);
            break;
        }

        case BLE_GAP_EVT_DISCONNECTED:
        {
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            err_code = bsp_buttons_disable();

            NRF_LOG_INFO("Disconnected");
            APP_ERROR_CHECK(err_code);
            advertising_start_if_no_ranging();
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
            break;
        }

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        }
        
        default:
        {
            // No implementation needed.
            break;
        }
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
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
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


static void ble_init(void)
{
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
}


static void ble_uninit(void)
{
    uint32_t err_code;

    err_code = nrf_sdh_disable_request();
    APP_ERROR_CHECK(err_code);
}


/*
 * Buttons/LEDs/Timers
 *****************************************************************************/

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] event         The button event.
 */
static void button_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_DISCONNECT:
        {
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                NRF_LOG_INFO("Manual disconnect.");
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                NRF_LOG_INFO("Manual disconnect failed: not in a connection.");
            }
            break;
        }

        default:
        {
            break;
        }
    }
}


void button_ble_error_handler(uint32_t nrf_error)
{
    APP_ERROR_CHECK(nrf_error);
}


/**@brief Function for initializing the button/LED handler module.
 *
 * @details Initializes all buttons and LEDs used by the application.
 */
static void buttons_leds_init(void)
{
    ret_code_t err_code;

    err_code = bsp_init(BSP_INIT_BUTTONS | BSP_INIT_LEDS, button_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(button_ble_error_handler, NULL);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timer_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/*
 * Miscellaneous
 *****************************************************************************/

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

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
            bsp_board_led_on(RTTR_READY_LED);
            NRF_LOG_INFO("RTT Ranging enabled!");
            break;
        }

        case RTTR_HELPER_EVT_DISABLED:
        {
            bsp_board_led_off(RTTR_READY_LED);
            NRF_LOG_INFO("RTT Ranging disabled!");
            break;
        }

        case RTTR_HELPER_EVT_STARTED:
        {
            bsp_board_led_off(RTTR_READY_LED);        
            bsp_board_led_on(RTTR_ONGOING_LED);
            break;
        }
        case RTTR_HELPER_EVT_FINISHED:
        {
            bsp_board_led_off(RTTR_ONGOING_LED);
            NRF_LOG_INFO("RTT Ranging finished!");
            advertising_start_if_no_ranging();
            break;
        }
        default:
        {
            // TODO: Handle failure events?
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
    if (!NRF_LOG_PROCESS())
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    timer_init();
    buttons_leds_init();
    power_management_init();
    qwr_init();
    ble_init();
    ranging_init();
    
    // Start execution.
    NRF_LOG_INFO("RTT Ranging Server started.");

    // Start advertising for peer
    advertising_start();

    // Enter main loop.
    for (;;)
    {
        rttr_helper_start_check(&m_rttr);
        idle_state_handle();
    }
}


/**
 * @}
 */
