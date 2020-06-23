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
/**
 * @brief BLE RTT Ranging Service central and client application main file.
 *
 * This file contains the source code for a sample client application using the RTT Ranging service.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_db_discovery.h"
#include "ble_rttrs_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "nrf_atfifo.h"
#include "nrf_atflags.h"
#include "rttr.h"
#include "rttr_helper.h"
#include "rttr_util.h"
#include "rttr_config.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "logger.h"


/*****************************************************************************
 * Macros
 *****************************************************************************/

#define APP_BLE_PHY                     BLE_GAP_PHY_CODED                   /**< The primary PHY used for scanning/connections. */
#define APP_BLE_TX_POWER                3                                   /**< The TX power in dBm used for advertising/connections. */

#define SCAN_INTERVAL                   MSEC_TO_UNITS(500, UNIT_0_625_MS)     /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                     MSEC_TO_UNITS(500, UNIT_0_625_MS)     /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION                   0x0000                              /**< Timeout when scanning. 0x0000 disables timeout. */
#define TRY_CONN_TIMEOUT                MSEC_TO_UNITS(1300, UNIT_10_MS)


/**< Determines minimum connection interval in milliseconds. */
#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(RTTR_CONFIG_BLE_CONN_INTERVAL_MS, UNIT_1_25_MS)
/**< Determines maximum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(RTTR_CONFIG_BLE_CONN_INTERVAL_MS, UNIT_1_25_MS)
#define SLAVE_LATENCY                   0                                       /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Determines supervision time-out in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG            1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define RTTR_TOGGLE_BUTTON_EVENT        BSP_EVENT_KEY_2                     /**< Button that will configure and enable RTTR on the peer. */
#define RTTR_ONGOING_LED                BSP_BOARD_LED_2

#define MAX_RTT_SAMPLE_COUNT            64                                  /**< RTT Ranging measurement count. */
#define APP_COMPANY_IDENTIFIER          0xFFAABA                            /**< Identifier for sheep tags */

#define RTTR_HELPER_INSTANCES           NRF_SDH_BLE_CENTRAL_LINK_COUNT

#define RTTR_HELPER_INSTANCE_DEFINE(_index,...) RTTR_INITIATOR_HELPER_DEF(CONCAT_2(m_rttr,_index), APP_BLE_OBSERVER_PRIO);
#define RTTR_HELPERS_DEFINE(_count) MACRO_REPEAT_FOR(_count,RTTR_HELPER_INSTANCE_DEFINE)
#define RTTR_HELPER_INSTANCE_PTR_GET(_index,...) CONCAT_2(&m_rttr,_index),
#define RTTR_HELPERS_INIT_LIST(_count) { MACRO_REPEAT_FOR(_count,RTTR_HELPER_INSTANCE_PTR_GET) }

#define RTTR_RECURRING_INITIAL_INTERVAL APP_TIMER_TICKS(10000)
#define RTTR_RECURRING_INTERVAL         APP_TIMER_TICKS(2000)

/*****************************************************************************
 * Forward declarations
 *****************************************************************************/

static void rttr_recurring_timeout_handler(void * p_context);
static rttr_helper_t * conn_rttr_helper_assign(uint16_t conn_handle,
                                               const ble_gap_addr_t * p_peer_addr);
static void conn_rttr_helper_clear(uint16_t conn_handle);
static uint32_t rttr_queue_all(void);
static uint32_t rttr_start_queued(void);

/*****************************************************************************
 * Static variables
 *****************************************************************************/

/** GATT module instance. */
NRF_BLE_GATT_DEF(m_gatt);
/** DB discovery module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);
/** BLE GATT Queue instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static ble_gap_scan_params_t m_scan_params = {
    .extended               = 0x01,
    .report_incomplete_evts = 0x00,
    .active                 = 0x00,
    .filter_policy          = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .scan_phys              = APP_BLE_PHY,
    .interval               = SCAN_INTERVAL,
    .window                 = SCAN_WINDOW,
    .timeout                = SCAN_DURATION,
    .channel_mask           = {0}
};

/**@brief Buffer where advertising reports will be stored by the SoftDevice. */
static uint8_t              m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_EXTENDED_MIN];

/**@brief Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
static ble_data_t           m_scan_buffer =
{
    m_scan_buffer_data,
    BLE_GAP_SCAN_BUFFER_EXTENDED_MIN
};

static ble_gap_conn_params_t m_conn_params = {
    .min_conn_interval = MIN_CONNECTION_INTERVAL,
    .max_conn_interval = MAX_CONNECTION_INTERVAL,
    .slave_latency = SLAVE_LATENCY,
    .conn_sup_timeout = SUPERVISION_TIMEOUT
};

RTTR_HELPERS_DEFINE(NRF_SDH_BLE_CENTRAL_LINK_COUNT)

static rttr_helper_t * const    mp_rttr_list[NRF_SDH_BLE_CENTRAL_LINK_COUNT] = 
    RTTR_HELPERS_INIT_LIST(NRF_SDH_BLE_CENTRAL_LINK_COUNT);
static rttr_helper_t *          mp_conn_rttr[BLE_GAP_ROLE_COUNT_COMBINED_MAX] = { NULL };
static uint8_t                  m_device_ids[BLE_GAP_ROLE_COUNT_COMBINED_MAX] = { 0x00 };
static uint8_t                  m_pending_device_id;

enum __app_atflag_e
{
    APP_FLAG_RTTR_ACTIVE = 0,
    APP_FLAG_RTTR_RECURRING_ACTIVE,
    APP_FLAG_DB_DISCOVERY_ACTIVE,
    APP_FLAG_SCAN_ACTIVE,
    APP_FLAG_LOG_SAVE_ACTIVE,
    APP_FLAG_COUNT
};

static int32_t m_rttr_samples[MAX_RTT_SAMPLE_COUNT];
static uint16_t m_rttr_packet_count = MAX_RTT_SAMPLE_COUNT;

NRF_ATFIFO_DEF(m_db_discovery_fifo, uint16_t, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
NRF_ATFLAGS_DEF(m_app_flags, APP_FLAG_COUNT);

static ble_conn_state_user_flag_id_t m_conn_flag_ready = BLE_CONN_STATE_USER_FLAG_INVALID;
static ble_conn_state_user_flag_id_t m_conn_flag_queued = BLE_CONN_STATE_USER_FLAG_INVALID;

APP_TIMER_DEF(m_rttr_recurring_timer);


/*****************************************************************************
 * Static functions
 *****************************************************************************/

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
    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        ret_code_t err_code = rttr_helper_initiator_services_init(mp_rttr_list[i], &m_ble_gatt_queue);
        APP_ERROR_CHECK(err_code);
    }
}


/*
 * BLE Scanning
 *****************************************************************************/

/**@brief Function to start scanning.
 */
static void scan_start(bool cont)
{
    ret_code_t err_code;

    if (!nrf_atflags_fetch_set(m_app_flags, APP_FLAG_SCAN_ACTIVE))
    {
        err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT,
                                           BLE_CONN_HANDLE_INVALID,
                                           APP_BLE_TX_POWER);
        APP_ERROR_CHECK(err_code);

        m_scan_params.timeout = SCAN_DURATION;
    
        if (!cont)
        {
            err_code = sd_ble_gap_scan_start(&m_scan_params, &m_scan_buffer);
        }
        else
        {
            err_code = sd_ble_gap_scan_start(NULL, &m_scan_buffer);
        }
        APP_ERROR_CHECK(err_code);

        bsp_indication_set(BSP_INDICATE_SCANNING);
    }
}


static void scan_start_if_link_available(void)
{
    if (ble_conn_state_central_conn_count() < NRF_SDH_BLE_CENTRAL_LINK_COUNT)
    {
        scan_start(false);
    }
}


static void scan_stop(void)
{
    (void) sd_ble_gap_scan_stop();
    nrf_atflags_clear(m_app_flags, APP_FLAG_SCAN_ACTIVE);
}


static inline uint32_t manufacturer_id_get(ble_gap_evt_adv_report_t const * p_adv_report)
{
    if (p_adv_report->data.len >= 7)
    {
        return p_adv_report->data.p_data[4] << 16 |
               p_adv_report->data.p_data[5] << 8 |
               p_adv_report->data.p_data[6];
    }
    else
    {
        return 0;
    }
}


static void ble_adv_report_handle(ble_gap_evt_t const * p_gap_evt)
{
    ret_code_t err_code;
    ble_gap_evt_adv_report_t const * p_adv_report = &p_gap_evt->params.adv_report;
    bool continue_scan = true;
    uint32_t manufacturer_id;
    
    nrf_atflags_clear(m_app_flags, APP_FLAG_SCAN_ACTIVE);
    
    manufacturer_id = manufacturer_id_get(p_adv_report);
    if (manufacturer_id == APP_COMPANY_IDENTIFIER && p_adv_report->data.len >= 8)
    {
        m_pending_device_id = p_adv_report->data.p_data[7];
        continue_scan = false; 
        m_scan_params.timeout = TRY_CONN_TIMEOUT;
        err_code =  sd_ble_gap_connect(&p_adv_report->peer_addr,
                                       &m_scan_params,
                                       &m_conn_params,
                                       APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);
    }
    if (continue_scan)
    {
        scan_start(true); 
    }
}


/*
 * BLE Database Discovery
 *****************************************************************************/

static void db_discovery_next_start(void)
{
    uint32_t err_code;
    uint16_t conn_handle;

    if (!nrf_atflags_fetch_set(m_app_flags, APP_FLAG_DB_DISCOVERY_ACTIVE))
    {
        err_code = nrf_atfifo_get_free(m_db_discovery_fifo, &conn_handle, sizeof(uint16_t), NULL);
        if (err_code == NRF_SUCCESS)
        {
            err_code = ble_db_discovery_start(&m_db_disc, conn_handle);
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            nrf_atflags_clear(m_app_flags, APP_FLAG_DB_DISCOVERY_ACTIVE);
        }
    }
}


static void db_discovery_queue(uint16_t conn_handle)
{
    uint32_t err_code;
    bool visible;

    err_code = nrf_atfifo_alloc_put(m_db_discovery_fifo,
                                    &conn_handle,
                                    sizeof(uint16_t),
                                    &visible);
    APP_ERROR_CHECK(err_code);
    if (visible && !nrf_atflags_get(m_app_flags, APP_FLAG_DB_DISCOVERY_ACTIVE))
    {
        db_discovery_next_start();
    }
    // else: ???
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    uint32_t err_code;

    switch (p_evt->evt_type)
    {
        case BLE_DB_DISCOVERY_COMPLETE:
        {
            for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
            {
                if (rttr_helper_connected(mp_rttr_list[i]))
                {
                    rttr_helper_on_db_disc_evt(mp_rttr_list[i], p_evt);
                }
            }
            break;
        }
        case BLE_DB_DISCOVERY_ERROR:
        case BLE_DB_DISCOVERY_SRV_NOT_FOUND:
        {
            nrf_atflags_clear(m_app_flags, APP_FLAG_DB_DISCOVERY_ACTIVE);
            NRF_LOG_ERROR("Unable to discover services on connection %d. Disconnecting.",
                          p_evt->conn_handle);
            // TODO: Disconnect
            err_code = sd_ble_gap_disconnect(p_evt->conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        }
        case BLE_DB_DISCOVERY_AVAILABLE:
        {
            nrf_atflags_clear(m_app_flags, APP_FLAG_DB_DISCOVERY_ACTIVE);
            // Start any queued DB discovery operations
            db_discovery_next_start();
            break;
        }
        default:
        {
            break;
        }
    }
}


/**@brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(db_init));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);

    err_code = NRF_ATFIFO_INIT(m_db_discovery_fifo);
    APP_ERROR_CHECK(err_code);
}


static void db_discovery_close(void)
{
    ret_code_t err_code;

    err_code = ble_db_discovery_close(&m_db_disc);
    APP_ERROR_CHECK(err_code);
}


/*
 * BLE Connection State
 *****************************************************************************/

static void conn_state_init(void)
{
    ble_conn_state_init();

    m_conn_flag_ready = ble_conn_state_user_flag_acquire();
    if (m_conn_flag_ready == BLE_CONN_STATE_USER_FLAG_INVALID)
    {
        APP_ERROR_CHECK(NRF_ERROR_RESOURCES);
    }

    m_conn_flag_queued = ble_conn_state_user_flag_acquire();
    if (m_conn_flag_queued == BLE_CONN_STATE_USER_FLAG_INVALID)
    {
        APP_ERROR_CHECK(NRF_ERROR_RESOURCES);
    }
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

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        {
            nrf_atflags_clear(m_app_flags, APP_FLAG_SCAN_ACTIVE);

            err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN,
                                               p_gap_evt->conn_handle,
                                               APP_BLE_TX_POWER);
            APP_ERROR_CHECK(err_code);

            if (conn_rttr_helper_assign(p_gap_evt->conn_handle,
                                         &p_gap_evt->params.connected.peer_addr))
            {
                // Copy the device ID for this tag
                m_device_ids[ble_conn_state_conn_idx(p_gap_evt->conn_handle)] = m_pending_device_id;
            }
            else
            {
                NRF_LOG_ERROR("Unable to assign RTTR helper to connection.");
            }

            db_discovery_queue(p_gap_evt->conn_handle);

            NRF_LOG_INFO("Connected (handle: %d).", p_gap_evt->conn_handle);
            bsp_indication_set(BSP_INDICATE_CONNECTED);

            scan_start_if_link_available();
            break;
        }

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            conn_rttr_helper_clear(p_gap_evt->conn_handle);

            if (nrf_atflags_get(m_app_flags, APP_FLAG_DB_DISCOVERY_ACTIVE) && m_db_disc.conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                // HACK: no event is sent by the DB discovery module if disconnecting during DB discovery
                nrf_atflags_clear(m_app_flags, APP_FLAG_DB_DISCOVERY_ACTIVE);
            }

            NRF_LOG_INFO("Disconnected (handle: %d).", p_gap_evt->conn_handle);
            scan_start(false);
            break;
        }

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out (handle: %d).", p_gap_evt->conn_handle);
                scan_start(false);
            }
            break;
        }

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
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

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout (handle: %d).", p_ble_evt->evt.gattc_evt.conn_handle);
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        }

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout (handle: %d).", p_ble_evt->evt.gatts_evt.conn_handle);
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        }

        case BLE_GAP_EVT_ADV_REPORT:
        {
            ble_adv_report_handle(p_gap_evt);
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
 * @details Initializes the SoftDevice and the BLE event interrupts.
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
    gatt_init();
    db_discovery_init();
    services_init();
    conn_state_init();
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
        case RTTR_TOGGLE_BUTTON_EVENT:
        {
            // NRF_LOG_INFO("RTTR button pressed.");

            
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

    bsp_buttons_disable();
}


/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_rttr_recurring_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                rttr_recurring_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}


/*
 * Miscellaneous
 *****************************************************************************/

/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for initializing the log.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the Power manager. */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/*
 * RTT Ranging
 *****************************************************************************/

static void rttr_recurring_timeout_handler(void * p_context)
{
    uint32_t err_code;
    (void) p_context;

    if (!nrf_atflags_get(m_app_flags, APP_FLAG_LOG_SAVE_ACTIVE))
    {
        if (!nrf_atflags_get(m_app_flags, APP_FLAG_DB_DISCOVERY_ACTIVE))
        {
            if (!nrf_atflags_get(m_app_flags, APP_FLAG_RTTR_ACTIVE))
            {
                if (rttr_queue_all() == 0)
                {
                    NRF_LOG_ERROR("RTTR start failed: no connections were ready.");
                }
                (void) rttr_start_queued();
            }
            else
            {
                NRF_LOG_ERROR("RTTR start failed: already started.");
            }
        }
        else
        {
            NRF_LOG_INFO("RTTR start failed: DB discovery in progress.");
        }
    }
    else
    {
        NRF_LOG_INFO("RTTR start failed: log save in progress.");
    }

    err_code = app_timer_start(m_rttr_recurring_timer, RTTR_RECURRING_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


static void rttr_recurring_start(void)
{
    uint32_t err_code;

    err_code = app_timer_start(m_rttr_recurring_timer, RTTR_RECURRING_INITIAL_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Recurring RTTR starting in 10 seconds.");
}


static rttr_helper_t * conn_rttr_helper_assign(uint16_t conn_handle,
                                               const ble_gap_addr_t * p_peer_addr)
{
    uint16_t idx;
    rttr_helper_t * p_helper = NULL;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        if (!rttr_helper_connected(mp_rttr_list[i]))
        {
            p_helper = mp_rttr_list[i];
            break;
        }
    }

    if (p_helper == NULL)
    {
        return NULL;
    }

    idx = ble_conn_state_conn_idx(conn_handle);
    if (idx >= BLE_CONN_STATE_MAX_CONNECTIONS)
    {
        APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
        return NULL;
    }
    mp_conn_rttr[idx] = p_helper;

    rttr_helper_conn_assign(p_helper, conn_handle, p_peer_addr);

    return p_helper;
}


static void conn_rttr_helper_clear(uint16_t conn_handle)
{
    uint16_t idx;

    idx = ble_conn_state_conn_idx(conn_handle);
    if (idx < BLE_CONN_STATE_MAX_CONNECTIONS)
    {
        mp_conn_rttr[idx] = NULL;
    }
}


static rttr_helper_t * conn_rttr_helper_get(uint16_t conn_handle)
{
    uint16_t idx;

    idx = ble_conn_state_conn_idx(conn_handle);
    if (idx < BLE_CONN_STATE_MAX_CONNECTIONS)
    {
        return mp_conn_rttr[idx];
    }
    else
    {
        return NULL;
    }
}


static inline bool rttr_start(rttr_helper_t * p_helper)
{
    uint32_t err_code;
    bool success = false;

    if (!rttr_helper_ready(p_helper))
    {
        // Enable ranging
        scan_stop();
        err_code = rttr_helper_start(p_helper,
                                     &m_rttr_samples[0],
                                     m_rttr_packet_count);
        switch (err_code)
        {
            case NRF_SUCCESS:
            {
                success = true;
                NRF_LOG_INFO("Starting RTTR...");
                break;
            }
            case BLE_ERROR_INVALID_CONN_HANDLE:
            {
                NRF_LOG_ERROR("RTTR start failed: Invalid conn handle.");
                break;
            }
            case NRF_ERROR_INVALID_STATE:
            {
                NRF_LOG_ERROR("RTTR start failed: Invalid state.");
                break;
            }
            default:
            {
                APP_ERROR_CHECK(err_code);
                break;
            }
        }
    }
    else
    {
        NRF_LOG_ERROR("RTTR start failed: already started.");
    }

    if (!success)
    {
        scan_start_if_link_available();
    }

    return success;
}


static void rttr_start_on_link(uint16_t conn_handle, void * p_context)
{
    uint32_t err_code;
    rttr_helper_t * p_helper;

    if (!nrf_atflags_fetch_set(m_app_flags, APP_FLAG_RTTR_ACTIVE))
    {
        // Clear queued flag regardless of whether we are able to start.
        ble_conn_state_user_flag_set(conn_handle,
                                     m_conn_flag_queued,
                                     false);

        p_helper = conn_rttr_helper_get(conn_handle);
        if (p_helper == NULL)
        {
            APP_ERROR_CHECK(NRF_ERROR_NULL);
        }
        if (!rttr_start(p_helper))
        {
            nrf_atflags_clear(m_app_flags, APP_FLAG_RTTR_ACTIVE);
        }
    }
}


static void rttr_queue_on_link(uint16_t conn_handle, void * p_context)
{
    (void) p_context;
    ble_conn_state_user_flag_set(conn_handle,
                                 m_conn_flag_queued,
                                 true); 
}


static inline void rttr_set_ready(uint16_t conn_handle, bool ready)
{
    uint16_t idx;

    idx = ble_conn_state_conn_idx(conn_handle);
    ble_conn_state_user_flag_set(conn_handle,
                                 m_conn_flag_ready,
                                 ready);
}


static inline void rttr_set_queued(uint16_t conn_handle, bool queued)
{
    ble_conn_state_user_flag_set(conn_handle,
                                 m_conn_flag_queued,
                                 queued);
}


static uint32_t rttr_start_queued(void)
{
    // Try to start RTTR on each queued helper.
    return ble_conn_state_for_each_set_user_flag(m_conn_flag_queued,
                                                 rttr_start_on_link,
                                                 NULL);
}


static uint32_t rttr_queue_all(void)
{
    // Queue RTTR on each helper that is ready to start.
    return ble_conn_state_for_each_set_user_flag(m_conn_flag_ready,
                                                 rttr_queue_on_link,
                                                 NULL);
}


static void log_ranging_stats(uint8_t device_id, int32_t * p_samples, uint32_t count, uint32_t expected_count)
{
    uint32_t err_code;
    float ticks;
    float distance;
    rttr_stats_report_t report;
    tag_reading_entry_t tag_reading;

    if (count > 0)
    {
        err_code = rttr_stats_calculate(p_samples, count, &report);
        APP_ERROR_CHECK(err_code);

        ticks = MAX(report.mean, 0.0f);
        distance = rttr_distance_calculate(ticks);

        NRF_LOG_INFO("Avg: " NRF_LOG_FLOAT_MARKER ", Var: " NRF_LOG_FLOAT_MARKER,
                     NRF_LOG_FLOAT(report.mean),
                     NRF_LOG_FLOAT(report.variance));
        NRF_LOG_INFO("Cnt: %d/%d", report.count, expected_count);
        NRF_LOG_INFO("Est. distance: " NRF_LOG_FLOAT_MARKER " m", NRF_LOG_FLOAT(distance));

        tag_reading.tag_id = device_id;
        tag_reading.packet_count = count;
        tag_reading.expected_packet_count = expected_count;
        tag_reading.p_samples = p_samples;

        logger_log_tag(&tag_reading);
    }
    else
    {
        NRF_LOG_INFO("No RTT packets received!");
    }
}


static void rttr_helper_evt_handle(rttr_helper_t * p_helper,
                                   rttr_helper_evt_t * p_evt)
{
    bool finished = false;

    switch (p_evt->type)
    {
        case RTTR_HELPER_EVT_CONNECTED:
        {
            ret_code_t err_code;
            
            // Set flag to indicate that RTTR is available on the link
            // FIXME: should get conn_handle from event
            rttr_set_ready(p_evt->params.connected.conn_handle, true);

            NRF_LOG_INFO("RTTR ready (handle: %d).",
                         p_evt->params.connected.conn_handle);

            err_code = bsp_buttons_enable();
            APP_ERROR_CHECK(err_code);
            break;
        }
        case RTTR_HELPER_EVT_DISCONNECTED:
        {
            ret_code_t err_code;

            // Clear flags to indicate that RTTR is unavailable on the link
            // FIXME: should get conn_handle from event
            rttr_set_queued(p_evt->params.disconnected.conn_handle, false);
            rttr_set_ready(p_evt->params.disconnected.conn_handle, false);

            if (p_evt->params.disconnected.aborted)
            {
                nrf_atflags_clear(m_app_flags, APP_FLAG_RTTR_ACTIVE);
            }

            if (ble_conn_state_central_conn_count() == 0)
            {
                err_code = bsp_buttons_disable();
                APP_ERROR_CHECK(err_code);
            }
            break;
        }
        case RTTR_HELPER_EVT_RTTR_CHAR_WRITE:
        {
            NRF_LOG_INFO("RTTR characteristic write.");
            break;
        }
        case RTTR_HELPER_EVT_RTTR_CHAR_WRITE_FAILED:
        {
            nrf_atflags_clear(m_app_flags, APP_FLAG_RTTR_ACTIVE);
            finished = true;
            NRF_LOG_INFO("RTTR characteristic write failed.");
            break;
        }
        case RTTR_HELPER_EVT_STARTED:
        {
            bsp_board_led_on(RTTR_ONGOING_LED);
            NRF_LOG_INFO("RTTR started!");
            break;
        }
        case RTTR_HELPER_EVT_FINISHED:
        {
            bsp_board_led_off(RTTR_ONGOING_LED);
            NRF_LOG_INFO("RTTR finished!");
            uint8_t device_id = m_device_ids[ble_conn_state_conn_idx(p_helper->service.p_rttrs_c->conn_handle)];
            log_ranging_stats(device_id,
                              p_evt->params.finished.p_samples,
                              p_evt->params.finished.count,
                              p_evt->params.finished.expected_count);
            nrf_atflags_clear(m_app_flags, APP_FLAG_RTTR_ACTIVE);
            finished = true;
            break;
        }
        default:
        {
            break;
        }
    }

    // Continue RTTR on any queued instances
    if (!nrf_atflags_get(m_app_flags, APP_FLAG_RTTR_ACTIVE))
    {
        if (rttr_start_queued() == 0 && finished)
        {
            scan_start_if_link_available();
        }
    }
}


static void ranging_init(void)
{
    ret_code_t err_code;
    rttr_helper_init_t rttr_init;

    rttr_init.phy_init = (rttr_radio_phy_t) APP_BLE_PHY;
    rttr_init.evt_cb = rttr_helper_evt_handle;

    for (uint32_t i = 0; i < RTTR_HELPER_INSTANCES; i++)
    {
        err_code = rttr_helper_init(mp_rttr_list[i], &rttr_init);
        APP_ERROR_CHECK(err_code);
    }
}


/*
 * Main loop
 *****************************************************************************/

// FIXME: move
static void power_mode_init(void)
{
    uint32_t err_code;

    err_code = sd_power_mode_set(NRF_POWER_MODE_CONSTLAT);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (!NRF_LOG_PROCESS())
    {
        nrf_pwr_mgmt_run();
    }
}


static void rtt_log_process(void)
{
    if (!nrf_atflags_fetch_set(m_app_flags, APP_FLAG_LOG_SAVE_ACTIVE))
    {
        if (!nrf_atflags_get(m_app_flags, APP_FLAG_RTTR_ACTIVE) &&
            !nrf_atflags_get(m_app_flags, APP_FLAG_DB_DISCOVERY_ACTIVE))
        {
            logger_process();
        }
        nrf_atflags_clear(m_app_flags, APP_FLAG_LOG_SAVE_ACTIVE);
    }
}


/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    timer_init();
    clock_init();
    buttons_leds_init();
    power_management_init();
    ble_init();
    power_mode_init();
    ranging_init();
    logger_init();

    // Start execution.
    NRF_LOG_INFO("RTTR Client started.");

    // Start scanning for peer
    scan_start(false);
    rttr_recurring_start();

    // Enter main loop.
    for (;;)
    {
        rtt_log_process();
        idle_state_handle();
    }
}
