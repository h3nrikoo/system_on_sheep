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
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_rttrs_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "rttr.h"
#include "rttr_helper.h"
#include "rttr_util.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "logger.h"

/*****************************************************************************
 * Macros
 *****************************************************************************/

#define MAX_RTT_SAMPLE_COUNT            64                                 /**< RTT Ranging measurement count. */

#define APP_COMPANY_IDENTIFIER          0xFFAABA                            /**< Identifier for sheep tags */

#define APP_BLE_PHY                     BLE_GAP_PHY_CODED                   /**< The primary PHY used for scanning/connections. */
#define APP_BLE_TX_POWER                3                                   /**< The TX power in dBm used for advertising/connections. */

#define SCAN_INTERVAL                   MSEC_TO_UNITS(2000, UNIT_0_625_MS)  /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                     MSEC_TO_UNITS(2000, UNIT_0_625_MS)  /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION                   0x0000                              /**< Timeout when scanning. 0x0000 disables timeout. */
#define TRY_CONN_TIMEOUT                MSEC_TO_UNITS(1300, UNIT_10_MS)    /**< Timeout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(30, UNIT_1_25_MS)     /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY                   0                                   /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG            1                                   /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO           3                                   /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define RTTR_TOGGLE_BUTTON_EVENT        BSP_EVENT_KEY_2                     /**< Button that will configure and enable RTTR on the peer. */

#define RTTR_READY_LED                  BSP_BOARD_LED_2
#define RTTR_ONGOING_LED                BSP_BOARD_LED_3


/*****************************************************************************
 * Static variables
 *****************************************************************************/

NRF_BLE_SCAN_DEF(m_scan);                                       /**< Scanning module instance. */
NRF_BLE_GATT_DEF(m_gatt);                                       /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                /**< DB discovery module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

//static char const m_target_periph_name[] = "Sheep_RTTRS";       /**< Name of the device we try to connect to. This name is searched in the scan report data*/
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;        /**< Connection handle of the current connection. */

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

static ble_gap_conn_params_t m_conn_params = {
    .min_conn_interval = MIN_CONNECTION_INTERVAL,
    .max_conn_interval = MAX_CONNECTION_INTERVAL,
    .slave_latency = SLAVE_LATENCY,
    .conn_sup_timeout = SUPERVISION_TIMEOUT
};

RTTR_INITIATOR_HELPER_DEF(m_rttr, APP_BLE_OBSERVER_PRIO);
static int32_t m_rttr_samples[MAX_RTT_SAMPLE_COUNT];
static int8_t m_rttr_rssi_samples[MAX_RTT_SAMPLE_COUNT]; 
static uint16_t m_rttr_packet_count = MAX_RTT_SAMPLE_COUNT;
static uint8_t device_id; 

/**@brief Buffer where advertising reports will be stored by the SoftDevice. */
static uint8_t m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_EXTENDED_MIN];

/**@brief Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
static ble_data_t m_scan_buffer =
{
    m_scan_buffer_data,
    BLE_GAP_SCAN_BUFFER_EXTENDED_MIN
};


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
    ret_code_t err_code = rttr_helper_initiator_services_init(&m_rttr, &m_ble_gatt_queue);
    APP_ERROR_CHECK(err_code);
}


/*
 * BLE Scanning
 *****************************************************************************/

/**@brief Function for handling Scaning events.
 *
 * @param[in]   p_scan_evt   Scanning event.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
            break;
        }
        default:
        {
            break;
        }
    }
}


/**@brief Function to start scanning.
 */
static void scan_start(bool cont)
{   
    ret_code_t err_code;
  
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
    NRF_LOG_INFO("SCANNING..."); 
}


static void scan_start_if_no_ranging(void)
{
    if (!rttr_helper_ready(&m_rttr))
    {
        scan_start(false);  
    }
}

static inline uint32_t manufacturer_id_get(ble_gap_evt_adv_report_t const * p_adv_report)
{
    return p_adv_report->data.p_data[4] << 16 |
           p_adv_report->data.p_data[5] << 8 |
           p_adv_report->data.p_data[6];
}


static void ble_adv_report_handle(ble_gap_evt_t const * p_gap_evt)
{
    ret_code_t err_code;
    ble_gap_evt_adv_report_t const * p_adv_report = &p_gap_evt->params.adv_report;
    bool continue_scan = true;

    uint32_t manufacturer_id;
    manufacturer_id = manufacturer_id_get(p_adv_report);
    if (manufacturer_id == APP_COMPANY_IDENTIFIER)
    {
        device_id = p_adv_report->data.p_data[7];
        

        continue_scan = false; 
        m_scan_params.timeout = TRY_CONN_TIMEOUT; 
        err_code =  sd_ble_gap_connect(&p_adv_report->peer_addr,
                                           &m_scan_params,
                                           &m_conn_params,
                                           APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);
    }
    if(continue_scan)
    {
        scan_start(true); 
    }


}

/*
 * BLE Database Discovery
 *****************************************************************************/

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
    rttr_helper_on_db_disc_evt(&m_rttr, p_evt);
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
}


static void db_discovery_close(void)
{
    ret_code_t err_code;

    err_code = ble_db_discovery_close(&m_db_disc);
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

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        // discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
        {
            m_conn_handle = p_gap_evt->conn_handle;
            err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN,
                                               m_conn_handle,
                                               APP_BLE_TX_POWER);
            APP_ERROR_CHECK(err_code);


            err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_INFO("Connected.");
            bsp_indication_set(BSP_INDICATE_CONNECTED);
            break;
        }

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            NRF_LOG_INFO("Disconnected.");
            scan_start_if_no_ranging();
            break;
        }

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection request timed out.");
                scan_start_if_no_ranging(); 
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
            NRF_LOG_INFO("PHY update request.");
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
            NRF_LOG_INFO("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
        }

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            NRF_LOG_INFO("GATT Server Timeout.");
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                NRF_LOG_INFO("Manual disconnect.");
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
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
}


static void ble_uninit(void)
{
    uint32_t err_code;

    db_discovery_close();
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
        case RTTR_TOGGLE_BUTTON_EVENT:
        {
            if (!rttr_helper_ready(&m_rttr))
            {
                // Enable ranging
                err_code = rttr_helper_enable(&m_rttr,
                                              &m_rttr_samples[0],
                                              &m_rttr_rssi_samples[0],
                                              m_rttr_packet_count);
            }
            else
            {
                // Disable ranging
                err_code = rttr_helper_disable(&m_rttr);
            }

            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;
        }

        case BSP_EVENT_KEY_3:
        {
            logger_save();
            break;
        }

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
                NRF_LOG_ERROR("Manual disconnect failed: not in a connection.");
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


/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
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

static void log_ranging_stats(int32_t * p_samples, int8_t * p_rssi_samples, uint32_t count, uint32_t expected_count, uint8_t tag_id)
{
    uint32_t err_code;
    float ticks;
    float distance;
    rttr_stats_report_t report;

    m_tag_reading_t reading = {
        .tag_id = tag_id,
        .packet_count = count,
        .expected_packet_count = expected_count
    };

    if (count > 0)
    {  
        
        //old logger values - we really only need clk and rssi samples - rest is post processing 
        /*
        err_code = rttr_stats_calculate(p_samples, count, &report);
        APP_ERROR_CHECK(err_code);

        ticks = MAX(report.mean, 0.0f);
        distance = rttr_distance_calculate(ticks);

        NRF_LOG_INFO("Avg: " NRF_LOG_FLOAT_MARKER ", Var: " NRF_LOG_FLOAT_MARKER,
                     NRF_LOG_FLOAT(report.mean),
                     NRF_LOG_FLOAT(report.variance));
        NRF_LOG_INFO("Cnt: %d/%d", report.count, expected_count);
        NRF_LOG_INFO("Est. distance: " NRF_LOG_FLOAT_MARKER " m", NRF_LOG_FLOAT(distance));
        */
        
        
        //temp print clk and rssi samples (NRF_LOG buffer gets full, but all samples are there) 
        //FIXME : add logging of count, expected count,  
        for (int i = 0; i < count; i++)
        {
           reading.p_samples[i] = p_samples[i];
           reading.p_rssi_samples[i] = p_rssi_samples[i];
        }
    }
    else
    {
        NRF_LOG_INFO("No RTT packets received!");
    }
    
    logger_log_tag(reading);
}


static void rttr_helper_evt_handle(rttr_helper_t * p_helper,
                                   rttr_helper_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case RTTR_HELPER_EVT_CONNECTED:
        {
            ret_code_t err_code;
            //err_code = bsp_buttons_enable();
            err_code = rttr_helper_enable(&m_rttr,
                                              &m_rttr_samples[0],
                                              &m_rttr_rssi_samples[0],
                                              m_rttr_packet_count);
            APP_ERROR_CHECK(err_code);
            break;
        }
        case RTTR_HELPER_EVT_DISCONNECTED:
        {
            ret_code_t err_code;
            //err_code = bsp_buttons_disable();
            //APP_ERROR_CHECK(err_code);
            break;
        }
        case RTTR_HELPER_EVT_ENABLED:
        {   
            ret_code_t err_code;
            bsp_board_led_on(RTTR_READY_LED);
            NRF_LOG_INFO("RTT Ranging enabled!");
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
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
            log_ranging_stats(p_evt->params.finished.p_samples,
                              p_evt->params.finished.p_rssi_samples,
                              p_evt->params.finished.count,
                              p_evt->params.finished.expected_count,
                              device_id);
            scan_start_if_no_ranging();
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
    ret_code_t err_code;
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
 * @details Handle any pending log operation(s), then sleep until the next event occurs.
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
    ble_init();
    //scan_init();
    ranging_init();
    logger_init();

    // Start execution.
    NRF_LOG_INFO("RTT Ranging Client started.");

    // Start scanning for peer
    scan_start(false);

    // Enter main loop.
    for (;;)
    {
        rttr_helper_start_check(&m_rttr);
        idle_state_handle();
        logger_process();
    }
}
