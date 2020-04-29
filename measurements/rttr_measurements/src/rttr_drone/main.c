/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_serial.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"

#include "rttr_helper.h"
#include "sos_log.h"


/*****************************************************************************
 * Macros
 *****************************************************************************/

#define SCANNING_LED    BSP_BOARD_LED_1 //LED2
#define GPS_LED         BSP_BOARD_LED_2 //LED3
#define SD_CARD_LED     BSP_BOARD_LED_3 //LED4

#define MEASURE_TIME_MS 10100 //20ms * 100 packets * 5 tx_powers + 100ms 

#define APP_BLE_PHY                 BLE_GAP_PHY_CODED                   /**< The primary PHY used for scanning/connections. */
#define APP_BLE_CONN_CFG_TAG        1                                   /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO       3                                   /**< BLE observer priority of the application. There is no need to modify this value. */
#define SCAN_INTERVAL               MSEC_TO_UNITS(2000, UNIT_0_625_MS)   /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 MSEC_TO_UNITS(2000, UNIT_0_625_MS)   /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION               0x0000                              /**< Duration of the scanning in units of 10 milliseconds. If set to 0x0000, scanning will continue until it is explicitly disabled. */

#define OP_QUEUES_SIZE          3
#define APP_TIMER_PRESCALER     NRF_SERIAL_APP_TIMER_PRESCALER

#define SERIAL_FIFO_TX_SIZE 64
#define SERIAL_FIFO_RX_SIZE 64

#define SERIAL_BUFF_TX_SIZE 1
#define SERIAL_BUFF_RX_SIZE 32

#define RTT_SAMPLE_COUNT_MIN            32
#define RTT_SAMPLE_COUNT_MAX            1024

#define SHEEP_TAG_ID 0xFFAABA //0xFF signals manufacture ID is coming, which is set to 0xBAAA 

/*****************************************************************************
 * Type definitions
 *****************************************************************************/

typedef struct 
{
    uint8_t tag_id;
    int8_t TXpower;
    int8_t rssi;
    uint8_t measure_num;
}
tag_packet_t;

typedef struct
{
    bool isCoded_phy;
    bool isLogging;
}
logger_state_t;

typedef struct
{
    bool done;
    int32_t sample_buffer[RTT_SAMPLE_COUNT_MAX];
    uint32_t sample_count;
} 
rttr_state_t;


/*****************************************************************************
 * Static variables
 *****************************************************************************/

/*
 * BLE GATT/GAP variables
 *****************************************************************************/
BLE_DB_DISCOVERY_DEF(m_db_disc);
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static ble_gap_scan_params_t m_scan_param;                                /**< Scan parameters requested for scanning and connection. */
static uint8_t               m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_EXTENDED_MIN]; /**< buffer where advertising reports will be stored by the SoftDevice. */

//typedef enum {LOW_70CM = 0, HIGH_2M}M_HEIGHT; 
//typedef enum {DEG90 = 0, DEG45, DEG0}M_ROTATION; 

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;        /**< Connection handle of the current connection. */

static ble_gap_scan_params_t m_scan_params = {
#if APP_BLE_PHY == BLE_GAP_PHY_CODED
    .extended               = 0x01,
    .report_incomplete_evts = 0x00,
#else
    .extended               = 0x00,
    .report_incomplete_evts = 0x00,
#endif
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



/**@brief Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
static ble_data_t m_scan_buffer =
{
    m_scan_buffer_data,
    BLE_GAP_SCAN_BUFFER_EXTENDED_MIN
};

/*
 * RTTR/measurement variables
 *****************************************************************************/

RTTR_INITIATOR_HELPER_DEF(m_rttr, APP_BLE_OBSERVER_PRIO);
static rttr_state_t m_rttr_state;


static int8_t tx_power_dBm[] = {-8, -4, 0, 3, 4};

APP_TIMER_DEF(m_save_measurement_timer); //Timer for logging after a measurement 

static tag_packet_t packet_info; //init 

logger_state_t logger_state = {true, false}; //init 
static sos_log_logger_t sos_logger;

static bool isMeasuring = false; 

/*
 * Peripheral driver variables
 *****************************************************************************/

NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uarte1_drv_config,
                      ARDUINO_1_PIN, ARDUINO_0_PIN,
                      ARDUINO_2_PIN, ARDUINO_3_PIN,
                      NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_9600,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);
NRF_SERIAL_QUEUES_DEF(serial1_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);
NRF_SERIAL_BUFFERS_DEF(serial1_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);
NRF_SERIAL_CONFIG_DEF(serial1_config, NRF_SERIAL_MODE_DMA,
                      &serial1_queues, &serial1_buffs, serial_event_handler, NULL);
NRF_SERIAL_UART_DEF(serial1_uarte, 1);

/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void serial_init(void);
static char nmea[200] = "";
static int nmea_len = 0;
static void serial_event_handler(struct nrf_serial_s const * p_serial, nrf_serial_event_t event) {
    if (event == NRF_SERIAL_EVENT_RX_DATA) {
        char c[32];
        size_t read = 0;
        ret_code_t ret = nrf_serial_read(p_serial, &c, sizeof(char)*32, &read, 0);
        static char a;
        static char b;
        for (size_t i = 0; i < read; i++) {
            b = a;
            a = c[i];
            nmea[nmea_len] = a;
            nmea_len++;
            if (a == '\n' && b == '\r') {
                int log_status = sos_log_nmea(&sos_logger, nmea, nmea_len);
                if (log_status == SOS_LOG_STATUS_OK) {
                    bsp_board_led_invert(GPS_LED);
                }
                nmea_len = 0;
            }
        }
    } else if (event == NRF_SERIAL_EVENT_FIFO_ERR ) {
        NRF_LOG_ERROR("NRF_SERIAL_EVENT_FIFO_ERR\n");
    } else if (event == NRF_SERIAL_EVENT_DRV_ERR  ) {
        NRF_LOG_ERROR("NRF_SERIAL_EVENT_DRV_ERR\n");
        if (sos_logger.n_total_measurements > 0) {
            sos_logger.save_flag = true;
        } else if (!sos_logger.save_flag) {
            nrf_serial_uninit(p_serial);
            serial_init();
        }
    }
}




/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

//static void print_state(logger_state_t state, tag_packet_t info) {
//    if(state.isLogging) {
//         NRF_LOG_INFO("---LOGGING MODE---"); 
//         NRF_LOG_INFO("TX RADIO POWER:\t\t%idBm", info.TXpower); 
//         NRF_LOG_INFO("RSSI:\t\t\t\%idBm", info.rssi);
//        if(info.height == HIGH_2M) {
//            NRF_LOG_INFO("HEIGHT:\t\t\t2 m;");
//        } else {
//            NRF_LOG_INFO("HEIGHT:\t\t\t70 cm");
//        }
//        if(info.rotation == DEG90) {
//            NRF_LOG_INFO("ROTATION:\t\t\t90 DEG");   
//        } else if(packet_info.rotation == DEG45){
//            NRF_LOG_INFO("ROTATION:\t\t\t45 DEG");
//        } else {
//        NRF_LOG_INFO("ROTATION:\t\t\t0 DEG");
//        }
//        return; 
//    }
//    
//    char *str_sym; 
//
//    if(state.isCoded_phy) {
//        str_sym = "125 ksym/s";
//    } else {
//        str_sym = "1 Msym/s";
//    }
//
//    NRF_LOG_INFO("----INPUT MODE----"); 
//    NRF_LOG_INFO("%s", (uint32_t)str_sym);
//    NRF_LOG_INFO("Btn1 toggle logger/input mode");
//    NRF_LOG_INFO("Btn3 toggle 1 Msym/s and 125 ksym/s"); 
//} 


void log_sheepinfo()
{
    sos_radio_measurement_t measurement = {
        .tag_id = packet_info.tag_id,
        .tx_power = packet_info.TXpower,
        .rssi = packet_info.rssi,
        .measure_num = packet_info.measure_num
       // .height = packet_info.height,
       // .rotation = packet_info.rotation
    };
    int err = sos_log_radio_measurement(&sos_logger, measurement);
}


static inline uint32_t rttr_sample_count_next_get(uint32_t current_count)
{
    uint32_t next_count = current_count * 2;
    return next_count < RTT_SAMPLE_COUNT_MAX ? next_count RTT_SAMPLE_COUNT_MIN;
}


static void rttr_state_init(void)
{
    m_rttr_state.done = false;
    m_rttr_state.sample_count = RTT_SAMPLE_COUNT_MIN;
}


static bool rttr_state_transition(void)
{
    m_rttr_state.sample_count = rttr_sample_count_next_get(m_rttr_state.sample_count);
    m_rttr_state.done = m_rttr_state.sample_count == RTT_SAMPLE_COUNT_MIN;
    return m_rttr_state.done;
}


static inline bool rttr_state_done(void)
{
    return m_rttr_state.done;
}



/*
 * BLE GATT/services
 *****************************************************************************/

static void services_init(void)
{
    ret_code_t err_code = rttr_helper_initiator_services_init(&m_rttr, &m_ble_gatt_queue);
    APP_ERROR_CHECK(err_code);
}


/*
 * BLE Scanning
 *****************************************************************************/

static inline uint32_t manufacturer_id_get(ble_gap_evt_adv_report_t const * p_adv_report)
{
    return p_adv_report->data.p_data[4] << 16 | p_adv_report->data.p_data[5] << 8 | p_adv_report->data.p_data[6];
}


static void ble_adv_report_handle(ble_gap_evt_t const * p_gap_evt)
{
    ret_code_t err_code;
    ble_gap_evt_adv_report_t const * p_adv_report = &p_gap_evt->params.adv_report;

    uint32_t manufacturer_id;

    manufacturer_id = manufacturer_id_get(p_adv_report);
    if (manufacturer_id == SHEEP_TAG_ID)
    {
        //uint32_t time = app_timer_cnt_get(); 
        //uint32_t time_diff = app_timer_cnt_diff_compute(time, prev_time); 
        
        if (isMeasuring == false)
        {
            isMeasuring = true; 
            APP_ERROR_CHECK(app_timer_start(m_save_measurement_timer, APP_TIMER_TICKS(MEASURE_TIME_MS), NULL));
            bsp_board_led_on(SCANNING_LED); 
        }
        
        packet_info.tag_id = p_adv_report->data.p_data[7];
        uint8_t tx_power_index = p_adv_report->data.p_data[8];
        packet_info.TXpower = tx_power_dBm[tx_power_index]; 
        packet_info.measure_num = p_adv_report->data.p_data[9];
        packet_info.rssi = p_adv_report->rssi; 

        //NRF_LOG_INFO("ID: %i, measurement: %i, tx_power: %i", packet_info.tag_id, packet_info.measure_num, packet_info.TXpower); 

        log_sheepinfo();
    }  
    // Continue scanning.
    sd_ble_gap_scan_start(NULL, &m_scan_buffer);
}


/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_scan_start(&m_scan_param, &m_scan_buffer);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("SCANNING..."); 
}


static void scan_start_if_no_ranging(void)
{
    if (!rttr_helper_ready(&m_rttr))
    {
        scan_start();
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
            NRF_LOG_INFO("Connected.");

            m_conn_handle = p_gap_evt->conn_handle;
            err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN,
                                                m_conn_handle,
                                                APP_BLE_TX_POWER);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);

            // Update LEDs status,

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
                NRF_LOG_DEBUG("Connection request timed out.");
            }
            break;
        }

        case BLE_GAP_EVT_ADV_REPORT:
        {
            ble_adv_report_handle(p_gap_evt);
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

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(m_conn_handle,
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

void bsp_event_callback(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
        {   
            sos_logger.save_flag = true;
            break;
        }
        case BSP_EVENT_KEY_1:
        {   
            break; 
        }
        case BSP_EVENT_KEY_2: 
        {
//            if(!logger_state.isLogging) {
//                logger_state.isCoded_phy = !logger_state.isCoded_phy;  
//                print_state(logger_state, packet_info); 
//            } 
            break; 
        }
        case BSP_EVENT_KEY_3: 
        {   
            break; 
        }
        default:
        {   
            //NOP
            break;
        }
    }
}



/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

//static void btns_init(void)
//{
//    ret_code_t err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_callback);
//    APP_ERROR_CHECK(err_code);
//}

static void leds_n_btns_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_callback);
    APP_ERROR_CHECK(err_code);
}

static void serial_init(void)
{
    ret_code_t ret = nrf_serial_init(&serial1_uarte, &m_uarte1_drv_config, &serial1_config);
    APP_ERROR_CHECK(ret);
    //char c[] = "$PMTK314,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
    //(void)nrf_serial_write(&serial1_uarte, &c, sizeof(c), NULL, 1000);

}

static void save_measurement_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    
    isMeasuring = false;  
    bsp_board_led_off(SCANNING_LED);
    NRF_LOG_INFO("Measurement done"); 

    sos_logger.save_flag = true; 
}

static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_save_measurement_timer, APP_TIMER_MODE_SINGLE_SHOT, save_measurement_timer_handler);
    APP_ERROR_CHECK(err_code);
}

static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

static void power_init(void)
{
    ret_code_t err_code = nrf_drv_power_init(NULL);
    APP_ERROR_CHECK(err_code);
}


/*
 * RTT Ranging
 *****************************************************************************/

static void log_ranging_stats(int32_t * p_samples, uint32_t count, uint32_t expected_count)
{
    uint32_t err_code;
    rttr_stats_report_t report;

    if (count > 0)
    {
        err_code = rttr_stats_calculate(p_samples, count, &report);
        APP_ERROR_CHECK(err_code);

        // TODO: write to SD card
    }
    else
    {
        NRF_LOG_INFO("No RTT packets received!");
    }
}


static void rttr_helper_evt_handle(rttr_helper_t * p_helper,
                                   rttr_helper_evt_t * p_evt)
{
    uint32_t err_code;

    switch (p_evt->type)
    {
        case RTTR_HELPER_EVT_CONNECTED:
        {
            NRF_LOG_INFO("RTT Ranging connected!");
            if (!rttr_state_done())
            {
                err_code = rttr_helper_enable(&m_rttr,
                                              &m_rttr_state.sample_buffer[0],
                                              m_rttr_state.sample_count);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // Disconnect without enabling RTTR to tell the peer that the rttr series is done
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            break;
        }
        case RTTR_HELPER_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("RTT Ranging disconnected!");
            break;
        }
        case RTTR_HELPER_EVT_ENABLED:
        {
            // This is handled in main()
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
            NRF_LOG_INFO("RTT Ranging started!");
            break;
        }
        case RTTR_HELPER_EVT_FINISHED:
        {
            /*
            // TODO
            log_ranging_stats(p_evt->params.finished.p_samples,
                              p_evt->params.finished.count,
                              p_evt->params.finished.expected_count);
            */
            rttr_state_transition();
            NRF_LOG_INFO("RTT Ranging finished!");
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

int main(void)
{
    log_init();
    clock_init();
    power_init();
    nrf_drv_clock_lfclk_request(NULL);
    timer_init();
    serial_init();
    ble_stack_init();
    leds_n_btns_init();
    sos_logger = sos_log_init();
    scan_start_coded_phy(); 
    sos_logger.save_flag = true;

    // Enter main loop.
    for (;;)
    {
        NRF_LOG_PROCESS();
        rttr_helper_start_check(&m_rttr);
        sos_log_check_and_save(&sos_logger);
    }
}
