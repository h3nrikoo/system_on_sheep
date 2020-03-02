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

#include "sos_log.h"


#define APP_BLE_CONN_CFG_TAG        1                                   /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO       3                                   /**< BLE observer priority of the application. There is no need to modify this value. */
#define SCAN_INTERVAL               MSEC_TO_UNITS(2000, UNIT_0_625_MS)   /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 MSEC_TO_UNITS(2000, UNIT_0_625_MS)   /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_DURATION               0x0000                              /**< Duration of the scanning in units of 10 milliseconds. If set to 0x0000, scanning will continue until it is explicitly disabled. */

static ble_gap_scan_params_t m_scan_param;                                /**< Scan parameters requested for scanning and connection. */
static uint8_t               m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_EXTENDED_MIN]; /**< buffer where advertising reports will be stored by the SoftDevice. */

/**@brief Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
static ble_data_t m_scan_buffer =
{
    m_scan_buffer_data,
    BLE_GAP_SCAN_BUFFER_EXTENDED_MIN
};


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


#define SHEEP_TAG_ID 0xFFAABA //0xFF signals manufacture ID is coming, which is set to 0xBAAA 

typedef enum {LOW_70CM = 0, HIGH_2M}M_HEIGHT; 
typedef enum {DEG90 = 0, DEG45, DEG0}M_ROTATION; 

static int8_t tx_power_dBm[] = {-20, -16, -12, -8, -4, 0, 3, 4};

typedef struct 
{
    int8_t TXpower; 
    int8_t rssi; 
    bool isCoded_phy;
    M_HEIGHT height; 
    M_ROTATION rotation; 
    uint16_t distance_m; 
}sheep_packet_t;

static sheep_packet_t sheep_info; //init 

typedef struct {
    bool isCoded_phy;
    bool isLogging; 
    uint16_t distance_m; 
}logger_state_t; 

logger_state_t logger_state = {false, false, 0}; //init 

static void print_state(logger_state_t state, sheep_packet_t info) {
    if(state.isLogging) {
         NRF_LOG_INFO("---LOGGING MODE---"); 
         NRF_LOG_INFO("TX RADIO POWER:\t\t%idBm", info.TXpower); 
         NRF_LOG_INFO("RSSI:\t\t\t\%idBm", info.rssi);
        if(info.height == HIGH_2M) {
            NRF_LOG_INFO("HEIGHT:\t\t\t2 m;");
        } else {
            NRF_LOG_INFO("HEIGHT:\t\t\t70 cm");
        }
        if(info.rotation == DEG90) {
            NRF_LOG_INFO("ROTATION:\t\t\t90 DEG");   
        } else if(sheep_info.rotation == DEG45){
            NRF_LOG_INFO("ROTATION:\t\t\t45 DEG");
        } else {
        NRF_LOG_INFO("ROTATION:\t\t\t0 DEG");
        }
        NRF_LOG_INFO("DISTANCE:\t\t\t%um", state.distance_m); 
        return; 
    } 

    char *str_sym; 

    if(state.isCoded_phy) {
        str_sym = "125 ksym/s";
    } else {
        str_sym = "1 Msym/s";
    }

    NRF_LOG_INFO("----INPUT MODE----"); 
    NRF_LOG_INFO("%s, %u m", (uint32_t)str_sym, state.distance_m);
    NRF_LOG_INFO("Btn1 toggle logger/input mode");
    NRF_LOG_INFO("Btn3 toggle 1 Msym/s and 125 ksym/s"); 
    NRF_LOG_INFO("Btn2 +10m, Btn4 -10m");
} 

static sos_data_logger_t sos_logger;

void log_sheepinfo() {
    sos_measurement_t measurement = {
        .tx_power = sheep_info.TXpower,
        .rssi = sheep_info.rssi,
        .height = sheep_info.height,
        .rotation = sheep_info.rotation
    };
    int err = sos_log_measurement(&sos_logger, measurement);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;
    ble_gap_evt_adv_report_t const * p_adv_report = &p_ble_evt->evt.gap_evt.params.adv_report;

    static uint32_t manufacture_id;
    static uint32_t counter = 0;
 
    if(p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT){

        manufacture_id = p_adv_report->data.p_data[4] << 16 | p_adv_report->data.p_data[5] << 8 | p_adv_report->data.p_data[6]; 
        if(manufacture_id == SHEEP_TAG_ID){

            //uint32_t time = app_timer_cnt_get(); 
            //uint32_t time_diff = app_timer_cnt_diff_compute(time, prev_time);   
            
            uint8_t tx_power_index = p_adv_report->data.p_data[7];
            sheep_info.TXpower = tx_power_dBm[tx_power_index]; 
            sheep_info.height = p_adv_report->data.p_data[8]; 
            sheep_info.rotation = p_adv_report->data.p_data[9]; 
            sheep_info.rssi = p_adv_report->rssi; 
            sheep_info.isCoded_phy = logger_state.isCoded_phy; 
            sheep_info.distance_m = logger_state.distance_m; 
            log_sheepinfo();
            counter++;
            if (counter > 50) {
                print_state(logger_state, sheep_info);  
                counter = 0;
            }
        }  
        // Continue scanning.
        sd_ble_gap_scan_start(NULL, &m_scan_buffer);

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


/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function to start scanning. */
static void scan_start(bool coded_phy)
{
    ret_code_t ret;

    //coded phy: 
    //m_scan_param.extended = 1; 
    m_scan_param.extended       = coded_phy;

    m_scan_param.active         = 0; //dont't do scan req  
    m_scan_param.interval       = SCAN_INTERVAL;
    m_scan_param.window         = SCAN_WINDOW;
    m_scan_param.timeout        = SCAN_DURATION;
    m_scan_param.scan_phys      = coded_phy ? BLE_GAP_PHY_CODED : BLE_GAP_PHY_1MBPS;
    m_scan_param.filter_policy  = BLE_GAP_SCAN_FP_ACCEPT_ALL;

    ret = sd_ble_gap_scan_start(&m_scan_param, &m_scan_buffer);
    APP_ERROR_CHECK(ret);
}

void bsp_event_callback(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
        {   
            logger_state.isLogging = !logger_state.isLogging; 
            if(logger_state.isLogging) {
                scan_start(logger_state.isCoded_phy); 
                NRF_LOG_INFO("---LOGGING MODE---"); 
            } else {
                APP_ERROR_CHECK(sd_ble_gap_scan_stop());
                print_state(logger_state, sheep_info);
                sos_logger.save_flag = true;
            }
            break;
        }
        case BSP_EVENT_KEY_1:
        {   
            if(!logger_state.isLogging) {
                logger_state.distance_m += 10; 
                print_state(logger_state, sheep_info); 
            }
            break; 
        }
        case BSP_EVENT_KEY_2: 
        {
            if(!logger_state.isLogging) {
                logger_state.isCoded_phy = !logger_state.isCoded_phy;  
                print_state(logger_state, sheep_info); 
            } 
            break; 
        }
        case BSP_EVENT_KEY_3: 
        {   
            if(!logger_state.isLogging) {
                if(logger_state.distance_m > 0) {
                    logger_state.distance_m -= 10; 
                    print_state(logger_state, sheep_info); 
                }
            }
            
            break; 
        }
        default:
        {   
            //NOP
            break;
        }
    }
}


static void btns_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_callback);
    APP_ERROR_CHECK(err_code);
}

static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    ret_code_t ret;

    // Initialize.
    log_init();
    ble_stack_init();
    timer_init();
    btns_init();
    sos_logger = sos_init();

    NRF_LOG_INFO("----INPUT MODE----"); 
    NRF_LOG_INFO("1 Msym/s, 0 m");
    NRF_LOG_INFO("Btn1 toggle logger/input mode");
    NRF_LOG_INFO("Btn3 toggle 1 Msym/s and 125 ksym/s"); 
    NRF_LOG_INFO("Btn2 +10m, Btn4 -10m")


    // Enter main loop.
    for (;;)
    {
        NRF_LOG_PROCESS();
        sos_logger.is_coded_phy = logger_state.isCoded_phy;
        sos_logger.distance_m = logger_state.distance_m;
        check_and_save(&sos_logger);
    }
}
