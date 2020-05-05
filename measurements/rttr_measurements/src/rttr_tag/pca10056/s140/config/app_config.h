#ifndef APP_CONFIG_H__
#define APP_CONFIG_H__

#define NRF_LOG_BACKEND_RTT_ENABLED 1
#define NRF_LOG_BACKEND_UART_ENABLED 0

#define NRF_LOG_BUFSIZE 128

//#define NRFX_PPI_ENABLED 1
//#define PPI_ENABLED 1

#ifdef DEBUG
#define RTTR_DEBUG
#endif
#define RTTR_HELPER_FEATURE_RESPONDER
#define BLE_RTTRS_BLE_OBSERVER_PRIO 2

#define NRF_BLE_QWR_ENABLED 1
#define NRF_SDH_BLE_VS_UUID_COUNT 1
#define NRF_SDH_BLE_PERIPHERAL_LINK_COUNT 1
#define NRF_SDH_BLE_GATTS_ATTR_TAB_SIZE (248 + 128)

// Logging does not work in SES without this define
#define NRF_FPRINTF_FLAG_AUTOMATIC_CR_ON_LF_ENABLED 0

#endif
