#ifndef APP_CONFIG_H__
#define APP_CONFIG_H__

#define NRF_LOG_BACKEND_RTT_ENABLED                 1
#define NRF_LOG_BACKEND_UART_ENABLED                0

#define NRFX_PPI_ENABLED                            1
#define PPI_ENABLED                                 1

#define NRF_SDH_BLE_PERIPHERAL_LINK_COUNT           0
#define NRF_SDH_BLE_CENTRAL_LINK_COUNT              2
#define NRF_SDH_BLE_TOTAL_LINK_COUNT                (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + \
                                                     NRF_SDH_BLE_CENTRAL_LINK_COUNT)

#ifdef DEBUG
#define RTTR_DEBUG
#endif
#define RTTR_HELPER_FEATURE_INITIATOR
#define BLE_RTTRS_C_BLE_OBSERVER_PRIO               2

// Logging does not work in SES without this define
#define NRF_FPRINTF_FLAG_AUTOMATIC_CR_ON_LF_ENABLED 0

#define APP_SDCARD_ENABLED 1

#define SPI_ENABLED 1
#define NRFX_SPI_ENABLED 1
#define NRFX_SPIM_ENABLED 1

#define SPI0_ENABLED 1
#define SPI0_USE_EASY_DMA 1

#define NRF_SERIAL_ENABLED 1
#define UART1_ENABLED 1

#endif
