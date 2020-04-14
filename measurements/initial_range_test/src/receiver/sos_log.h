#ifndef SOS_LOG_H
#define SOS_LOG_H

#include "nrf.h"
#include "bsp.h"
#include <stdint.h>

#define SDC_SCK_PIN     NRF_GPIO_PIN_MAP(0, 30)
#define SDC_MOSI_PIN    NRF_GPIO_PIN_MAP(0, 29)
#define SDC_MISO_PIN    NRF_GPIO_PIN_MAP(0, 28)
#define SDC_CS_PIN      NRF_GPIO_PIN_MAP(0, 31)

#define SOS_MAX_LOG_ELEMENTS 6000

#define SOS_LOG_STATUS_OK 0
#define SOS_LOG_STATUS_FULL 1

#define SOS_SAVE_STATUS_OK 0
#define SOS_SAVE_STATUS_FAILED 1

typedef struct {
    int8_t tx_power; 
    int8_t rssi; 
    int8_t height;
    int8_t rotation;
} sos_measurement_t;

typedef struct {
    int current_log_id;
    uint32_t n_measurements;
    bool save_flag;
    sos_measurement_t measurements[SOS_MAX_LOG_ELEMENTS];
    bool is_coded_phy;
    uint16_t distance_m;
} sos_data_logger_t;

int sos_log_measurement(sos_data_logger_t* logger, sos_measurement_t measurement);
int sos_save_log(sos_data_logger_t* logger);
void sos_clear_log(sos_data_logger_t* logger);
sos_data_logger_t sos_init();
void check_and_save(sos_data_logger_t* logger);

#endif //SOS_LOG_H