#ifndef SOS_LOG_H
#define SOS_LOG_H

#include "nrf.h"
#include "bsp.h"
#include <stdint.h>

#define SOS_LOG_SDC_SCK_PIN     NRF_GPIO_PIN_MAP(0, 29)
#define SOS_LOG_SDC_MOSI_PIN    NRF_GPIO_PIN_MAP(0, 30)
#define SOS_LOG_SDC_MISO_PIN    NRF_GPIO_PIN_MAP(0, 31)
#define SOS_LOG_SDC_CS_PIN      NRF_GPIO_PIN_MAP(0, 28)

#define SOS_LOG_MAX_RADIO_ELEMENTS 4000
#define SOS_LOG_MAX_GPS_ELEMENTS 200

#define SOS_LOG_STATUS_OK 0
#define SOS_LOG_STATUS_FAILED 1

#define SOS_LOG_SAVE_STATUS_OK 0
#define SOS_LOG_SAVE_STATUS_FAILED 1

#define SOS_LOG_LATITUDE_LEN 10
#define SOS_LOG_LONGITUDE_LEN 11

typedef struct {
    uint16_t measurement_number;
    uint8_t tag_id;
    uint8_t measure_num;
    int8_t tx_power; 
    int8_t rssi;
} sos_radio_measurement_t;

typedef struct {
    uint16_t measurement_number;
    char latitude[SOS_LOG_LATITUDE_LEN];
    char longitude[SOS_LOG_LONGITUDE_LEN];
    char time[10];
    char date[6];
    char speed[5];
    char heading[5];
} sos_gps_measurement_t;

typedef struct {
    int current_log_id;
    bool save_flag;
    uint16_t n_radio_measurements;
    uint16_t n_gps_measurements;
    uint16_t n_total_measurements;
    sos_radio_measurement_t radio_measurements[SOS_LOG_MAX_RADIO_ELEMENTS];
    sos_gps_measurement_t gps_measurements[SOS_LOG_MAX_GPS_ELEMENTS];
    bool last_time_set;
    char last_time[10];
    char last_height_msl[6];
} sos_log_logger_t;

int sos_log_radio_measurement(sos_log_logger_t* logger, sos_radio_measurement_t measurement);
int sos_log_nmea(sos_log_logger_t* logger, char* nmea, int nmea_length);
int sos_log_save(sos_log_logger_t* logger);
void sos_log_clear(sos_log_logger_t* logger);
sos_log_logger_t sos_log_init(void);
void sos_log_check_and_save(sos_log_logger_t* logger);

#endif //SOS_LOG_H