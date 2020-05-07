#ifndef SOS_LOG_H
#define SOS_LOG_H

#include "nrf.h"
#include "bsp.h"
#include <stdint.h>

#define SOS_LOG_SDC_SCK_PIN     NRF_GPIO_PIN_MAP(0, 29)
#define SOS_LOG_SDC_MOSI_PIN    NRF_GPIO_PIN_MAP(0, 30)
#define SOS_LOG_SDC_MISO_PIN    NRF_GPIO_PIN_MAP(0, 31)
#define SOS_LOG_SDC_CS_PIN      NRF_GPIO_PIN_MAP(0, 28)

#define SOS_LOG_MAX_RTTR_ELEMENTS 500

#define SOS_LOG_STATUS_OK 0
#define SOS_LOG_STATUS_FAILED 1

#define SOS_LOG_SAVE_STATUS_OK 0
#define SOS_LOG_SAVE_STATUS_FAILED 1

#define SOS_LOG_LATITUDE_LEN 10
#define SOS_LOG_LONGITUDE_LEN 11
#define SOS_LOG_TIME_LEN 10
#define SOS_LOG_DATE_LEN 6
#define SOS_LOG_SPEED_LEN 5
#define SOS_LOG_HEADING_LEN 5

typedef struct {
    char latitude[SOS_LOG_LATITUDE_LEN];
    char longitude[SOS_LOG_LONGITUDE_LEN];
    char time[SOS_LOG_TIME_LEN];
    char date[SOS_LOG_DATE_LEN];
    char speed[SOS_LOG_SPEED_LEN];
    char heading[SOS_LOG_HEADING_LEN];
} sos_gps_measurement_t;

typedef struct {
    uint8_t measurement_series;
    uint8_t tag_id;
    uint16_t series;
    uint16_t measurement;
    uint32_t distance;
    int32_t mean;
    int32_t variance;
    uint16_t success_count;
    uint16_t expected_count;
    char latitude[SOS_LOG_LATITUDE_LEN];
    char longitude[SOS_LOG_LONGITUDE_LEN];
} sos_rttr_measurement_t;

typedef struct {
    int current_log_id;
    bool save_flag;
    uint16_t n_rttr_measurements;
    sos_rttr_measurement_t rttr_measurements[SOS_LOG_MAX_RTTR_ELEMENTS];
    sos_gps_measurement_t last_gps_position;
    uint16_t current_measurement_series;
    bool last_time_set;
    char last_time[10];
} sos_log_logger_t;

int sos_log_rttr_measurement(sos_log_logger_t* logger, sos_rttr_measurement_t measurement);
int sos_log_nmea(sos_log_logger_t* logger, char* nmea, int nmea_length);
int sos_log_save(sos_log_logger_t* logger);
void sos_log_clear(sos_log_logger_t* logger);
sos_log_logger_t sos_log_init(void);
void sos_log_check_and_save(sos_log_logger_t* logger);
void sos_log_start_save(sos_log_logger_t* logger);
sos_gps_measurement_t* sos_get_last_gps_measurement(sos_log_logger_t* logger);
void sos_log_start_measurement_series(sos_log_logger_t* logger);
void sos_log_end_measurement_series(sos_log_logger_t* logger);
#endif //SOS_LOG_H