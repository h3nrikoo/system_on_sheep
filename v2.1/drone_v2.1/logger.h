#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdint.h>
#include <stdbool.h>

#define LOGGER_MAX_GPS_READINGS 64
#define LOGGER_MAX_TAG_READINGS 64

#define LOGGER_DATE_LEN 6
#define LOGGER_TIME_LEN 6
#define LOGGER_LATITUDE_LEN 10
#define LOGGER_LONGITUDE_LEN 11
#define LOGGER_ALTITUDE_LEN 6
#define LOGGER_SPEED_LEN 5
#define LOGGER_COURSE_LEN 5
#define LOGGER_HDOP_LEN 4
#define LOGGER_SATELLITES_LEN 2
#define LOGGER_GEODIAL_SEP_LEN 5

typedef struct {
    int reading_number;
    char date[LOGGER_DATE_LEN];
    char time[LOGGER_TIME_LEN];
    char latitude[LOGGER_LATITUDE_LEN];
    char longitude[LOGGER_LONGITUDE_LEN];
    char altitude_msl[LOGGER_ALTITUDE_LEN];
    char speed[LOGGER_SPEED_LEN];
    char course[LOGGER_COURSE_LEN];
    char hdop[LOGGER_HDOP_LEN];
    char satellites[LOGGER_SATELLITES_LEN];
    char geodial_seperation[LOGGER_GEODIAL_SEP_LEN];
} m_gps_reading_t;

typedef struct {
    int reading_number;
    uint8_t tag_id;
    uint16_t gps_delay;
    uint32_t packet_count;
    uint32_t expected_packet_count;
    int32_t p_samples[64];
    int8_t p_rssi_samples[64];
} m_tag_reading_t;


typedef struct
{
    uint8_t tag_id;
    uint32_t packet_count;
    uint32_t expected_packet_count;
    int32_t * p_samples;
}
tag_reading_entry_t;


typedef struct {
    m_gps_reading_t gps_readings[LOGGER_MAX_GPS_READINGS];
    int num_gps_readings;
    m_tag_reading_t tag_readings[LOGGER_MAX_TAG_READINGS];
    int num_tag_readings;
    int total_readings;
    int search_number;
    bool save_flag;
    uint32_t gps_time;
} m_logger_t;


void logger_init();
void logger_log_tag(tag_reading_entry_t * p_reading);
void logger_process();
void logger_save();
#endif //LOGGER_H