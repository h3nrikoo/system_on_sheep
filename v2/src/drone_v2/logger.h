#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdint.h>
#include <stdbool.h>

#define LOGGER_MAX_GPS_READINGS 128
#define LOGGER_MAX_TAG_READINGS 128

#define LOGGER_DATE_LEN 6
#define LOGGER_TIME_LEN 6
#define LOGGER_LATITUDE_LEN 10
#define LOGGER_LONGITUDE_LEN 11
#define LOGGER_ALTITUDE_LEN 6
#define LOGGER_SPEED_LEN 5
#define LOGGER_COURSE_LEN 5

typedef struct {
    int reading_number;
    char date[LOGGER_DATE_LEN];
    char time[LOGGER_TIME_LEN];
    char latitude[LOGGER_LATITUDE_LEN];
    char longitude[LOGGER_LONGITUDE_LEN];
    char altitude_msl[LOGGER_ALTITUDE_LEN];
    char speed[LOGGER_SPEED_LEN];
    char course[LOGGER_COURSE_LEN];
} m_gps_reading_t;

typedef struct {
    int reading_number;
    uint16_t tag_id;
    uint16_t gps_delay;
    int16_t rssi;
    float distance;
    float distance_msl;
    uint16_t distance_packets;
} m_tag_reading_t;

typedef struct {
    m_gps_reading_t gps_readings[LOGGER_MAX_GPS_READINGS];
    int num_gps_readings;
    m_tag_reading_t tag_readings[LOGGER_MAX_TAG_READINGS];
    int num_tag_readings;
    int total_readings;
    int search_number;
    bool save_flag;
} m_logger_t;


void logger_init();
void logger_start();
void logger_stop();
void logger_log_tag(m_tag_reading_t reading);
void logger_process();
#endif //LOGGER_H