#include "sos_log.h"

#include "nrf.h"
#include "bsp.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
        APP_SDCARD_CONFIG(SOS_LOG_SDC_MOSI_PIN, SOS_LOG_SDC_MISO_PIN, SOS_LOG_SDC_SCK_PIN, SOS_LOG_SDC_CS_PIN)
    ),
    NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

void sos_log_clear(sos_log_logger_t* logger) {
    logger->n_rttr_measurements = 0;
}

int sos_log_rttr_measurement(sos_log_logger_t* logger, sos_rttr_measurement_t measurement) {
    if (logger->n_rttr_measurements < SOS_LOG_MAX_RTTR_ELEMENTS) {
        measurement.measurement_series = logger->current_measurement_series;
        strncpy(measurement.latitude, logger->last_gps_position.latitude, SOS_LOG_LATITUDE_LEN);
        strncpy(measurement.longitude, logger->last_gps_position.longitude, SOS_LOG_LONGITUDE_LEN);
        logger->rttr_measurements[logger->n_rttr_measurements] = measurement;
        logger->n_rttr_measurements++;
        if (logger->n_rttr_measurements > SOS_LOG_MAX_RTTR_ELEMENTS - 20) {
            sos_log_start_save(logger);
        }
        return SOS_LOG_STATUS_OK;
    }
    return SOS_LOG_STATUS_FAILED;
}

void sos_log_start_save(sos_log_logger_t* logger) {
    logger->save_flag = true;
}

sos_gps_measurement_t* sos_get_last_gps_measurement(sos_log_logger_t* logger) {
    return &logger->last_gps_position;
}

int sos_log_nmea(sos_log_logger_t* logger, char* nmea, int nmea_length) {
    if (memcmp(nmea, "$GPRMC", 6) == 0 && nmea_length > 2) {
        char* command = strtok(nmea, ",");
        char* time = strtok(NULL, ",");
        char* status = strtok(NULL, ",");
        if (status[0] == 'V') {
            return SOS_LOG_STATUS_FAILED;
        }
        char* latitude = strtok(NULL, ",");
        char* NS = strtok(NULL, ",");
        char* longitude = strtok(NULL, ",");
        char* EW = strtok(NULL, ",");
        char* speed = strtok(NULL, ",");
        char* heading = strtok(NULL, ",");
        char* date = strtok(NULL, ",");

        sos_gps_measurement_t* measurement = sos_get_last_gps_measurement(logger);

        measurement->latitude[0] = NS[0];
        memcpy(&measurement->latitude[1], latitude, 9);
        measurement->longitude[0] = EW[0];
        memcpy(&measurement->longitude[1], longitude, 10);
        memcpy(&measurement->time[0], time, 10);
        memcpy(&measurement->date[0], date, 6);
        memcpy(&measurement->speed[0], speed, 5);
        memcpy(&measurement->heading[0], heading, 5);
        memcpy(&logger->last_time[0], time, 10);
        logger->last_time_set = true;
        return SOS_LOG_STATUS_OK;
      }
}


sos_log_logger_t sos_log_init(void) {
    sos_log_logger_t logger = {
        .current_log_id = 1,
        .save_flag = false,
        .n_rttr_measurements = 0,
        .rttr_measurements = {},
        .current_measurement_series = 0,
        .last_time_set = false,
        .last_time = {0},
    };
    return logger;
}

int sos_log_save(sos_log_logger_t* logger) {
    NRF_LOG_INFO("Saving");
    FATFS fs;
    DIR dir;
    FILINFO fno;
    FIL file;
    FRESULT ff_result;
    uint32_t bytes_written;
    DSTATUS disk_state = STA_NOINIT;
   
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        NRF_LOG_INFO("Failed Disk Init");
        return SOS_LOG_SAVE_STATUS_FAILED;
    }

    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Failed Disk Mount");
        return SOS_LOG_SAVE_STATUS_FAILED;
    }

    char file_name[12];
    if (logger->last_time_set) {
        snprintf(file_name, 13, "%.6s.csv", logger->last_time);
    } else {
        snprintf(file_name, 13, "%05d.csv", logger->current_log_id);
    }
    
    NRF_LOG_INFO("%s", file_name);
    ff_result = f_open(&file, file_name, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Failed File Open");
        return SOS_LOG_SAVE_STATUS_FAILED;
    }

    for (uint16_t i = 0; i < logger->n_rttr_measurements; i++) {
        char log_buffer[200] = {0};

        sos_rttr_measurement_t measurement = logger->rttr_measurements[i];
        int length = snprintf(
            log_buffer,
            200,
            "%d;%d;%d;%d;%d;%d;%d;%d;%d;%.10s;%.11s\n",
            measurement.measurement_series,
            measurement.tag_id,
            measurement.series,
            measurement.measurement,
            measurement.distance,
            measurement.mean,
            measurement.variance,
            measurement.success_count,
            measurement.expected_count,
            measurement.latitude,
            measurement.longitude
        );
        ff_result = f_write(&file, log_buffer, length, (UINT *) &bytes_written);
        if (ff_result) {
            NRF_LOG_INFO("Write failed");
            return SOS_LOG_SAVE_STATUS_FAILED;
        }
    }
    (void) f_close(&file);
    return SOS_LOG_SAVE_STATUS_OK;
}

void sos_log_start_measurement_series(sos_log_logger_t* logger) {
}

void sos_log_end_measurement_series(sos_log_logger_t* logger) {
    logger->save_flag = true;
}

void sos_log_check_and_save(sos_log_logger_t* logger) {
    if (logger->save_flag) {
        //bsp_board_led_off(BSP_BOARD_LED_3);
        int status = sos_log_save(logger);
        sos_log_clear(logger);
        logger->save_flag = false;
        logger->current_log_id++;
        if (status == SOS_LOG_SAVE_STATUS_OK) {
            //bsp_board_led_on(BSP_BOARD_LED_3);
        }
    }
}