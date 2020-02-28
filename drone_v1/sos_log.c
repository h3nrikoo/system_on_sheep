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
        APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
    ),
    NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

void sos_clear_log(sos_data_logger_t* logger) {
    logger->n_measurements = 0;
}

int sos_log_measurement(sos_data_logger_t* logger, sos_measurement_t measurement) {
    if (logger->n_measurements < SOS_MAX_LOG_ELEMENTS) {
        logger->measurements[logger->n_measurements] = measurement;
        logger->n_measurements++;
        return SOS_LOG_STATUS_OK;
    }
    return SOS_LOG_STATUS_FULL;
}

int get_last_save_id() {
    return 0;
}

sos_data_logger_t sos_init() {
    sos_data_logger_t logger = {
        .current_log_id = get_last_save_id() + 1,
        .n_measurements = 0,
        .save_flag = false,
        .measurements = {}
    };

    return logger;
}

int sos_save_log(sos_data_logger_t* logger) {
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
        return SOS_SAVE_STATUS_FAILED;
    }

    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Failed Disk Mount");
        return SOS_SAVE_STATUS_FAILED;
    }

    char file_name[12];
    snprintf(file_name, 13, "S%02d%s%04d.csv", logger->current_log_id, logger->is_coded_phy ? "Y" : "N", logger->distance_m);
    NRF_LOG_INFO("%s", file_name);
    ff_result = f_open(&file, file_name, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Failed File Open");
        return SOS_SAVE_STATUS_FAILED;
    }

    char log_buffer[100] = "                                                                                                    ";
    int l = snprintf(
        log_buffer,
        100,
        "# Distance: %i m. Using coded phy: %s\n",
        logger->distance_m,
        logger->is_coded_phy ? "Yes" : "No"
    );

    ff_result = f_write(&file, log_buffer, l, (UINT *) &bytes_written);
    if (ff_result) {
        NRF_LOG_INFO("Write failed");
        return SOS_SAVE_STATUS_FAILED;
    }

    for (int i = 0; i < logger->n_measurements; i++) {
        char log_buffer[100] = "                                                                                                    ";
        sos_measurement_t measurement = logger->measurements[i];
        int length = snprintf(
            log_buffer, 
            100, 
            "%d;%d;%d;%d\n", 
            measurement.tx_power, 
            measurement.rssi, 
            measurement.height,
            measurement.rotation
            );
        ff_result = f_write(&file, log_buffer, length, (UINT *) &bytes_written);
        if (ff_result) {
            NRF_LOG_INFO("Write failed");
            return SOS_SAVE_STATUS_FAILED;
        }
    }
    (void) f_close(&file);
    return SOS_SAVE_STATUS_OK;
}

void check_and_save(sos_data_logger_t* logger) {
    if (logger->save_flag) {
        sos_save_log(logger);
        sos_clear_log(logger);
        logger->save_flag = false;
        logger->current_log_id++;
    }
}