#include "logger.h"
#include <stdio.h> 
#include <stdlib.h>
#include "nrf.h"
#include "bsp.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"
#include "nrf_log.h"
#include "nrf_serial.h"

static FATFS fs;
static FIL file;
static m_logger_t logger = {};

static void initialize_sd_card();
void initialize_search_number();
void save_log();
void log_gps(m_gps_reading_t reading);
void serial_event_handler(struct nrf_serial_s const * p_serial, nrf_serial_event_t event);
void serial_init();

/**
 *  Configure SD Card
 * */
#define SDC_SCK_PIN     NRF_GPIO_PIN_MAP(0, 29)  ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN    NRF_GPIO_PIN_MAP(0, 30)  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    NRF_GPIO_PIN_MAP(0, 31)  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN      NRF_GPIO_PIN_MAP(0, 28)  ///< SDC chip select (CS) pin.

NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

/**
 * Configure UART
 */

#define OP_QUEUES_SIZE          3
#define APP_TIMER_PRESCALER     NRF_SERIAL_APP_TIMER_PRESCALER

#define SERIAL_FIFO_TX_SIZE         64
#define SERIAL_FIFO_RX_SIZE         64

#define SERIAL_BUFF_TX_SIZE         1
#define SERIAL_BUFF_RX_SIZE         32
NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uarte1_drv_config,
                      ARDUINO_1_PIN, ARDUINO_0_PIN,
                      ARDUINO_2_PIN, ARDUINO_3_PIN,
                      NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_9600,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);
NRF_SERIAL_QUEUES_DEF(serial1_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);
NRF_SERIAL_BUFFERS_DEF(serial1_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);
NRF_SERIAL_CONFIG_DEF(serial1_config, NRF_SERIAL_MODE_DMA,
                      &serial1_queues, &serial1_buffs, serial_event_handler, NULL);
NRF_SERIAL_UART_DEF(serial1_uarte, 1);

#define LINE_BUFFER_LENGTH 16
#define LINE_BUFFER_LINE_LENGTH 128

typedef struct {
    char lines[LINE_BUFFER_LENGTH][LINE_BUFFER_LINE_LENGTH];
    int start_idx;
    int end_idx;
} m_line_buffer_t;

m_line_buffer_t line_buffer;

void initialize_line_buffer() {
    line_buffer.end_idx = 0;
    line_buffer.start_idx = 0;
    memset(&line_buffer.lines[0], 0, sizeof(line_buffer.lines));
}

void serial_event_handler(struct nrf_serial_s const * p_serial, nrf_serial_event_t event) {
    static int nmea_len = 0;
    switch (event)
    {
        case NRF_SERIAL_EVENT_RX_DATA:
        {
            char c[32];
            size_t read = 0;
            ret_code_t ret = nrf_serial_read(p_serial, &c, sizeof(char)*32, &read, 0);
            static char a;
            static char b;
            for (size_t i = 0; i < read; i++)
            {
                b = a;
                a = c[i];
                line_buffer.lines[line_buffer.end_idx][nmea_len] = a;
                nmea_len++;
                if (a == '\n' && b == '\r')
                {
                    line_buffer.end_idx = (line_buffer.end_idx + 1) % LINE_BUFFER_LENGTH;
                    nmea_len = 0;
                }
            }
            break;
        }
        case NRF_SERIAL_EVENT_FIFO_ERR:
        {
            NRF_LOG_ERROR("NRF_SERIAL_EVENT_FIFO_ERR\n");
            break;
        }
        case NRF_SERIAL_EVENT_DRV_ERR:
        {
            NRF_LOG_ERROR("NRF_SERIAL_EVENT_DRV_ERR\n");
            nrf_serial_uninit(p_serial);
            serial_init();
            break;
        }
        default:
        {
            break;
        }
    }
}

void serial_init() {
    ret_code_t ret = nrf_serial_init(&serial1_uarte, &m_uarte1_drv_config, &serial1_config);
    APP_ERROR_CHECK(ret);
}

void logger_init() {
    logger.num_gps_readings = 0;
    logger.num_tag_readings = 0;
    logger.total_readings = 0;
    logger.save_flag = false;
    initialize_sd_card();
    initialize_search_number();
    serial_init();
    initialize_line_buffer();
}

static void initialize_sd_card() {
    FRESULT ff_result;
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
        NRF_LOG_INFO("Disk initialization failed.");
        return;
    }
    
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed.");
        return;
    }

    NRF_LOG_INFO("Finish logger init.");
}

void initialize_search_number()
{
    static DIR dir;
    static FILINFO fno;
    FRESULT ff_result;
    ff_result = f_opendir(&dir, '/');
    if (ff_result) {
        NRF_LOG_INFO("Directory open failed.");
        return;
    }

    int max_number = 0;

    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            NRF_LOG_INFO("Directory read failed.");
            return;
        }

        if (fno.fname[0] && !(fno.fattrib & AM_DIR)) {
            int id = atoi(fno.fname);
            if (id >= max_number) max_number = id;
        }
    }
    while (fno.fname[0]);
    logger.search_number = max_number + 1;
}

void clear_log() {
    logger.num_gps_readings = 0;
    logger.num_tag_readings = 0;
    logger.total_readings = 0;
}

void save_log() {
    FRESULT ff_result;

    char filename[13];
    snprintf(filename, 13, "%.4d.csv", logger.search_number);
    
    ff_result = f_open(&file, filename, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Failed to open file.");
        return;
    }

    uint32_t bytes_written;
    char text_buffer[128] = "";
    int text_length;

    int gps_index = 0;
    int tag_index = 0;

    // Merge arrays of different readings
    for (int i = 0; i < logger.total_readings; i++) {

        // Next reading is a GPS reading
        if (gps_index < logger.num_gps_readings && logger.gps_readings[gps_index].reading_number == i) {
            m_gps_reading_t gps_reading = logger.gps_readings[gps_index];
            text_length = snprintf(text_buffer, 128, 
                "GPS;%.*s;%.*s;%.*s;%.*s;%.*s;%.*s;%.*s;%.*s;%.*s;%.*s\n",
                LOGGER_DATE_LEN, gps_reading.date,
                LOGGER_TIME_LEN, gps_reading.time,
                LOGGER_LATITUDE_LEN, gps_reading.latitude,
                LOGGER_LONGITUDE_LEN, gps_reading.longitude,
                LOGGER_ALTITUDE_LEN, gps_reading.altitude_msl,
                LOGGER_SPEED_LEN, gps_reading.speed,
                LOGGER_COURSE_LEN, gps_reading.course,
                LOGGER_HDOP_LEN, gps_reading.hdop,
                LOGGER_SATELLITES_LEN, gps_reading.satellites,
                LOGGER_GEODIAL_SEP_LEN, gps_reading.geodial_seperation
            );

            gps_index++;
        }

        // Next reading is a Tag reading
        if (tag_index < logger.num_tag_readings && logger.tag_readings[tag_index].reading_number == i) {
            m_tag_reading_t tag_reading = logger.tag_readings[tag_index];
            text_length = snprintf(text_buffer, 128, 
                "TAG;%d\n",
                tag_reading.tag_id
            );
            tag_index++;
        }

        ff_result = f_write(&file, text_buffer, text_length, &bytes_written);
        if (ff_result != FR_OK)
        {
            NRF_LOG_INFO("Failed file write.");
        }
    }
    clear_log();
    (void) f_close(&file);
}

void logger_log_tag(m_tag_reading_t reading) {
    if (logger.num_tag_readings < LOGGER_MAX_TAG_READINGS) {
        reading.reading_number = logger.total_readings;
        logger.tag_readings[logger.num_tag_readings] = reading;
        logger.num_tag_readings++;
        logger.total_readings++;

        if (logger.num_tag_readings > LOGGER_MAX_TAG_READINGS - 20) {
            logger.save_flag = true;
        }
    }
}

void log_gps(m_gps_reading_t reading) {
    if (logger.num_gps_readings < LOGGER_MAX_GPS_READINGS) {
        reading.reading_number = logger.total_readings;
        logger.gps_readings[logger.num_gps_readings] = reading;
        logger.num_gps_readings++;
        logger.total_readings++;

        if (logger.num_gps_readings > LOGGER_MAX_GPS_READINGS - 20) {
            logger.save_flag = true;
        }
    }
}

void parse_nmea(char* line) {
    static m_gps_reading_t reading = {};
    if (!memcmp(line, "$GPRMC", 6)) {
        char* command = strtok(line, ",");
        char* time = strtok(NULL, ",");
        char* status = strtok(NULL, ",");
        if (status[0] == 'V') {
            return;
        }
        char* latitude = strtok(NULL, ",");
        char* NS = strtok(NULL, ",");
        char* longitude = strtok(NULL, ",");
        char* EW = strtok(NULL, ",");
        char* speed = strtok(NULL, ",");
        char* course = strtok(NULL, ",");
        char* date = strtok(NULL, ",");
        strncpy(reading.date, date, LOGGER_DATE_LEN);
        strncpy(reading.time, time, LOGGER_TIME_LEN);
        strncpy(reading.latitude, NS, 1);
        strncpy(&reading.latitude[1], latitude, LOGGER_LATITUDE_LEN-1);
        strncpy(reading.longitude, EW, 1);
        strncpy(&reading.longitude[1], longitude, LOGGER_LONGITUDE_LEN-1);
        strncpy(reading.speed, speed, LOGGER_SPEED_LEN);
        strncpy(reading.course, course, LOGGER_COURSE_LEN);
        log_gps(reading);
        save_log();
    }

    if (!memcmp(line, "$GPGGA", 6)) {
        char* command = strtok(line, ",");
        char* time = strtok(NULL, ",");
        char* latitude = strtok(NULL, ",");
        char* NS = strtok(NULL, ",");
        char* longitude = strtok(NULL, ",");
        char* EW = strtok(NULL, ",");
        if (EW[0] =='M') {
            return;
        }
        char* fix = strtok(NULL, ",");
        char* satellites = strtok(NULL, ",");
        char* HDOP = strtok(NULL, ",");
        char* altitude_msl = strtok(NULL, ",");
        char* altitude_unit = strtok(NULL, ",");
        char* geodial_seperation = strtok(NULL, ",");
        char* geodial_seperation_unit = strtok(NULL, ",");
        strncpy(reading.altitude_msl, altitude_msl, LOGGER_ALTITUDE_LEN);
        strncpy(reading.hdop, HDOP, LOGGER_HDOP_LEN);
        strncpy(reading.satellites, satellites, LOGGER_SATELLITES_LEN);
        strncpy(reading.geodial_seperation, geodial_seperation, LOGGER_GEODIAL_SEP_LEN);
    }
}

void logger_process() {
    if (logger.save_flag) {
        save_log();
        logger.save_flag = false;
    }

    while (line_buffer.start_idx != line_buffer.end_idx) {
        parse_nmea(line_buffer.lines[line_buffer.start_idx]);
        memset(&line_buffer.lines[line_buffer.start_idx][0], 0, sizeof(line_buffer.lines[line_buffer.start_idx]));
        line_buffer.start_idx = (line_buffer.start_idx + 1) % LINE_BUFFER_LENGTH;
    }
}