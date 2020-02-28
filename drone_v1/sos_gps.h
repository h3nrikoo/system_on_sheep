#ifndef SOS_GPS_H
#define SOS_GPS_H

#include <stdint.h>

typedef struct {
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    int32_t direction;
    int32_t speed;
} sos_gps_values_t;

typedef struct {
    sos_gps_values_t current_values;
    
} sos_gps_t;

void sos_update_gps(sos_gps_t* gps, char* nmea);


#endif //SOS_GPS_H