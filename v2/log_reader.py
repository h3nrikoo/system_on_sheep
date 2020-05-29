from collections import namedtuple
from datetime import datetime
import folium
import webbrowser
import statistics
import random
import math
import numpy as np
import matplotlib.pyplot as plt
from geopy import distance
import pprint

center_coordinates = [63.406514, 10.476741]

CSV_TYPE = 0
CSV_TYPE_GPS = "GPS"
CSV_TYPE_TAG = "TAG"
CSV_GPS_DATE = 1
CSV_GPS_TIME = 2
CSV_GPS_LATITUDE = 3
CSV_GPS_LONGITUDE = 4
CSV_GPS_ALTITUDE = 5
CSV_GPS_GROUND_SPEED = 6
CSV_GPS_COURSE = 7
CSV_GPS_HDOP = 8
CSV_GPS_SATELLITES = 9
CSV_GPS_GEODIAL_SEPERATION = 10
CSV_TAG_TAG_ID = 1
CSV_TAG_GPS_DELAY = 2
CSV_TAG_PACKET_COUNT = 3
CSV_TAG_EXPECTED_PACKET_COUNT = 4
CSV_TAG_P_SAMPLES = 5
CSV_TAG_P_RSSI_SAMPLES = 6

GPSReading = namedtuple("GPSReading", ["datetime", "latitude", "longitude", "altitude_msl", "ground_speed", "course", "hdop", "satellites", "geodial_seperation"])
TagReading = namedtuple("TagReading", ["tag_id", "gps_delay", "packet_count", "expected_packet_count", "p_samples", "p_rssi_samples"])
LocationReading = namedtuple("LocationReading", ["tag_id", "distance", "latitude", "longitude", "altitude"])
Location = namedtuple("Location", ["latitude", "longitude", "altitude"])

def knots_to_meters_per_second(knots):
    return 0.5144*knots


def coordinates_degrees(latitude, longitude):
    lat_heading = 1 if latitude[0] == 'N' else -1
    long_heading = 1 if longitude[0] == 'E' else -1
    lat_deg = (int(latitude[1:3]) + float(latitude[3:10]) / 60) * lat_heading
    long_deg = (int(longitude[1:4]) + float(longitude[4:11]) / 60) * long_heading
    return lat_deg, long_deg


true_tag_lat_1, true_tag_long_1 = coordinates_degrees("N6324.2962", "E01028.6035")
true_tag_alt_msl_1 = 155.7+0.7
true_tag_lat_2, true_tag_long_2  = coordinates_degrees("N6324.3374", "E01028.5852")
true_tag_alt_msl_2 = 156.5+0.7

true_tag_locations = {
    123: Location(true_tag_lat_1, true_tag_long_1, true_tag_alt_msl_1),
    105: Location(true_tag_lat_1, true_tag_long_1, true_tag_alt_msl_1),
    137: Location(true_tag_lat_1, true_tag_long_1, true_tag_alt_msl_1),
    200: Location(true_tag_lat_1, true_tag_long_1, true_tag_alt_msl_1),
    109: Location(true_tag_lat_2, true_tag_long_2, true_tag_alt_msl_2),
    141: Location(true_tag_lat_2, true_tag_long_2, true_tag_alt_msl_2),
    154: Location(true_tag_lat_2, true_tag_long_2, true_tag_alt_msl_2),
    69: Location(true_tag_lat_2, true_tag_long_2, true_tag_alt_msl_2)
}
current_tag_id = 69
pprint.pprint(true_tag_locations)

class SearchLogReader:

    def _create_reading(self, values):
        type = values[CSV_TYPE]
        if type == CSV_TYPE_GPS:
            return self._create_GPSReading(values)
        if type == CSV_TYPE_TAG:
            return self._create_TagReading(values)

    def _create_GPSReading(self, values):
        date = values[CSV_GPS_DATE]
        day, month, year = int(date[0:2]), int(date[2:4]), int(date[4:6])+2000
        time = values[CSV_GPS_TIME]
        hour, minute, second = int(time[0:2]), int(time[2:4]), int(time[4:6])
        datetime_ = datetime(year, month, day, hour, minute, second)
        latitude, longitude = coordinates_degrees(values[CSV_GPS_LATITUDE], values[CSV_GPS_LONGITUDE])
        altitude = float(values[CSV_GPS_ALTITUDE])
        speed_mps = knots_to_meters_per_second(float(values[CSV_GPS_GROUND_SPEED]))
        course = float(values[CSV_GPS_COURSE])
        hdop = float(values[CSV_GPS_HDOP])
        satellites = int(values[CSV_GPS_SATELLITES])
        geodial_seperation = float(values[CSV_GPS_GEODIAL_SEPERATION])
        return GPSReading(datetime_, latitude, longitude, altitude, speed_mps, course, hdop, satellites, geodial_seperation)

    def _create_TagReading(self, values):
        tag_id = int(values[CSV_TAG_TAG_ID])
        gps_delay = int(values[CSV_TAG_GPS_DELAY])
        packet_count = int(values[CSV_TAG_PACKET_COUNT])
        expected_packet_count = int(values[CSV_TAG_EXPECTED_PACKET_COUNT])
        p_samples = [int(i) for i in values[CSV_TAG_P_SAMPLES].split(",")][0:packet_count]
        p_rssi_samples = [int(i) for i in values[CSV_TAG_P_RSSI_SAMPLES].split(",")][0:packet_count]

        return TagReading(tag_id, gps_delay, packet_count, expected_packet_count, p_samples, p_rssi_samples)

    def read(self, filename):
        with open(filename) as file:
            readings = []
            for line in file.readlines():
                line = line.strip()
                values = line.split(";")
                readings.append(self._create_reading(values))
            return SearchLog(readings)


class SearchLog:

    def __init__(self, readings):
        self.readings = readings
        self.location_readings = []

    def _generate_location_readings(self):
        for reading in self.readings:
            if isinstance(reading, GPSReading):
                latitude, longitude, altitude = reading.latitude, reading.longitude, reading.altitude_msl
            if isinstance(reading, TagReading):
                tag_id = reading.tag_id
                distance = math.sqrt((statistics.mean(reading.p_samples) * 9.37)**2 - (altitude-true_tag_locations[reading.tag_id].altitude)**2)
                self.location_readings.append(LocationReading(tag_id, distance, latitude, longitude, altitude))

    def get_location_readings(self):
        if len(self.location_readings) == 0:
            self._generate_location_readings()
        return self.location_readings

    def get_random_location_readings(self, n):

        return random.sample(self.get_location_readings(), min(n, len(self.location_readings)))

    def print(self):
        for readings in self.readings:
            print(readings)


class LaterationEstimator:

    def __init__(self, search_log):
        self.search_log = search_log

    def get_estimate(self):
        pass






def main():
    global current_tag_id
    current_tag_id = 69
    search_log = SearchLogReader().read("data/raw/0019.CSV")
    m = folium.Map(location=center_coordinates, zoom_start=16)
    folium.Marker(location=[true_tag_locations[current_tag_id].latitude, true_tag_locations[current_tag_id].longitude]).add_to(m)
    for reading in search_log.get_random_location_readings(6):
        folium.Circle(radius=reading.distance, location=[reading.latitude, reading.longitude], color="crimson", fill=False).add_to(m)
    m.save("map.html")
    webbrowser.open("map.html")


main()
