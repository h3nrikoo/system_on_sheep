from gmplot import gmplot
from collections import namedtuple
from math import pi, cos, radians, sin, atan2, sqrt

Coordinate = namedtuple("Cooridnate", ["latitude", "longitude"])
Measurement = namedtuple("Measurement", ["coordinate", "distance"])
init = Coordinate(63.40742, 10.47752)
gmap = gmplot.GoogleMapPlotter(init.latitude, init.longitude, 18, apikey="AIzaSyC2wE-bU54q_OK_1lLdk81gK4VglwUiCrU")



def degree_decimalminutes_to_degrees(latitude, longitude):
    lat_heading = 1 if latitude[0] == 'N' else -1
    long_heading = 1 if longitude[0] == 'E' else -1
    lat_deg = (int(latitude[1:3]) + float(latitude[3:10]) / 60) * lat_heading
    long_deg = (int(longitude[1:4]) + float(longitude[4:11]) / 60) * long_heading
    return (lat_deg, long_deg)


def draw_points():
    with open('data/raw-2/combined.csv') as file:
        current_position = (0, 0)
        for line in file.readlines():
            stripped = line.strip()
            values = line.split(";")
            if values[0] == "GPS":
                latitude, longitude = degree_decimalminutes_to_degrees(values[1], values[2])
                #gmap.circle(latitude, longitude, 2, color="red", face_alpha=0.1)
                current_position = (latitude, longitude)
            if values[0] == "RADIO":
                gmap.circle(current_position[0], current_position[1], 2)


def draw_dot(coordinates, size=2, color="red", alpha=0.3):
    gmap.circle(coordinates.latitude, coordinates.longitude, size, color=color, face_alpha=alpha)


def add_meters_to_coordinates(coordinates, dx, dy):
    lat = coordinates.latitude + (180/pi)*(dy/6371000)

    long = coordinates.longitude + (180/pi)*(dx/6371000)/cos(radians(coordinates.latitude))
    return Coordinate(lat, long)

def haversine(c1, c2):
    dlong = radians(c2.longitude - c1.longitude)
    dlat = radians(c2.latitude - c1.latitude)
    a = sin(dlat/2)**2 + cos(radians(c1.latitude)) * cos(radians(c2.latitude)) * sin(dlong/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    d = 6371000*c
    return d

measurements = [
    Measurement(add_meters_to_coordinates(init, 0, -230), 230),
    Measurement(add_meters_to_coordinates(init, 0, -634), 634),
    Measurement(add_meters_to_coordinates(init, 173, 0), 173),
]

draw_dot(init)
for i in range(-5, 6):
    for j in range(0, -21, -1):
        heatmap_coordinates = add_meters_to_coordinates(init, 50*i, 50*j)
        pr = 1
        for measurement in measurements:
            pr *= p(heatmap_coordinates, measurement.coordinate, measurement.distance)
        print(pr)
        draw_dot(heatmap_coordinates, color="blue", size=25, alpha=pr)

gmap.draw("my_map.html")