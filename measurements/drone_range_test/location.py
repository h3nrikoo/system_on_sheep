class Location:
    pass


class FRIISDistance:
    pass


class PathLossDistance:
    pass


class ShadowDistance:
    pass


class LinearRegressionDistance:
    pass


class LeastSquaresLocation:
    pass


class MaximumLikelihoodLocation:
    pass


class TrilaterationLocations:
    pass


from collections import namedtuple
import numpy as np
import pandas as pd
import math

Measurement = namedtuple("Measurement", ["latitude", "longitude", "altitude_msl", "tag_id", "rssi", "rttr"])
EARTH_RADIUS = 6361
tag_coordinates = (63.404805, 10.476199)
P = (63.403805, 10.476199)
R = (63.403805, 10.474199)

def ecef(latitude, longitude):
    x = EARTH_RADIUS * (np.cos(np.radians(latitude)) * np.cos(np.radians(longitude)))
    y = EARTH_RADIUS * (np.cos(np.radians(latitude)) * np.sin(np.radians(longitude)))
    z = EARTH_RADIUS * np.sin(np.radians(latitude))
    return x, y, z

def ecef_to_coordinates(x, y, z):
    latitude = np.degrees(np.arcsin(z / EARTH_RADIUS))
    longitude = np.degrees(np.arctan2(y, x))
    return latitude, longitude

def geodetic_to_enu(longitude, latitude):
    xp, yp, zp = ecef(P[0], P[1])
    xr, yr, zr = ecef(R[0], R[1])
    V = np.array([[xp-xr], [yp-yr], [zp-zr]])

    long = np.radians(longitude)
    lat = np.radians(latitude)
    enu = np.array([
        [-np.sin(long), np.cos(long), 0],
        [-np.sin(lat)*np.cos(long), -np.sin(lat)*np.sin(long), np.cos(lat)],
        [np.cos(lat)*np.cos(long), np.cos(lat)*np.sin(long), np.sin(lat)]
    ]).dot(V)
    return enu


measurements = [
    Measurement(63.406508, 10.476941, 261.2, 1, -78, 192.57),
    Measurement(63.403414, 10.477324, 261.2, 1, -78, 164.9),
    Measurement(63.404351, 10.474184, 261.2, 1, -78, 112.33),
]

df = pd.DataFrame(measurements)
print(geodetic_to_enu(df.latitude, df.longitude))