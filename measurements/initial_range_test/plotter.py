import matplotlib.pyplot as plt
import os
from collections import namedtuple
import numpy as np
import pandas as pd

DATA_DIRECTORY = "data/"

Measurement = namedtuple("Measurement", ['distance', 'coded_phy', 'tx_power', 'rssi', 'height', 'angle'])

files = []
for directory_item in os.listdir(DATA_DIRECTORY):
    if ".CSV" in directory_item:
        files.append(directory_item)

measurements = []
for file in files:
    if file[0] == "C":
        coded_phy = True
        distance = int(file[1:4])
    else:
        coded_phy = False
        distance = int(file[0:3])
    with open("{}{}".format(DATA_DIRECTORY, file)) as data_file:
        for line in data_file.readlines():
            if line[0] == "#":
                continue
            line = line.strip()
            data = line.split(";")
            tx_power = int(data[0])
            rssi = int(data[1])
            if data[2] == "0":
                height = 70
            else:
                height = 200
            if data[3] == "0":
                angle = 90
            elif data[3] == "1":
                angle = 45
            else:
                angle = 0

            measurements.append(Measurement(distance, coded_phy, tx_power, rssi, height, angle))

print(measurements)