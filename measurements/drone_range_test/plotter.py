import pandas as pd
from collections import namedtuple

def generate_dataset():
    Measurement = namedtuple("Measurement", ['distance', 'angle', 'tx_power', 'rssi'])

    measurements = []

    def m2d(m):
        return (m - 1) * 50

    def id2dir(id):
        if id == 10:
            return 0
        elif id == 11:
            return 90

    with open('data/raw/combined.csv') as file:
        for line in file.readlines():
            line = line.strip()
            values = line.split(";")
            if values[0] == 'RADIO':
                measurement = Measurement(m2d(int(values[2])), id2dir(int(values[1])), int(values[3]), int(values[4]))
                measurements.append(measurement)

    df = pd.DataFrame(measurements)
    df.to_csv('data/dataset.csv', index=False)

#generate_dataset()