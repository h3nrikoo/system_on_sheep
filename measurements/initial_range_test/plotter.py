import matplotlib.pyplot as plt
import os
from collections import namedtuple
import pandas as pd
import seaborn as sns
import math
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

df = pd.DataFrame(measurements)


def plot_angles(coded_phy):
    df2 = df.groupby(['distance', 'coded_phy', 'angle']).size().reset_index().rename(columns={0: 'records'})
    df2['records'] = df2['records'] / (100 * 8 * 2)
    plt.figure()
    ax = sns.barplot(palette='Paired', data=df2[df2['coded_phy'] == coded_phy], x='distance', y='records', hue='angle')
    ax.set(xlabel='Distance (m)', ylabel='Packet Ratio')
    ax.set_title("Angles")
    plt.legend(title="Angle", loc="upper right")
    plt.show()


def plot_0dbm():
    df2 = df[df["tx_power"] == 4]
    df2 = df2.groupby(['distance', 'coded_phy']).size().reset_index().rename(columns={0: 'records'})
    df2['records'] = df2['records'] / (100 * 1 * 2 * 3)
    plt.figure()
    ax = sns.barplot(palette='Paired', data=df2, x='distance', y='records', hue='coded_phy')
    ax.set(xlabel='Distance (m)', ylabel='Packet Ratio')
    ax.set_title("Coded Phy")
    plt.legend(title="Using Coded Phy", loc="upper right")
    plt.savefig("4dbm.svg")
    plt.show()


def friis(d):
    return 10 * math.log(((0.125/(4 * math.pi*d))**2), 10)


def plot_rssi():
    df2 = df[df["tx_power"] == 0]
    df2 = df2[df2["height"] == 200]
    df2 = df2[df2["angle"] == 90]
    df2 = df2.groupby(['distance'])['rssi'].mean().reset_index()
    plt.figure()
    ax = sns.lineplot(palette='Paired', data=df2, x='distance', y='rssi')
    ax.set(xlabel='Distance (m)', ylabel='RSSI')
    ax.set_title("RSSI")

    plt.plot(df2['distance'], [friis(d) for d in df2['distance']])

    plt.show()


plot_rssi()
