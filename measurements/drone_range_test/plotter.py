import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
from collections import namedtuple
import math
import geopy.distance
pd.set_option('display.max_rows', 10000)

def generate_dataset_gps():
    # tx_coord = (63.4073927,10.4775050) #old 
    tx_coord = (63.40742, 10.47752) #ole sine koordinater 

    Measurement = namedtuple("Measurement", ['gps_dist','th_dist', 'angle', 'tx_power', 'rssi'])

    measurements = []

    def m2d(m):
        if m < 9:
            return math.floor(math.sqrt(((m - 2) * 50)**2 + 100**2))
        else:
            return math.floor(math.sqrt(((m - 3) * 50) ** 2 + 100 ** 2))

    def id2dir(id):
        if id == 10:
            return 90
        elif id == 11:
            return 0

    #convert degrees decimal minutes to decimal degrees 
    def dmm2dd(d, dm):
        m = math.floor(dm)
        s = (dm - m) * 60
        dd = float(d) + float(m)/60 + float(s)/(60*60)
        return dd   

    #convert logged coord to decimal degrees 
    def convert_coord(n_coord, e_coord): 
        n_coord = n_coord.replace("N","")
        d_n,dm_n = int(n_coord[:2]), float(n_coord[2:])
        e_coord = e_coord.replace("E","")
        d_e,dm_e = int(e_coord[:3]), float(e_coord[3:])
        return (dmm2dd(d_n, dm_n), dmm2dd(d_e, dm_e))

    curr_id = 0  
    measure_num = 0 
    gps_dist = 0 
    n_coord = 0
    e_coord = 0

    with open('data/raw-2/combined.csv') as file:
        for line in file.readlines():
            line = line.strip()
            values = line.split(";")

            if values[0] == 'GPS':
                n_coord = values[1]
                e_coord = values[2]
            elif values[0] == 'RADIO':
                if curr_id != values[1] or measure_num != values[2]:
                    curr_id = values[1]
                    measure_num = values[2]
                    coords = convert_coord(n_coord, e_coord)
                    gps_dist = math.floor(math.sqrt((geopy.distance.distance(coords, tx_coord).m)**2 + 100**2))

                measurement = Measurement(gps_dist, m2d(int(values[2])), id2dir(int(values[1])), int(values[3]), int(values[4]))
                measurements.append(measurement)

    df = pd.DataFrame(measurements)
    df.to_csv('data/dataset_2_w_gps.csv', index=False)

generate_dataset_gps()
 
FIGURE_DIRECTORY = "figures/"

#df = pd.read_csv('data/dataset_1_w_gps.csv')

def plot_tx_powers(angle): 

    df2 = df[(df.angle == angle)]
   
    df2 = df2.drop('rssi', axis = 1)
    df2 = df2.drop('angle', axis = 1)

    df2 = df2.groupby(['tx_power', 'th_dist']).size().reset_index().rename(columns={0: 'pdr'})
    df2['pdr'] = df2['pdr'] / (100)

    fig_dims = (10, 5)
    fig, ax = plt.subplots(figsize=fig_dims)
    sns.barplot(ax=ax,palette='Paired', data=df2, x='th_dist', y='pdr', hue='tx_power')
    # ax = sns.lineplot(palette='Paired', data=df2, x='distance', y='pdr', hue='tx_power')
    ax.set(xlabel='Distance [m]', ylabel='PDR')
    ax.set_title('Tx powers, ' + str(angle) + "deg")
    plt.legend(title = "Tx_power [dBm]", bbox_to_anchor=(1,1), borderaxespad=0)
    #file_name = "tx_powers_" + str(angle) + "deg"

    # df2.to_csv(CSV_DIRECTORY + file_name + ".csv", index = False)
    #plt.savefig(FIGURE_DIRECTORY + file_name + ".png", bbox_inches = "tight")
    plt.show()

#plot_tx_powers(0)
#plot_tx_powers(90)


def plot_rssi(distance, tx_power, angle):
    df = pd.read_csv('data/dataset-1.csv')
    df2 = df[(df.angle == angle) & (df.tx_power == tx_power) & (df.distance == distance)]
    plt.figure()
    df2 = df2.drop('angle', axis=1)
    df2 = df2.drop('tx_power', axis=1)
    df2 = df2.drop('distance', axis=1)
    ax = sns.distplot(df2)
    plt.show()

def plot_rssi_scatter(tx_power, angle):
    df = pd.read_csv('data/dataset-1.csv')
    df2 = df[(df.angle == angle) & (df.tx_power == tx_power)]
    plt.figure()
    df2 = df2.drop('angle', axis=1)
    df2 = df2.drop('tx_power', axis=1)
    df2 = df2.groupby(['distance', 'rssi']).size().reset_index(name="count")
    print(df2.head(1000))
    ax = sns.scatterplot(data=df2, x='distance', y='rssi', hue='count', size='count')
    plt.show()
#plot_rssi_scatter(-8, 0)
