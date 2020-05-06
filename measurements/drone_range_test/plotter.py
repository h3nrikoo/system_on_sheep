import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import numpy as np 
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

# generate_dataset_gps()


def generate_cloverleaf_dataset():
    # tx_coord = (63.4073927,10.4775050) #old 
    tx_coord = (63.40742, 10.47752) #ole sine koordinater 

    Measurement = namedtuple("Measurement", ['gps_dist', 'real_dist', 'th_gps_dist','angle', 'tx_power', 'rssi'])

    measurements = []

    def m2d(m):
        if m < 20:
            return math.floor((m - 1) * 50)
        else:
            return math.floor((m - 2) * 50)

    def id2dir(id):
        if id == 10:
            return 'H_BC' #horizontal polarization, best case
        elif id == 11:
            return 'V_BC' #vertical polarization, best case 
        elif id == 12: 
            return 'H_WC' #horizontal pol, worst case 
        elif id == 13: 
            return 'V_WC' #vertical pol, worst case 

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

    with open('data/combined_raw-kl1-kl2.csv') as file:
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
                    gps_dist = math.floor(geopy.distance.distance(coords, tx_coord).m)
                    real_dist = math.floor(math.sqrt((gps_dist**2 + (100-0.7)**2)))

                measurement = Measurement(gps_dist, real_dist, m2d(int(values[2])), id2dir(int(values[1])), int(values[3]), int(values[4]))
                measurements.append(measurement)

    df = pd.DataFrame(measurements)
    df.to_csv('data/dataset_clover_gps.csv', index=False)
    print("done")

# generate_cloverleaf_dataset()

FIGURE_DIRECTORY = "figures/"

DATASET = 3

if DATASET == 1:
    df = pd.read_csv('data/dataset_1_w_gps.csv')
elif DATASET == 2:
    df = pd.read_csv('data/dataset_2_w_gps.csv')
else: 
    df = pd.read_csv('data/dataset_clover_gps.csv')

#cloverleaf results 
def plot_tx_powers(angle): 

    df2 = df[(df.angle == angle)]
   
    df2 = df2.drop('rssi', axis = 1)
    df2 = df2.drop('angle', axis = 1)

    df2 = df2.groupby(['tx_power', 'th_gps_dist']).size().reset_index().rename(columns={0: 'pdr'})
    df2['pdr'] = df2['pdr'] / (100)

    fig_dims = (10, 5)
    fig, ax = plt.subplots(figsize=fig_dims)
    sns.barplot(ax=ax,palette='Paired', data=df2, x='th_gps_dist', y='pdr', hue='tx_power')
    # ax = sns.lineplot(palette='Paired', data=df2, x='distance', y='pdr', hue='tx_power')
    ax.set(xlabel='Distance [m]', ylabel='PDR')
    ax.set_title('Tx powers, ' + str(angle) + ", attempt " + str(DATASET))
    plt.legend(title = "Tx_power [dBm]", bbox_to_anchor=(1,1), borderaxespad=0)
    file_name = "tx_powers_" + str(angle) + "_attempt" + str(DATASET)

    # df2.to_csv(CSV_DIRECTORY + file_name + ".csv", index = False)
    plt.savefig(FIGURE_DIRECTORY + file_name + ".png", bbox_inches = "tight")
    plt.show()

# plot_tx_powers('H_BC')
# plot_tx_powers('V_BC')
# plot_tx_powers('H_WC')
# plot_tx_powers('V_WC')


def two_ray_ground_reflection(tx_power, dmax):
    ht = 0.7 #meter
    hr = 100 #meter
    wave_len = 0.1223642685714 #meter
    R = -1 
    G_los = 1 
    G_ref = 1

    def dBm2mW(dBm):
        return 10**(dBm / 10)

    Pt = dBm2mW(tx_power)

    def d_los(d):
        return math.sqrt(d**2 + (ht-hr)**2)

    def d_ref(d):
        return math.sqrt(d**2 + (ht+hr)**2)

    def phi(d): 
        return (2*math.pi*(d_ref(d)-d_los(d)))/wave_len

    def Pr(d):
        return Pt*(wave_len/(4*math.pi))**2 * abs(math.sqrt(G_los)/d_los(d) + R*(math.sqrt(G_ref)/d_ref(d))*math.e**(-1j*phi(d)))**2 

    def mW2dBm(mW):
        return 10*math.log(mW,10)

    graph = [mW2dBm(Pr(d)) - 3 for d in range (0, dmax)]
    
    return graph
    

def plot_pdr_two_ray_gnd(tx_power, angle):
    df2 = df[(df.tx_power == tx_power) & (df.angle == angle)]
    df2 = df2.groupby('th_gps_dist').size().reset_index().rename(columns={0: 'pdr'})
    df2['pdr'] = df2['pdr'] / (100)
    

    
    t1 = np.arange(0,df2['th_gps_dist'].max(),1)
    
   
    data1 = two_ray_ground_reflection(tx_power, df2['th_gps_dist'].max())

    fig, ax1 = plt.subplots()

    color = 'tab:red'
    ax1.set_xlabel('distance[d]')
    ax1.set_ylabel('RSSI', color=color)
    ax1.plot(t1, data1, color=color)
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

    sns.lineplot(ax = ax2, palette='Paired', data=df2, x='th_gps_dist', y='pdr')

    color = 'tab:blue'
    ax2.set_ylabel('PDR', color=color)  # we already handled the x-label with ax1
    ax2.tick_params(axis='y', labelcolor=color)

    fig.tight_layout()  # otherwise the right y-label is slightly clipped
    plt.show()



# plot_pdr_two_ray_gnd(3, 'V_BC')



#dataset 1 and 2 
def plot_rssi(distance, tx_power, angle):
    df = pd.read_csv('data/dataset-1.csv')
    df2 = df[(df.angle == angle) & (df.tx_power == tx_power) & (df.distance == distance)]
    plt.figure()
    df2 = df2.drop('angle', axis=1)
    df2 = df2.drop('tx_power', axis=1)
    df2 = df2.drop('distance', axis=1)
    ax = sns.distplot(df2)
    plt.show()

#dataset 1 and 2 
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
