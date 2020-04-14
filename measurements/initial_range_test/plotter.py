import matplotlib.pyplot as plt
import os
from collections import namedtuple
import pandas as pd
import seaborn as sns
import math
DATA_DIRECTORY = "data/"
FIGURE_DIRECTORY = "figures/"
CSV_DIRECTORY = "processed_data/"

def import_csv(): 

    Measurement = namedtuple("Measurement", ['distance', 'phy', 'tx_power', 'rssi', 'height', 'angle'])

    files = []
    for directory_item in os.listdir(DATA_DIRECTORY):
        if ".CSV" in directory_item:
            files.append(directory_item)

    measurements = []
    for file in files:
        if file[0] == "C":
            phy = 'Coded'
            distance = int(file[1:4])
        else:
            phy = 'LE'
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

                measurements.append(Measurement(distance, phy, tx_power, rssi, height, angle))

    df = pd.DataFrame(measurements)

    df.to_csv('data/dataset.csv', index=False)

df = pd.read_csv('data/dataset.csv')



def plot_tx_powers(phy, angle, height): 

    df2 = df[(df.phy == phy) & (df.height == height) & (df.angle == angle)]
   
    df2 = df2.drop('rssi', axis = 1)
    df2 = df2.drop('height', axis = 1)
    df2 = df2.drop('angle', axis = 1)
    df2 = df2.drop('phy', axis = 1)

    df2 = df2.groupby(['tx_power', 'distance']).size().reset_index().rename(columns={0: 'pdr'})
    df2['pdr'] = df2['pdr'] / (100)

    ax = sns.lineplot(palette='Paired', data=df2, x='distance', y='pdr', hue='tx_power')
    ax.set(xlabel='Distance [m]', ylabel='Packet Delivery Ratio')
    ax.set_title(phy + " PHY, " + str(height) + "cm, " + str(angle) + "deg")
    # plt.legend(title="Tx_power [dBm]", loc="upper left", bbox_to_anchor=(1.04, 1))
    plt.legend(title = "Tx_power [dbm]", bbox_to_anchor=(1.04,1), borderaxespad=0)
    file_name = "tx_powers_" + str(height) + "cm_" + str(angle) + "dBm_" + phy + "_PHY"

    df2.to_csv(CSV_DIRECTORY + file_name + ".csv", index = False)
    plt.savefig(FIGURE_DIRECTORY + file_name + ".png", bbox_inches = "tight")
    plt.show()

# plot_tx_powers('Coded', 90, 200)
# plot_tx_powers('LE', 90, 200)


def plot_Phy_modes(tx_power_le, tx_power_coded = None):
    if tx_power_coded == None: 
        tx_power_coded = tx_power_le
    if tx_power_le == 'all':
        tx_power_coded = 'all' 

    df_plot_le = df[df["phy"] == 'LE']
    if tx_power_le != 'all': 
        df_plot_le = df_plot_le[df_plot_le["tx_power"] == tx_power_le]

    df_plot_coded = df[df["phy"] == 'Coded']
    if tx_power_coded != 'all': 
        df_plot_coded = df_plot_coded[df_plot_coded["tx_power"] == tx_power_coded]

    df_plot = df_plot_le.append(df_plot_coded, sort=False)

    df_plot = df_plot.groupby(['distance', 'phy']).size().reset_index().rename(columns={0: 'packets'})
    
    if tx_power_le != 'all': 
        df_plot['packets'] = df_plot['packets'] / (100 * 1 * 2 * 3)
    else: 
        df_plot['packets'] = df_plot['packets'] / (100 * 8 * 2 * 3)

    plt.figure()

    ax = sns.barplot(palette='Paired', data=df_plot, x='distance', y='packets', hue='phy')
    ax.set(xlabel='Distance (m)', ylabel='Packet Ratio')
    ax.set_title("Coded Phy")
    plt.legend(loc="upper right", title = "Coded vs LE PHY")
    file_name = FIGURE_DIRECTORY + "phy_modes_" + str(tx_power_le) + "dBm_" + str(tx_power_coded) + "dBm.png"
    plt.savefig(file_name)
    plt.show()

# plot_Phy_modes(4, -4)

    
def plot_angles(phy):
    df2 = df.groupby(['distance', 'phy', 'angle']).size().reset_index().rename(columns={0: 'packets'})
    df2['packets'] = df2['packets'] / (100 * 8 * 2)
    plt.figure()
    ax = sns.barplot(palette='Paired', data=df2[df2['phy'] == phy], x='distance', y='packets', hue='angle')
    ax.set(xlabel='Distance (m)', ylabel='Packet Delivery Ratio')
    ax.set_title("Angles")
    plt.legend(title="Angle", loc="upper right")
    file_name = FIGURE_DIRECTORY + "angles_" + phy + "_PHY.png"
    plt.savefig(file_name)
    plt.show()
   
def dBm2mW(dBm):
    return 10**(dBm / 10)  

def mW2dBm(mW):
    return 10*math.log(mW,10)

def dBi2dB(dBi): 
    return 10**(dBi/10)

def friis(d, tx_power):
    LAMBDA = 0.122 
    return mW2dBm(dBi2dB(0)*dBi2dB(0)*dBm2mW(tx_power)*((LAMBDA/(4*math.pi*d))**2))

def friis_db(d, tx_power, angle):
    LAMBDA = 0.122 
    phi = math.radians(90 - angle)
    plf = math.cos(phi)**2
    return 20*math.log10(LAMBDA/(4*math.pi*d)) + tx_power + math.log10(plf)

def rssi_friis_stuff(d, tx_power):
    LAMBDA = 0.122 
    return tx_power + 0 + 20*math.log10(LAMBDA/4*math.pi) - 10*math.log10(d)

def plot_rssi_friis(tx_power, angle, height, phy):
    df2 = df[df["tx_power"] == tx_power]
    df2 = df2[df2["height"] == height]
    df2 = df2[df2["angle"] == angle]
    df2 = df2[df2["phy"] == phy]
    df2 = df2.groupby(['distance'])['rssi'].mean().reset_index()
    plt.figure()
    ax = sns.lineplot(palette='Paired', data=df2, x='distance', y='rssi', label = 'Average Measured RSSI')
    a = [friis(d, tx_power) for d in df2['distance']]
    plt.plot(df2['distance'], a , label = 'Theoretical RSSI')

    ax.set(xlabel='Distance[m]', ylabel='RSSI[dBm]')
    ax.set_title("Friis transmission equation vs measured")

    file_name = FIGURE_DIRECTORY + "rrsi_friis" + str(height) + "cm_" + str(angle) + "deg_"+ str(tx_power) + "dBm_" + phy + "_PHY.png"
    plt.savefig(file_name)

    plt.show()

# plot_rssi_friis(4, 90, 200, 'Coded')

def plot_rssi_angle(tx_power, height, phy): 
    df2 = df
    if tx_power != 'all':
        df2 = df2[df["tx_power"] == tx_power]
    if height != 'all':
        df2 = df2[df2["height"] == height]
    if phy != 'all':
        df2 = df2[df2["phy"] == phy]
    plt.figure()
    # ax = sns.boxplot(palette='Paired', data=df2, x='distance', y='rssi', hue = 'angle', showmeans=True, showfliers = True)

    df2 = df2.groupby(['distance', 'angle'])['rssi'].min().reset_index()
    ax = sns.lineplot(palette='Paired', data=df2, x='distance', y='rssi', hue = 'angle')

    df2 = df2.groupby(['distance'])['rssi'].min().reset_index()

    print(df2.head(50))

    a = [friis_db(d, tx_power, 0) for d in df2['distance']]
    b = [friis_db(d, tx_power, 45) for d in df2['distance']]
    c = [friis_db(d, tx_power, 90) for d in df2['distance']]
    df2['rssi_t0'] = a 
    df2['rssi_t45'] = b 
    df2['rssi_t90'] = c 

    ax1 = sns.lineplot(palette='Paired', data=df2, x='distance', y='rssi_t0', label = 'Theoretical RSSI 0 deg')
    ax2 = sns.lineplot(palette='Paired', data=df2, x='distance', y='rssi_t45', label = 'Theoretical RSSI 45 deg')
    ax3 = sns.lineplot(palette='Paired', data=df2, x='distance', y='rssi_t90', label = 'Theoretical RSSI 90 deg')

    ax.set_title("Measured vs theoretical RSSI," + phy + " PHY, "+ str(tx_power) + "dBm, " + str(height) + "cm")
    ax.set_xscale('log')


    file_name = FIGURE_DIRECTORY + "rrsi_angle_" + str(height) + "cm_" + str(tx_power) + "dBm_" + phy + "_PHY.png"
    plt.savefig(file_name)
    
    plt.show() 
    
plot_rssi_angle(0,200,'Coded')

def plot_rssi_hist(tx_power, height, angle, phy, distance): 
    df2 = df[(df.phy == phy) & (df.height == height) & (df.angle == angle) & (df.tx_power == tx_power) & (df.distance == distance)]
    plt.figure()
    df2 = df2.drop('height', axis = 1)
    df2 = df2.drop('angle', axis = 1)
    df2 = df2.drop('phy', axis = 1)
    df2 = df2.drop('distance', axis = 1)
    df2 = df2.drop('tx_power', axis = 1)

    ax = sns.distplot(df2)

    plt.show()

# plot_rssi_hist(0,200,90,'Coded', 100)
# plot_rssi_hist(0,200,90,'Coded', 200)
# plot_rssi_hist(0,200,90,'Coded', 300)
# plot_rssi_hist(0,200,90,'Coded', 400)
# plot_rssi_hist(0,200,90,'Coded', 500)
# plot_rssi_hist(0,200,90,'Coded', 600)
# plot_rssi_hist(0,200,90,'Coded', 700)
# plot_rssi_hist(0,200,90,'Coded', 900)

def plot_rssi_friis_2(tx_power, angle, height, phy):
    df2 = df[(df.phy == phy) & (df.height == height) & (df.angle == angle) & (df.tx_power == tx_power)]
    df3 = df2.groupby(['distance'])['rssi'].max().reset_index()
    df4 = df2.groupby(['distance'])['rssi'].min().reset_index()

    plt.figure()
    ax1 = sns.lineplot(palette='Paired', data=df3, x='distance', y='rssi', label = 'Max Measured RSSI')
    ax2 = sns.lineplot(palette='Paired', data=df4, x='distance', y='rssi', label = 'Min Measured RSSI')
    # a = [friis(d, tx_power-1) for d in df2['distance']]
    # plt.plot(df2['distance'], a , label = 'Theoretical Pr')

    b = [friis_db(d, tx_power, angle) for d in df3['distance']]
    df3['rssi_t'] = b 

    if phy == 'Coded':
        df3['threshold'] = -103
    else:
        df3['threshold'] = -95

    ax3 = sns.lineplot(palette='Paired', data=df3, x='distance', y='rssi_t', label = 'Theoretical RSSI')

    ax4 = sns.lineplot(palette='Paired', data=df3, x='distance', y='threshold', label = 'Sensitivity/Threshold?')

    # plt.plot(df2['distance'], b , label = 'Theoretical RSSI with PLF')
    # c = [rssi_friis_stuff(d, tx_power) for d in df2['distance']]
    # plt.plot(df2['distance'], c , label = 'Theoretical RSSI')

    # d = [rssi_friis_simple(d, tx_power) for d in df2['distance']]
    # plt.plot(df2['distance'], d , label = 'Simple RSSI')

    ax1.set(xlabel='Distance[m]', ylabel='RSSI[dBm]')
    ax1.set_title("Measured vs theoretical RSSI," + phy + " PHY, "+ str(tx_power) + "dBm, " + str(height) + "cm, " + str(angle) + "deg")
    ax1.set_xscale('log')


    file_name = FIGURE_DIRECTORY + "rrsi_friis" + str(height) + "cm_" + str(angle) + "deg_"+ str(tx_power) + "dBm_" + phy + "_PHY.png"
    plt.savefig(file_name)

    plt.show()

# plot_rssi_friis_2(0, 90, 200, 'Coded')
# plot_rssi_friis_2(0, 90 ,200,'LE')



