import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import numpy as np 
from collections import namedtuple
import math
import geopy.distance
pd.set_option('display.max_rows', 10000)

DRONE_HEIGHT = 100 - 1.5

def generate_dataset_gps():
    tx_coord = (63.40742, 10.47752) #ole sine koordinater 


    Measurement = namedtuple("Measurement", ['bin_dist','true_dist','measured_dist','error','mean_clk_ticks','var_clk_ticks','num_pkt_tx','num_pkt_rx', 'pdr'])

    measurements = []

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

    

    distance_bins = []
    for d in range(0,800,50):
        distance_bins.append(round(math.sqrt(d*d + DRONE_HEIGHT*DRONE_HEIGHT)))

        
    with open('data/raw-combined/Combined.csv') as file:
        skip_header = True
        for line in file.readlines():
            line = line.strip()
            values = line.split(",")

            if(skip_header) :
                skip_header = False 
                continue

            measured_dist = int(values[4])

            #remove bad measurements 
            if measured_dist < 80:
                continue 

            gps_coords = convert_coord(values[9], values[10])
            true_dist = math.floor(math.sqrt((geopy.distance.distance(gps_coords, tx_coord).m)**2 + DRONE_HEIGHT**2))
            error = measured_dist - true_dist


            #sort measurements into bins since true dist varies a lot 
            bin_dist = 0
            for d in distance_bins: 
                if(true_dist + 5 + d/50 >= d):
                    bin_dist = d 
                else: 
                    break


            mean_clk_ticks = int(values[5])
            var_clk_ticks = int(values[6])
            num_pkt_tx = int(values[8])
            num_pkt_rx = int(values[7])
            pdr = num_pkt_rx/num_pkt_tx

            #drop measurements with low pdr 
            if(pdr < 0.5):
               continue

            measurement = Measurement(bin_dist,true_dist, measured_dist, error, mean_clk_ticks, var_clk_ticks, num_pkt_tx, num_pkt_rx, pdr)
            measurements.append(measurement)

    df = pd.DataFrame(measurements)
    df.to_csv('data/rttr_measurement_dataset.csv', index=False)

generate_dataset_gps()

df = pd.read_csv('data/rttr_measurement_dataset.csv')
df = df[df.bin_dist != 608]

print(df.head(1000))

def box_plot(): 
    fig_dims = (16, 9)
    fig, ax = plt.subplots(figsize=fig_dims)
    ax = sns.boxplot(palette='Paired', data=df, x='bin_dist', y='error', hue = 'num_pkt_tx', showmeans=True, showfliers = True)
    ax.set_title("RTTR error with different number of packets transmitted")
    ax.set(xlabel='Distance bin [m]', ylabel='RTTR error [m]')
    plt.legend(title = "Packet number", bbox_to_anchor=(1,1), borderaxespad=0)
    file_name = "figures/box_plot.png"
    plt.savefig(file_name)
    plt.show()

def mean_error_line_plot(): 
    df2 = df.groupby(['bin_dist','num_pkt_tx'])['error'].mean().reset_index()

    fig_dims = (16, 9)
    fig, ax = plt.subplots(figsize=fig_dims)
    
    ax = sns.lineplot(palette='Paired', data=df2, x = 'bin_dist', y = 'error', hue='num_pkt_tx')
    ax.set_title("RTTR mean error with different number of packets transmitted")
    ax.set(xlabel='Distance [m]', ylabel='Mean error [m]')
    plt.legend(title = "Packet number", bbox_to_anchor=(1,1), borderaxespad=0)
    file_name = "figures/mean_error_line_plot_2.png"
    plt.savefig(file_name)
    plt.show()

def mean_error_bar_plot(): 
    df2 = df.groupby(['bin_dist','num_pkt_tx'])['error'].mean().reset_index()

    fig_dims = (16, 9)
    fig, ax = plt.subplots(figsize=fig_dims)
    
    ax = sns.barplot(palette='Paired', data=df2, x = 'bin_dist', y = 'error', hue='num_pkt_tx')
    ax.set_title("RTTR mean error with different number of packets transmitted")
    ax.set(xlabel='Distance bin [m]', ylabel='Mean error [m]')
    plt.legend(title = "Packet number", bbox_to_anchor=(1,1), borderaxespad=0)
    file_name = "figures/mean_error_box_plot_2.png"
    plt.savefig(file_name)
    plt.show()



def combined_boxplot(): 

    fig_dims = (16, 9)
    fig, ax = plt.subplots(figsize=fig_dims)
    ax.xaxis.grid(True)
    ax = sns.boxplot(palette='Paired', data=df, y='num_pkt_tx', x='error', orient='h', order=[1024,512,256,128,64,32,16], showfliers=False, showmeans=True, meanprops={"marker":".","markerfacecolor":"darkred", "markeredgecolor":"white", "markersize":"10"})
    # ax = sns.swarmplot(data=df, y='num_pkt_tx', x='error', orient='h', order=[1024,512,256,128,64,32,16], size=3,linewidth=0, alpha=0.9)
    ax.set_title("RTTR error with different number of packets transmitted")
    ax.set(xlabel='Error[m]', ylabel='Num packets')


    ax.set_xticks(np.arange(-50,61,10))
    file_name = "figures/combined_box_plot_2.png"
    plt.savefig(file_name)
    plt.show()

def strip_plot(): 
    fig_dims = (16, 9)
    fig, ax = plt.subplots(figsize=fig_dims)
    ax.xaxis.grid(True)
    
    
    ax.set_title("RTTR error with different number of packets transmitted")
    ax.set(xlabel='Error[m]', ylabel='Num packets')


    ax.set_xticks(np.arange(-50,61,10))
    file_name = "figures/dunno.png"
    plt.savefig(file_name)
    plt.show()



box_plot()
mean_error_line_plot() 
mean_error_bar_plot()
# err_pkt_num_bar_plot(16)
combined_boxplot()


