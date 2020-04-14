# import pandas as pd 
# import glob 
# import matplotlib.pyplot as plt 

# headers = ["tx power", "RSSI", "height", "rotation", "coded_phy", "distance"]
# # data = pd.read_csv("data/100.CSV", names = headers, sep = ";", skiprows = 1)

# # data2 = pd.read_csv("data/200.CSV", names = headers, sep = ";", skiprows = 1)

# # df = pd.concat([pd.read_csv(f, names = headers, sep = ";", skiprows = 1) for f in glob.glob('data/*.csv')], ignore_index = True)


# li = []

# for filename in glob.glob('data/*.csv'):
#     df = pd.read_csv(filename, names = headers, sep = ";", skiprows = 1)

#     if filename.find('C') == 5: 
#         df["coded_phy"] = 1
#         df["distance"] = str(filename[6:9])
#     else: 
#         df["coded_phy"] = 0
#         df["distance"] = str(filename[5:8])

#     li.append(df)

# data = pd.concat(li, axis=0, ignore_index=True)

# # data.plot(x = "distance", y = "rssi", kind = "bar")

# # def plot_angles(coded_phy):
# #     df2 = df.groupby(['distance', 'coded_phy', 'angle']).size().reset_index().rename(columns={0: 'records'})
# #     df2['records'] = df2['records'] / (100 * 8 * 2)
# #     plt.figure()
# #     ax = sns.barplot(palette='Paired', data=df2[df2['coded_phy'] == coded_phy], x='distance', y='records', hue='angle')
# #     ax.set(xlabel='Distance (m)', ylabel='Packet Ratio')
# #     ax.set_title("Angles")
# #     plt.legend(title="Angle", loc="upper right")
# #     plt.show()


# # print(data)