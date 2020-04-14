import geopy.distance
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df1 = pd.read_csv('map_data/map_data_1km.csv')

x1 = []
y1 = []

length = df1.index.size - 1

for i in range(length, -1, -1):
    y1.append(df1.iloc[i,2])
    coords_1 = (df1.iloc[length,0], df1.iloc[length,1])
    coords_2 = (df1.iloc[i,0], df1.iloc[i,1])
    x1.append(geopy.distance.distance(coords_1, coords_2).m)


df1['dist'] = x1
df1['moh'] = y1
df1 = df1.drop('lat', axis = 1)
df1 = df1.drop('long', axis = 1)

df1.to_csv('map_data/altitude_data_1000m.csv', index = False)

df2 = pd.read_csv('map_data/map_data_700m.csv')

x2 = []
y2 = []

for i in range(df2.index.size): 
    y2.append(df2.iloc[i,2])
    coords_1 = (df2.iloc[0,0], df2.iloc[0,1])
    coords_2 = (df2.iloc[i,0], df2.iloc[i,1])
    x2.append(geopy.distance.distance(coords_1, coords_2).m)

df2['dist'] = x2
df2 = df2.drop('lat', axis = 1)
df2 = df2.drop('long', axis = 1)
df2.to_csv('map_data/altitude_data_700m.csv', index = False)

fig, ax = plt.subplots() 
ax.plot(x1,y1)
ax.plot(x2, y2)
plt.show() 

# df['distance'] = x

# # print(df.head(20))

# df.plot(x = 'distance', y = 'moh')

 
        
