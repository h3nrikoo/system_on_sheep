import os
import glob
import pandas as pd

flag = True 
results = pd.DataFrame()

for counter, current_file in enumerate(glob.glob("*.CSV")):
    
    namedf = pd.read_csv(current_file, header=None, sep=";")
    # print(namedf)
    results = pd.concat([results, namedf])

results.to_csv('Combined.csv', index=None, sep=",")



# extension = 'CSV'
# all_filenames = [i for i in glob.glob('*.{}'.format(extension))]


# #combine all files in the list
# combined_csv = pd.concat([pd.read_csv(f, sep=';') for f in all_filenames ])
# #export to csv
# print(combined_csv.head())
# # combined_csv.to_csv( "combined_raw.csv", index=False, encoding='utf-8-sig')