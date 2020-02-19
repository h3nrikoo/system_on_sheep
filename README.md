# System on Sheep
This project aims to track sheep using BLE long range tags on sheeps equipped with nRF811 and a drone equipped with nRF840. 

The project will first aim to detect advertising packets from tags and store the advertised data along with the GPS coordinate of the drone. The more advanced version will use Round Trip Timing to measure the distance between a tag and the drone. The distance will then be measured several times while the drone is moving in order to use multilateration to estimate the position of the sheep. 


## Use code with SDK and Segger 
1. Open the .emProject file with an IDE that is not Segger embedded studio 
2. Find and replace "../../../../../.." with $(SDK_ROOT:../../../../../..)
3. Open SEGGER -> tools -> options -> building -> gloal macros 
4. write: "SDK_ROOT=<YOUR_PATH>/SDK"

