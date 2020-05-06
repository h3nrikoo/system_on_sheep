#Segger project for nRF52840 DK (compatable with nRF52811) tag in drone range test measurement. 
 
-If LED2 is on, press button2 or button4 to increase or decrease tag ID between A, B, C or D shown in binary as 00, 01, 10 or 11 with LED3 and LED4 on the board. 

-If LED2 is on, press button1 to start transmission of 100x5 packets using -8,-4,0,3,4 dBm tx_power with 20ms adv interval. LED2 turns off.

-Each packets consists of tag_id, current tx_power and current measurement number (increased every time button1 is pressed) 

-If LED2 is off, no button input will be accepted and a transmission is on-going. 
