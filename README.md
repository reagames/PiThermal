# PiThermal
Python script which runs on Raspberry Pi, enabling interaction with Seek thermal imaging camera 

# What is it all about?
This is a significantly modified code originating from seek_2.0 script, published originally by eevblog user cynfab.
The code allows to run a complete setup of Raspberry Pi, Pi Camera and Seek Thermal thermal imaging device using framebuffer output to external SPI display (connected to Raspberry using GPIOs - see notro/fbtft project wiki for more insights). 
The code requires no X Server (meaning it can run from command line using framebuffer, thus allowing to use minimal Raspbian Lite image to save resources). The system has basic GUI which can be navigated using three hardware buttons attached to Raspberry GPIOs (middle button to cycle through menu and left/right to modify parameters). 

# Legal notes
This code is distributed under MIT license, meaning no originator's liability. The code was initially developed solely for research purposes and does not intend any other use. Use this code at your own risk and make sure to comply with laws and regulations. May the Force be with you.  
