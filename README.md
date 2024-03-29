# ESP32-Time-lapse-cam
Turn an ESP32-CAM into a time lapse camera and pull images remotely over the network

![Time lapse example](https://github.com/alhockly/ESP32-Time-lapse-cam/blob/master/example.gif)


This repo contains a Platformio project that can be opened in Visual Studio Code and a python script to download files from the ESP32-CAM

## Setup
1. Edit WifiCredentials.h to contain SSID and Password of Wifi to use
2. upload to ESP32 via platformio
3. Connect ESP32 to power and allow it to start
4. Modify python script to point to IP of ESP
5. Run python script with arg for number of files to download e.g `python3 download.py 40`


The ESP32 software implements the following endpoints via a webserver:

/list

/download e.g `/download?file=<fileName>`

/latest

/time

