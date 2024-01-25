# Overview
This is an example analog clock application for ESP32S3 based `LilyGo T-RGB 2.1 inch Half Circle` display module. 

Libs Dependency Graph
|-- GFX Library for Arduino @ 1.4.2
|-- OneButton @ 2.5.0
|-- lvgl @ 8.3.6


## To Build
1. Install Visual Studio Code and PlatformIO
2. Open the Folder in Visual Studio
3. Change the WiFi details in `src/main.cpp `
    ```
        #define WIFI_SSID "<WIFI_SSID>"
        #define WIFI_PASSWORD "<WiFI_PASSWORD>"
    ```
4. (Optional) Update the timezone and DST in `src/main.cpp `: 
    ```
        const long gmtOffset_sec = (1 * 60 * 60);
        const int daylightOffset_sec = 0;
    ``` 
5. Build and Flash the Project
