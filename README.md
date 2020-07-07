# ESP32_BME280_IDF
ESP32 - Board with BME280 - Sensor and "new" Bosch-Library (Version 3.5.0) in IDF

Under ESP-IDF - (PlatformIO) my first programm with a lot of lines from BOSCHSensortec / BME280_driver repository.
Notes:
In main.c you both alternative readings are implemented :  
    - Normal reading 
    - Forced reading
Additional as described on BOSCH-Library you can change the code for  
    - 32 bit or  
    - 64 bit machine  
as well as a variant with:   
    - Float precision or  
    - 64-bit precision  
can be choosed in bme280_defs.h

