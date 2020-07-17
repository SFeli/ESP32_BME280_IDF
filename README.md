# ESP32_BME280_IDF
ESP32 - Board with BME280 - Sensor and "new" Bosch-Library (Version 3.5.0) in IDF
https://github.com/BoschSensortec/BME280_driver

Under ESP-IDF - (PlatformIO) my first programm with a lot of lines from BOSCH-Sensortec / BME280_driver repository.
Notes:
In main.c both alternative readings are implemented :  
    - Normal reading  (periodic)
    - Forced reading  (on demand)
Additional as described on BOSCH-Library you can change the code for  
    - 32 bit or  
    - 64 bit machine  
as well as a variant with:   
    - Float precision or  
    - 64-bit precision  
can be choosed in bme280_defs.h
