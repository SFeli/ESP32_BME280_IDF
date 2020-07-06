/* Aus BOSCH-Anleitung unter https://github.com/BoschSensortec/BME280_driver */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "bme280.h"
 
#define SDA_GPIO 21
#define SCL_GPIO 22
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY
#define RTOS_DELAY_1SEC          ( 1 * 1000 / portTICK_PERIOD_MS)
struct bme280_data comp_data;
struct bme280_dev bme;
uint8_t settings_sel;
int8_t bmestatus = BME280_OK;

void i2c_master_init() {
    i2c_config_t i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = SDA_GPIO,
            .scl_io_num = SCL_GPIO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
 
//*************// Verify that the I2C slave is working properly
    esp_err_t f_retval;                 /* changed */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_I2C_ADDR_PRIM << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    f_retval = i2c_master_cmd_begin(I2C_NUM_0, cmd, RTOS_DELAY_1SEC);
    if (f_retval != ESP_OK) {
        printf("I2C slave NOT working or wrong I2C slave address - error (%i)", f_retval);
    }
    i2c_cmd_link_delete(cmd);
}

int8_t i2c_reg_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    int8_t iError;
    int8_t i2c_addr = *((int *)intf_ptr);
    esp_err_t esp_err;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, reg_addr, true);
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (i2c_addr << 1) | I2C_MASTER_READ, true);
    if (length > 1) {
        i2c_master_read(cmd_handle, reg_data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd_handle, reg_data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd_handle);
    esp_err = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000 / portTICK_PERIOD_MS);
 
    if (esp_err == ESP_OK) {
        iError = 0;
    } else {
        iError = -1;
    }
    i2c_cmd_link_delete(cmd_handle);
    return iError;
}
 
int8_t i2c_reg_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    int8_t i2c_addr = *((int *)intf_ptr);
    int8_t iError;
    esp_err_t esp_err;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, reg_addr, true);
    i2c_master_write(cmd_handle, reg_data, length,true);
    i2c_master_stop(cmd_handle);
    esp_err = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000 / portTICK_PERIOD_MS);
    if (esp_err == ESP_OK) {
        iError = 0;
    } else {
        iError = -1;
    }
    i2c_cmd_link_delete(cmd_handle);
    return iError;
}

void delay_ms(uint32_t period_ms, void *intf_ptr) {
    //vTaskDelay(period_ms / portTICK_PERIOD_MS);
//********************//
    ets_delay_us(period_ms * 1000);
}
 
void print_rslt(const char api_name[], int8_t rslt) {
 
    if (rslt != BME280_OK) {
        printf("%s\t", api_name);
        if (rslt == BME280_E_NULL_PTR) {
            printf("Error [%d] : Null pointer error\r\n", rslt);
        } else if (rslt == BME280_E_DEV_NOT_FOUND) {
            printf("Error [%d] : Device not found\r\n", rslt);
        } else if (rslt == BME280_E_INVALID_LEN) {
            printf("Error [%d] : Invalid Lenght\r\n", rslt);
        } else if (rslt == BME280_E_COMM_FAIL) {
            printf("Error [%d] : Bus communication failed\r\n", rslt);
        } else if (rslt == BME280_E_SLEEP_MODE_FAIL) {
            printf("Error [%d] : SLEEP_MODE_FAIL\r\n", rslt);
        } else {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }else{
        printf("%s\t", api_name);
        printf(" BME280 status [%d]\n",rslt);
    }
}

void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
        printf("Temperature in °C: %0.2f, Pressure: %0.2f, Humidity: %0.2f\r\n",comp_data->temperature, (comp_data->pressure / 100), comp_data->humidity);
#else
        printf("%ld, %ld, %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}

//void bme280_normal_mode() 
void bme280_normal_mode(struct bme280_dev *bme) 
{
//*******// configure the sampling according to data spec sheet recommendation
    bme->settings.osr_h = BME280_OVERSAMPLING_1X;
    bme->settings.osr_p = BME280_OVERSAMPLING_16X;
    bme->settings.osr_t = BME280_OVERSAMPLING_2X;
    bme->settings.filter = BME280_FILTER_COEFF_16;
    bme->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

    settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;

    bmestatus = bme280_set_sensor_settings(settings_sel, bme);
    print_rslt("bme280_set_sensor_settings status", bmestatus);

    bmestatus = bme280_set_sensor_mode(BME280_NORMAL_MODE, bme);  
    print_rslt("bme280_set_sensor_mode status", bmestatus);
    while (1) {
        //*************// Add a delay of minimum 70millisec or more to get sensor reading!
        bme->delay_us(70, bme->intf_ptr );
        /* Reading the raw data from sensor  (BME280_ALL, &comp_data, dev);  */
        bmestatus = bme280_get_sensor_data(BME280_ALL, &comp_data, bme);
        // print_rslt("bme280_get_sensor_data", bmestatus);
        print_sensor_data(&comp_data);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void bme280_forced_mode(struct bme280_dev *bme)
{
    uint32_t req_delay;
    /* Recommended mode of operation: Indoor navigation */
    bme->settings.osr_h = BME280_OVERSAMPLING_1X;
    bme->settings.osr_p = BME280_OVERSAMPLING_16X;
    bme->settings.osr_t = BME280_OVERSAMPLING_2X;
    bme->settings.filter = BME280_FILTER_COEFF_16;
    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    bmestatus = bme280_set_sensor_settings(settings_sel, bme);
    print_rslt("bme280_set_sensor_settings status", bmestatus);
	
    /*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
     *  and the oversampling configuration. */
    req_delay = bme280_cal_meas_delay(&bme->settings);

    printf("Temperature, Pressure, Humidity\r\n");
    /* Continuously stream sensor data */
    while (1) {
        bmestatus = bme280_set_sensor_mode(BME280_FORCED_MODE, bme);
	print_rslt("bme280_set_sensor_mode status", bmestatus);
        /* Wait for the measurement to complete and print data @25Hz */
        bme->delay_us(req_delay, bme->intf_ptr);
        bmestatus = bme280_get_sensor_data(BME280_ALL, &comp_data, bme);
        print_sensor_data(&comp_data);
    }
}

void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    i2c_master_init();

    uint8_t dev_addr = BME280_I2C_ADDR_PRIM;
    //  bme.intf_ptr = BME280_I2C_ADDR_SEC;     // 0x77
    bme.intf_ptr = &dev_addr;                   // 0x76
    bme.intf = BME280_I2C_INTF;
    bme.read = i2c_reg_read;
    bme.write = i2c_reg_write;
    bme.delay_us = delay_ms;

    bmestatus = bme280_init(&bme);
    print_rslt("bme280_init status      ", bmestatus);

    bme280_normal_mode(&bme);
    // bme280_forced_mode(&bme);
    // xTaskCreate(bme280_normal_mode, "bme280_normal", 2048, NULL, 6, NULL);
    // xTaskCreate(bme280_forced_mode, "bme280_forced", 2048, NULL, 6, NULL);
}
