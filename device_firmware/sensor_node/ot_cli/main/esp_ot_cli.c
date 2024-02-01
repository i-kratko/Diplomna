/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * OpenThread Command Line Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"

//OpenThread
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "esp_openthread.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_types.h"
#include "esp_ot_config.h"
#include "esp_vfs_eventfd.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/uart_types.h"
#include "nvs_flash.h"
#include "openthread/cli.h"
#include "openthread/instance.h"
#include "openthread/logging.h"
#include "openthread/tasklet.h"

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
#include "esp_ot_cli_extension.h"
#endif

// Sensor libraries
#include "driver/gpio.h"
#include <bme680.h>
#include <bmp280.h>
#include <bh1750.h>

// Config file
#include "snconf.h"

//Sensors Configuration
#define BME680_I2C_ADDR 0x77
#define BMP280_I2C_ADDR 0x76
#define BH1750_I2C_ADDR 0x23
#define PORT 0
#define CONFIG_I2C_MASTER_SDA 21
#define CONFIG_I2C_MASTER_SCL 22
#define PIR_GPIO 7

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

bme680_t sensor;
bme680_values_float_t values;
uint32_t duration;

#define TAG "sensor-node"

// *** NETWWORKING ***

// Initializing the OpenThread Interface

static esp_netif_t *init_openthread_netif(const esp_openthread_platform_config_t *config)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t *netif = esp_netif_new(&cfg);
    assert(netif != NULL);
    ESP_ERROR_CHECK(esp_netif_attach(netif, esp_openthread_netif_glue_init(config)));

    return netif;
}

// Initializing the OpenThread Task

static void ot_task_worker(void *aContext)
{
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };

    // Initialize the OpenThread stack
    ESP_ERROR_CHECK(esp_openthread_init(&config));

#if CONFIG_OPENTHREAD_LOG_LEVEL_DYNAMIC
    // The OpenThread log level directly matches ESP log level
    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
#endif
#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_init();
    printf("cuckwo");
#endif


    esp_netif_t *openthread_netif;
    // Initialize the esp_netif bindings
    openthread_netif = init_openthread_netif(&config);
    esp_netif_set_default_netif(openthread_netif);

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
    esp_cli_custom_command_init();
#endif // CONFIG_OPENTHREAD_CLI_ESP_EXTENSION

    // Run the main loop
#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_create_task();
    printf("mmm");
#endif
    // Initialize the OpenThread cli

#if CONFIG_OPENTHREAD_AUTO_START
    otOperationalDatasetTlvs dataset;
    otError error = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);
    ESP_ERROR_CHECK(esp_openthread_auto_start((error == OT_ERROR_NONE) ? &dataset : NULL));
#endif
    esp_openthread_launch_mainloop();

    // Clean up
    esp_openthread_netif_glue_deinit();
    esp_netif_destroy(openthread_netif);

    esp_vfs_eventfd_unregister();
    vTaskDelete(NULL);
}

// *** SENSORS ***

// Sensor node Functions

// BME680
void init_bme680(void)
{
    memset(&sensor, 0, sizeof(bme680_t));

    ESP_ERROR_CHECK(bme680_init_desc(&sensor, BME680_I2C_ADDR, PORT, CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL));

    // init the sensor
    ESP_ERROR_CHECK(bme680_init_sensor(&sensor));

    // Changes the oversampling rates to 8x oversampling for temperature
    // and 2x oversampling for humidity. Pressure measurement is skipped.
    bme680_set_oversampling_rates(&sensor, BME680_OSR_8X, BME680_OSR_2X, BME680_OSR_2X);

    // Change the IIR filter size for temperature and pressure to 7.
    bme680_set_filter_size(&sensor, BME680_IIR_SIZE_7);

    // Change the heater profile 0 to 200 degree Celsius for 100 ms.
    bme680_set_heater_profile(&sensor, 0, 200, 100);
    bme680_use_heater_profile(&sensor, 0);

    // Set ambient temperature to 10 degree Celsius
    bme680_set_ambient_temperature(&sensor, 10);

    // as long as sensor configuration isn't changed, duration is constant
    bme680_get_measurement_duration(&sensor, &duration);

}
void get_bme680_readings(void)
{
    while (1){
        // trigger the sensor to start one TPHG measurement cycle
        if (bme680_force_measurement(&sensor) == ESP_OK)
        {
            // passive waiting until measurement results are available
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            // get the results and do something with them
            if (bme680_get_results_float(&sensor, &values) == ESP_OK)
                printf("BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f KOhm\n",
                values.temperature, values.humidity, values.pressure, values.gas_resistance);
        }
    } 
}

// BMP280
void bmp280_func(void *pvParameters)
{
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));

    bool bme280p = dev.id == BME280_CHIP_ID;
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

    float pressure, temperature, humidity;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }

        printf("BMP280 Sensor: %.2f °C, %.2f %%, %.2f Pa \n",
         temperature, humidity, pressure);
    }
}

// BH1750
void bh1750_func(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t)); // Zero descriptor

    ESP_ERROR_CHECK(bh1750_init_desc(&dev, BH1750_I2C_ADDR, 0, CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(bh1750_setup(&dev, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH));

    while (1)
    {
        uint16_t lux;

        if (bh1750_read(&dev, &lux) != ESP_OK)
            printf("Could not read lux data\n");
        else
            printf("BH1750 Sensor: %d Lux\n", lux);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// SR602
void sr602_func(void)
{
    // Reset previous GPIO configuration
    gpio_reset_pin(PIR_GPIO);
    // Set the given GPIO as an input
    gpio_set_direction(PIR_GPIO, GPIO_MODE_INPUT);

    // Needed variables
    int motion_count = 0;
    int detected = 0;
    int waitTime = 3000;

    while (1) {
        if (detected) { 
            vTaskDelay(waitTime / portTICK_PERIOD_MS);            
            detected = 0;
        }
        if(gpio_get_level(PIR_GPIO)) {
            motion_count++;
            printf("SR602 Snesor: Motion was detected %d times\n", motion_count);
            detected = 1;   
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }   
    }
}

void app_main(void)
{
    // Used eventfds:
    // * netif
    // * ot task queue
    // * radio driver
    esp_vfs_eventfd_config_t eventfd_config = {
        .max_fds = 3,
    };

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));
    xTaskCreate(ot_task_worker, "ot_cli_main", 10240, xTaskGetCurrentTaskHandle(), 5, NULL);

    ESP_ERROR_CHECK(i2cdev_init());

    if (BME680_SNEN == 1) {
        init_bme680();
        xTaskCreatePinnedToCore(get_bme680_readings, "BME680", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    }
    if (BMP280_SNEN == 1) {
        xTaskCreatePinnedToCore(bmp280_func, "BMP280", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    }
    if (BH1750_SNEN == 1) {
        xTaskCreatePinnedToCore(bh1750_func, "BH1750", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL, APP_CPU_NUM);
    }
    if (SR602_SNEN == 1) {
        xTaskCreatePinnedToCore(sr602_func, "SR602", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL, APP_CPU_NUM);
    }

}
