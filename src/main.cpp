#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"

#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"

#include "mqtt_client.h"

// ================= НАЛАШТУВАННЯ =================

constexpr const char* WIFI_SSID = "YOUR_WIFI_SSID";
constexpr const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

constexpr const char* BROKER_URI = "mqtt://broker.hivemq.com";

constexpr const char* STUDENT_NAME = "oyakovlev";

// BME280
constexpr gpio_num_t I2C_SDA = GPIO_NUM_8;
constexpr gpio_num_t I2C_SCL = GPIO_NUM_9;
constexpr uint8_t BME280_ADDR = 0x76;

// ================================================

static const char* TAG = "MQTT_BME280";

static bool is_mqtt_started = false;

static esp_mqtt_client_handle_t mqtt_client = nullptr;

static i2c_master_bus_handle_t i2c_bus = nullptr;
static i2c_master_dev_handle_t bme280_dev = nullptr;

// ================= BME280 CALIBRATION =================

static uint16_t dig_T1;
static int16_t dig_T2;
static int16_t dig_T3;

static uint16_t dig_P1;
static int16_t dig_P2;
static int16_t dig_P3;
static int16_t dig_P4;
static int16_t dig_P5;
static int16_t dig_P6;
static int16_t dig_P7;
static int16_t dig_P8;
static int16_t dig_P9;

static uint8_t dig_H1;
static int16_t dig_H2;
static uint8_t dig_H3;
static int16_t dig_H4;
static int16_t dig_H5;
static int8_t dig_H6;

static int32_t t_fine = 0;

// ================= FORWARD DECLARATIONS =================

static void mqtt_app_start(void);
static void sensor_task(void* arg);

// ================= BME280 I2C =================

static esp_err_t bme280_write_register(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    return i2c_master_transmit(
        bme280_dev,
        data,
        sizeof(data),
        1000
    );
}

static esp_err_t bme280_read_registers(
    uint8_t reg,
    uint8_t* data,
    size_t len)
{
    return i2c_master_transmit_receive(
        bme280_dev,
        &reg,
        1,
        data,
        len,
        1000
    );
}

// ================= BME280 CALIBRATION =================

static void bme280_read_calibration(void)
{
    uint8_t calib1[26];
    uint8_t calib2[7];

    ESP_ERROR_CHECK(
        bme280_read_registers(
            0x88,
            calib1,
            sizeof(calib1)
        )
    );

    ESP_ERROR_CHECK(
        bme280_read_registers(
            0xE1,
            calib2,
            sizeof(calib2)
        )
    );

    dig_T1 = (uint16_t)((calib1[1] << 8) | calib1[0]);
    dig_T2 = (int16_t)((calib1[3] << 8) | calib1[2]);
    dig_T3 = (int16_t)((calib1[5] << 8) | calib1[4]);

    dig_P1 = (uint16_t)((calib1[7] << 8) | calib1[6]);
    dig_P2 = (int16_t)((calib1[9] << 8) | calib1[8]);
    dig_P3 = (int16_t)((calib1[11] << 8) | calib1[10]);
    dig_P4 = (int16_t)((calib1[13] << 8) | calib1[12]);
    dig_P5 = (int16_t)((calib1[15] << 8) | calib1[14]);
    dig_P6 = (int16_t)((calib1[17] << 8) | calib1[16]);
    dig_P7 = (int16_t)((calib1[19] << 8) | calib1[18]);
    dig_P8 = (int16_t)((calib1[21] << 8) | calib1[20]);
    dig_P9 = (int16_t)((calib1[23] << 8) | calib1[22]);

    dig_H1 = calib1[25];

    dig_H2 = (int16_t)((calib2[1] << 8) | calib2[0]);
    dig_H3 = calib2[2];

    dig_H4 = (int16_t)(
        ((int16_t)calib2[3] << 4) |
        (calib2[4] & 0x0F)
    );

    dig_H5 = (int16_t)(
        ((int16_t)calib2[5] << 4) |
        (calib2[4] >> 4)
    );

    dig_H6 = (int8_t)calib2[6];
}

// ================= BME280 COMPENSATION =================

static float compensate_temperature(int32_t adc_T)
{
    int32_t var1;
    int32_t var2;

    var1 =
        ((((adc_T >> 3) -
           ((int32_t)dig_T1 << 1))) *
         ((int32_t)dig_T2)) >> 11;

    var2 =
        (((((adc_T >> 4) -
            ((int32_t)dig_T1)) *
           ((adc_T >> 4) -
            ((int32_t)dig_T1))) >> 12) *
         ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;

    int32_t temperature =
        (t_fine * 5 + 128) >> 8;

    return temperature / 100.0f;
}

static float compensate_pressure(int32_t adc_P)
{
    int64_t var1;
    int64_t var2;
    int64_t p;

    var1 = ((int64_t)t_fine) - 128000;

    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 += (var1 * (int64_t)dig_P5) << 17;
    var2 += ((int64_t)dig_P4) << 35;

    var1 =
        ((var1 * var1 * (int64_t)dig_P3) >> 8) +
        ((var1 * (int64_t)dig_P2) << 12);

    var1 =
        (((((int64_t)1) << 47) + var1) *
         (int64_t)dig_P1) >> 33;

    if (var1 == 0) {
        return 0.0f;
    }

    p = 1048576 - adc_P;

    p =
        (((p << 31) - var2) * 3125) /
        var1;

    var1 =
        ((int64_t)dig_P9 *
         (p >> 13) *
         (p >> 13)) >> 25;

    var2 =
        ((int64_t)dig_P8 * p) >> 19;

    p =
        ((p + var1 + var2) >> 8) +
        ((int64_t)dig_P7 << 4);

    return (float)p / 256.0f / 100.0f;
}

static float compensate_humidity(int32_t adc_H)
{
    int32_t v;

    v = t_fine - 76800;

    v =
        (((((adc_H << 14) -
            (((int32_t)dig_H4) << 20) -
            (((int32_t)dig_H5) * v)) +
           16384) >> 15) *

         (((((((v * ((int32_t)dig_H6)) >> 10) *
              (((v * ((int32_t)dig_H3)) >> 11) +
               32768)) >> 10) +
            2097152) *
           ((int32_t)dig_H2) +
           8192) >> 14));

    v =
        v -
        (((((v >> 15) *
            (v >> 15)) >> 7) *
          ((int32_t)dig_H1)) >> 4);

    if (v < 0) {
        v = 0;
    }

    if (v > 419430400) {
        v = 419430400;
    }

    return (v >> 12) / 1024.0f;
}

// ================= BME280 READ =================

static esp_err_t bme280_read_data(
    float* temperature,
    float* humidity,
    float* pressure)
{
    uint8_t data[8];

    esp_err_t err = bme280_read_registers(
        0xF7,
        data,
        sizeof(data)
    );

    if (err != ESP_OK) {
        return err;
    }

    int32_t adc_P =
        ((int32_t)data[0] << 12) |
        ((int32_t)data[1] << 4) |
        ((int32_t)data[2] >> 4);

    int32_t adc_T =
        ((int32_t)data[3] << 12) |
        ((int32_t)data[4] << 4) |
        ((int32_t)data[5] >> 4);

    int32_t adc_H =
        ((int32_t)data[6] << 8) |
        data[7];

    *temperature = compensate_temperature(adc_T);
    *pressure = compensate_pressure(adc_P);
    *humidity = compensate_humidity(adc_H);

    return ESP_OK;
}

// ================= BME280 INIT =================

static void bme280_init(void)
{
    i2c_master_bus_config_t bus_config = {};

    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num = I2C_SDA;
    bus_config.scl_io_num = I2C_SCL;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(
        i2c_new_master_bus(
            &bus_config,
            &i2c_bus
        )
    );

    i2c_device_config_t dev_config = {};

    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = BME280_ADDR;
    dev_config.scl_speed_hz = 100000;

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(
            i2c_bus,
            &dev_config,
            &bme280_dev
        )
    );

    uint8_t chip_id = 0;

    ESP_ERROR_CHECK(
        bme280_read_registers(
            0xD0,
            &chip_id,
            1
        )
    );

    ESP_LOGI(
        TAG,
        "BME280 Chip ID: 0x%02X",
        chip_id
    );

    if (chip_id != 0x60) {
        ESP_LOGE(
            TAG,
            "BME280 не знайдено"
        );
        return;
    }

    bme280_read_calibration();

    // Humidity oversampling x1
    ESP_ERROR_CHECK(
        bme280_write_register(
            0xF2,
            0x01
        )
    );

    // Temperature x1, Pressure x1, Normal mode
    ESP_ERROR_CHECK(
        bme280_write_register(
            0xF4,
            0x27
        )
    );

    // Standby 1000 ms
    ESP_ERROR_CHECK(
        bme280_write_register(
            0xF5,
            0xA0
        )
    );

    ESP_LOGI(TAG, "BME280 initialized");
}

// ================= WIFI EVENTS =================

static void wifi_event_handler(
    void* arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void* event_data)
{
    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START) {

        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {

        ESP_LOGW(
            TAG,
            "Wi-Fi втрачено. Перепідключення..."
        );

        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT &&
             event_id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t* event =
            (ip_event_got_ip_t*)event_data;

        ESP_LOGI(
            TAG,
            "IP: " IPSTR,
            IP2STR(&event->ip_info.ip)
        );

        if (!is_mqtt_started) {
            mqtt_app_start();
            is_mqtt_started = true;
        }
    }
}

// ================= MQTT EVENTS =================

static void mqtt_event_handler(
    void* handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void* event_data)
{
    esp_mqtt_event_handle_t event =
        (esp_mqtt_event_handle_t)event_data;

    esp_mqtt_client_handle_t client =
        event->client;

    switch ((esp_mqtt_event_id_t)event_id) {

        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(
                TAG,
                "MQTT підключено"
            );

            /*
             * Підписка на дані інших студентів.
             *
             * "+" означає будь-яке ім'я студента.
             *
             * Наприклад:
             * EMB1/student1/temp
             * EMB1/student2/hum
             */
            esp_mqtt_client_subscribe(
                client,
                "EMB1/+/temp",
                0
            );

            esp_mqtt_client_subscribe(
                client,
                "EMB1/+/hum",
                0
            );

            esp_mqtt_client_subscribe(
                client,
                "EMB1/+/pres",
                0
            );

            ESP_LOGI(
                TAG,
                "Підписано на топіки студентів"
            );

            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(
                TAG,
                "MQTT відключено"
            );
            break;

        case MQTT_EVENT_DATA:

            ESP_LOGI(
                TAG,
                "--- MQTT MESSAGE ---"
            );

            printf(
                "Topic: %.*s\r\n",
                event->topic_len,
                event->topic
            );

            printf(
                "Data: %.*s\r\n",
                event->data_len,
                event->data
            );

            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(
                TAG,
                "MQTT error"
            );
            break;

        default:
            break;
    }
}

// ================= MQTT INIT =================

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {};

    mqtt_cfg.broker.address.uri =
        BROKER_URI;

    mqtt_client =
        esp_mqtt_client_init(&mqtt_cfg);

    esp_mqtt_client_register_event(
        mqtt_client,
        MQTT_EVENT_ANY,
        mqtt_event_handler,
        NULL
    );

    esp_mqtt_client_start(
        mqtt_client
    );

    // Запускаємо таск сенсора
    xTaskCreate(
        sensor_task,
        "sensor_task",
        4096,
        NULL,
        5,
        NULL
    );
}

// ================= SENSOR TASK =================

static void sensor_task(void* arg)
{
    char topic_temp[64];
    char topic_hum[64];
    char topic_pres[64];

    snprintf(
        topic_temp,
        sizeof(topic_temp),
        "EMB1/%s/temp",
        STUDENT_NAME
    );

    snprintf(
        topic_hum,
        sizeof(topic_hum),
        "EMB1/%s/hum",
        STUDENT_NAME
    );

    snprintf(
        topic_pres,
        sizeof(topic_pres),
        "EMB1/%s/pres",
        STUDENT_NAME
    );

    while (true) {

        float temperature;
        float humidity;
        float pressure;

        esp_err_t err = bme280_read_data(
            &temperature,
            &humidity,
            &pressure
        );

        if (err == ESP_OK) {

            ESP_LOGI(
                TAG,
                "T: %.2f C | H: %.2f %% | P: %.2f hPa",
                temperature,
                humidity,
                pressure
            );

            if (mqtt_client != nullptr) {

                char payload[32];

                // Temperature
                snprintf(
                    payload,
                    sizeof(payload),
                    "%.2f",
                    temperature
                );

                esp_mqtt_client_publish(
                    mqtt_client,
                    topic_temp,
                    payload,
                    0,
                    0,
                    0
                );

                // Humidity
                snprintf(
                    payload,
                    sizeof(payload),
                    "%.2f",
                    humidity
                );

                esp_mqtt_client_publish(
                    mqtt_client,
                    topic_hum,
                    payload,
                    0,
                    0,
                    0
                );

                // Pressure
                snprintf(
                    payload,
                    sizeof(payload),
                    "%.2f",
                    pressure
                );

                esp_mqtt_client_publish(
                    mqtt_client,
                    topic_pres,
                    payload,
                    0,
                    0,
                    0
                );

                ESP_LOGI(
                    TAG,
                    "MQTT data published"
                );
            }
        }
        else {
            ESP_LOGE(
                TAG,
                "BME280 read error: %s",
                esp_err_to_name(err)
            );
        }

        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}

// ================= WIFI INIT =================

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(
        esp_netif_init()
    );

    ESP_ERROR_CHECK(
        esp_event_loop_create_default()
    );

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg =
        WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(
        esp_wifi_init(&cfg)
    );

    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &wifi_event_handler,
            NULL,
            NULL
        )
    );

    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &wifi_event_handler,
            NULL,
            NULL
        )
    );

    wifi_config_t wifi_config = {};

    strncpy(
        (char*)wifi_config.sta.ssid,
        WIFI_SSID,
        sizeof(wifi_config.sta.ssid) - 1
    );

    strncpy(
        (char*)wifi_config.sta.password,
        WIFI_PASS,
        sizeof(wifi_config.sta.password) - 1
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_mode(
            WIFI_MODE_STA
        )
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_STA,
            &wifi_config
        )
    );

    ESP_ERROR_CHECK(
        esp_wifi_start()
    );
}

// ================= MAIN =================

extern "C" void app_main(void)
{
    esp_err_t ret =
        nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_ERROR_CHECK(
            nvs_flash_erase()
        );

        ret =
            nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    ESP_LOGI(
        TAG,
        "Starting ESP32-S3..."
    );

    // Спочатку сенсор
    bme280_init();

    // Потім мережа
    wifi_init_sta();
}