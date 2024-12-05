
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "Logging.h"
#include "config.h"
#include "master_i2c.h"
#include "senders/sender_waterius.h"
#include "senders/sender_http.h"
#include "senders/sender_mqtt.h"
#include "portal/active_point.h"
#include "voltage.h"
#include "utils.h"
#include "porting.h"
#include "json.h"
#include "Ticker.h"
#include "sync_time.h"
#include "wifi_helpers.h"
#include "config.h"
#include "board.h"

/*#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"*/

MasterI2C masterI2C;  // Для общения с Attiny85 по i2c
SlaveData data;       // Данные от Attiny85
Settings sett;        // Настройки соединения и предыдущие показания из EEPROM
CalculatedData cdata; // вычисляемые данные

Ticker voltage_ticker;
static bool usb_power_en = false;


//=====================================================================================
// Выполняется однократно при включении
//=====================================================================================

void setup()
{
	// Установка пинов
	initialize_pins();
	usb_power_en = !gpio_get_level(BATT_EN);
    gpio_set_level(LED_S2, 1);//usb_power_en);

    // Инициализация портов
	USBSerial.begin(115200);        // Консоль приложения
	Serial0.begin(115200);          // Системная консоль
    LOG_INFO(F("Booted"));
    LOG_INFO(F("Build: ") << __DATE__ << F(" ") << __TIME__);

    bool b = true;
    gpio_set_level(LED_STATE, b);

    int i = 50;
    while (i--) {
        delay(100);
        gpio_set_level(LED_S2, b);
        gpio_set_level(LED_STATE, b);
        b = !b;
    }


    // Определяем причину запуска
    ulp_event_t ulp_event = get_wakeup_event();
    if (ulp_event == ulp_event_t::NONE) {
        // Обычный запуск
        LOG_INFO(F("Initializing ULP"));
        initialize_rtc_pins();
        init_ulp_program();
    } else {
        // Проснулись по сигналу от ULP
        static const char event_text[][16] = { "None", "Time", "Button", "USB" };
        LOG_INFO(F("ULP wakeup, event: ") << event_text[(uint)ulp_event]);
    }

    /*get_voltage()->begin();
    voltage_ticker.attach_ms(300, []()
                             { get_voltage()->update(); }); // через каждые 300 мс будет измеряться напряжение
        */                     
    LOG_INFO(F("Initializing complete"));
}

//=====================================================================================
// Выполняется в цикле после setup
//=====================================================================================

void loop()
{
    usb_power_en = !gpio_get_level(BATT_EN);
    gpio_set_level(LED_S2, usb_power_en);

        bool b = 0;
        int i = 20;
        while (i--) {
            usb_power_en = !gpio_get_level(BATT_EN);
            delay(1000);
            gpio_set_level(LED_S2, b);
            b = !b;
            ulp_data_t data;
            ulp_read(data);
            bool usb_connected = USBSerial;
            LOG_INFO(F("wake ") << data.wake_up_counter << F("/") << data.wake_up_period << F(", battery ") << data.battery_adc_value << F(", usb ") << usb_connected);
        }
    deep_sleep();

    uint8_t mode = TRANSMIT_MODE; // TRANSMIT_MODE;
    bool config_loaded = false;

    // спрашиваем у Attiny85 повод пробуждения и данные true) 
    mode = SETUP_MODE;
    if (true) //masterI2C.getMode(mode) && masterI2C.getSlaveData(data))
    {
        // Загружаем конфигурацию из EEPROM
        config_loaded = load_config(sett);
        sett.mode = mode;
        LOG_INFO(F("Startup mode: ") << mode);

        // Вычисляем текущие показания
        calculate_values(sett, data, cdata);

        if (mode == SETUP_MODE)
        {
            LOG_INFO(F("Entering in setup mode..."));
            // Режим настройки - запускаем точку доступа на 192.168.4.1
            // Запускаем точку доступа с вебсервером

            start_active_point(sett, cdata);

            sett.setup_time = millis();
            sett.setup_finished_counter++;

            store_config(sett);

            wifi_shutdown();

            LOG_INFO(F("Set mode MANUAL_TRANSMIT to attiny"));
            //masterI2C.sendCmd('T'); // Режим "Передача"

            LOG_INFO(F("Restart ESP"));
            LOG_END();

            LOG_INFO(F("Finish setup mode..."));
            ESP.restart();

            return; // сюда не должно дойти никогда
        }

        if (config_loaded)
        {
            if (wifi_connect(sett))
            {
                log_system_info();

                DynamicJsonDocument json_data(JSON_DYNAMIC_MSG_BUFFER);

#ifndef MQTT_DISABLED
                // Подключаемся и подписываемся на мктт
                if (is_mqtt(sett))
                {
                    connect_and_subscribe_mqtt(sett, data, cdata, json_data);
                }
#endif
                // устанавливать время только при использовани хттпс или мктт
                if (is_mqtt(sett) || is_https(sett.waterius_host) || is_https(sett.http_url))
                {
                    if (!sync_ntp_time(sett)) {
                        sett.ntp_error_counter++;
                    }
                }

                voltage_ticker.detach(); // перестаем обновлять перед созданием объекта с данными
                LOG_INFO(F("Free memory: ") << ESP.getFreeHeap());

                // Формироуем JSON
                get_json_data(sett, data, cdata, json_data);

                LOG_INFO(F("Free memory: ") << ESP.getFreeHeap());

#ifndef WATERIUS_RU_DISABLED
                if (send_waterius(sett, json_data))
                {
                    LOG_INFO(F("HTTP: Send OK"));
                }
#endif

#ifndef HTTPS_DISABLED
                if (send_http(sett, json_data))
                {
                    LOG_INFO(F("HTTP: Send OK"));
                }
#endif

#ifndef MQTT_DISABLED
                if (is_mqtt(sett))
                {
                    if (send_mqtt(sett, data, cdata, json_data))
                    {
                        LOG_INFO(F("MQTT: Send OK"));
                    }
                }
                else
                {
                    LOG_INFO(F("MQTT: SKIP"));
                }
#endif
                // Все уже отправили,  wifi не нужен - выключаем
                //wifi_shutdown();

                update_config(sett, data, cdata);

/*                if (!masterI2C.setWakeUpPeriod(sett.set_wakeup))
                {
                    LOG_ERROR(F("Wakeup period wasn't set"));
                }
                else // Разбуди меня через...
                {
                    LOG_INFO(F("Wakeup period, min:") << sett.wakeup_per_min);
                    LOG_INFO(F("Wakeup period (adjusted), min:") << sett.set_wakeup);
                }*/

                store_config(sett);
            }
        }
    }
    LOG_INFO(F("Going to sleep"));
    LOG_END();

    if (!config_loaded)
    {
        delay(500);
        blink_led(3, 1000, 500);
    }

    //masterI2C.sendCmd('Z'); // "Можешь идти спать, attiny"

#ifdef ESP8266
    ESP.deepSleepInstant(0, RF_DEFAULT); // Спим до следущего включения EN. Instant не ждет 92мс
#endif
#ifdef ESP32
    esp_deep_sleep_start();
#endif

}