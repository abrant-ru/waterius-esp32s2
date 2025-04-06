
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "Logging.h"
#include "config.h"
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
#include "LittleFS.h"

SlaveData data;       // Данные от Attiny85
Settings sett;        // Настройки соединения и предыдущие показания из LittleFS config.bin
CalculatedData cdata; // вычисляемые данные
bool config_loaded = false;

// Debug UART: TX GPIO_40, RX GPIO_38

//=====================================================================================
// Выполняется однократно при включении
//=====================================================================================

void setup()
{
    // Инициализация портов
    LOG_BEGIN(115200);
    LOG_INFO(F("Booted"));
    LOG_INFO(F("Build: ") << __DATE__ << F(" ") << __TIME__);

	// Установка пинов
	initialize_pins();
    gpio_set_level(LED_S2, HIGH);
    gpio_set_level(LED_STATE, HIGH);

    if (!LittleFS.begin()) {
        LOG_ERROR(F("FS: Mounting LittleFS error"));
        ESP.deepSleep(30000000);
    }
    LOG_INFO(F("FS: LittleFS mounted"));

	// true - загрузили конфиг. false - ошибка памяти или инициализация конфига
    config_loaded = load_config(sett);

    // Определяем причину запуска
    get_wakeup_event();
    if (ulp_event == ulp_event_t::NONE) {
        // Обычный запуск
        initialize_rtc_pins();
        init_ulp_program();
    } else {
        // Проснулись по сигналу от ULP
    }
    LOG_INFO(F("mode: ") << sett.mode);

	// Читаем данные
   	board.read();

    autoprint("Initializing complete\r\n");
}

//=====================================================================================
// Выполняется в цикле после setup
//=====================================================================================

void loop()
{
	static unsigned long interval_1s = 0;
	unsigned long now = millis();
	unsigned long elapsed = now - interval_1s;
	if (elapsed > 5000) 
	{
		interval_1s = now;
		// Читаем данные
    	board.read();
		// Обновляем светодиоды
    	gpio_set_level(LED_STATE, 1);
    	gpio_set_level(LED_S2, (board.power == power_t::USB));
		// Пишем в консоль состояние
  		static const char power_text[][16] = { "Battery", "USB" };
    	static const char usb_text[][16] = { "not connected", "connected" };
    	autoprint("wake %u/%u, power %s, voltage %u, usb %s\r\n", board.wake_up_counter, board.wake_up_period, power_text[(uint)board.power], board.battery_voltage, usb_text[board.usb_connected]);
    	autoprint("pulse %u/%u, adc %u/%u\r\n", board.impulses0, board.impulses1, board.ch0.adc_value, board.ch1.adc_value);
		autoprint("input %u\r\n", board.input);
    	if (board.button_time) 
            autoprint("button %u\r\n", board.button_time);
		update_config(sett);
	}

    if (ulp_event == ulp_event_t::TIME)
        sett.mode = TRANSMIT_MODE;
    else if (ulp_event == ulp_event_t::BUTTON_SHORT)
        sett.mode = MANUAL_TRANSMIT_MODE;
    else if (ulp_event == ulp_event_t::BUTTON_LONG)
        sett.mode = SETUP_MODE;
    else
    {
        ulp_event = ulp_event_t::NONE;
    }
    //autoprint("mode %u\r\n", mode);
    
    if (sett.mode != NONE_MODE)
    {   
        LOG_INFO(F("mode: ") << sett.mode);
        // Вычисляем текущие показания
        calculate_values(sett, cdata);
        
        if (sett.mode == SETUP_MODE)
        {
            // Режим настройки - запускаем точку доступа на 192.168.4.1
            // Запускаем точку доступа с вебсервером
            if (active_point() == active_point_state_t::Finish)
            {
                sett.setup_time = millis();
                sett.setup_finished_counter++;

                autoprint("Finish setup mode...");
                store_config(sett);

                wifi_shutdown();

                //autoprint("Restart ESP");
                //ESP.restart();
                
                // Раньше нужен был restart, т.к. 8266 не могла передавать по https после режима AP. 
                // esp32 возможно может и не надо рестартовать! иначе после рестарта нужен mode = TRANSMIT_MODE
                //return; // сюда не должно дойти никогда
            }
        }
        
        if (config_loaded && wifi_connect(sett))
        {
            log_system_info();

            JsonDocument json_data;

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

            update_config(sett);

            /*if (!masterI2C.setWakeUpPeriod(sett.set_wakeup))
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
        
        sett.mode = NONE_MODE;
        ulp_event == ulp_event_t::NONE; // обработали всё
        LOG_INFO(F("mode set NONE"));
    } 

    if (sett.mode == NONE_MODE)
    { 
        // Если задач нету
        if (board.power == power_t::Battery)
        {
            // При питании от батареи - уходим в сон
            gpio_set_level(LED_STATE, 0);
            deep_sleep();
        }
        else
        {
            // При питании от USB - продолжаем работать
        }
    }
}