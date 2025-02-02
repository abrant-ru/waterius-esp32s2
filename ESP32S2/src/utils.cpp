#ifdef ESP8266
#include <ESP8266WiFi.h>
#endif
#ifdef ESP32
#include <WiFi.h>
#include <mbedtls/sha256.h>
#endif
#include "utils.h"
#include "Logging.h"
#include "time.h"
#include "porting.h"
#include "wifi_helpers.h"
#include "setup.h"

/**
 * @brief Форимрует строку с именем устройства
 * в виде ИМЯ_БРЕНДА%-ИДЕНТИФИКАТОР_ЧИПА,
 * пример waterius-12346
 *
 * @return строку с уникальным именем устройства
 */
String get_device_name()
{
	String deviceName = String(BRAND_NAME) + "-" + getChipId();
	return deviceName;
}

/**
 * @brief Форимрует строку с названием точки доступа
 * в виде ИМЯ_БРЕНДА%-ИДЕНТИФИКАТОР_ЧИПА-ВЕРСИЯ_ПРОШИВКИ,
 * пример waterius-12346-0.11.0
 *
 * @return строку с названием точки доступа
 */

String get_ap_name()
{
	return get_device_name() + "-" + String(FIRMWARE_VERSION);
}

/**
 * @brief Преобразует MAC адрес в шестнадцатиричный вид без разделителей. Например AABBCCDDEEFF
 *
 * @return строка с MAC адресом
 */
String get_mac_address_hex()
{
	uint8_t baseMac[6];
	char baseMacChr[13] = {0};
	WiFi.macAddress(baseMac);
	sprintf(baseMacChr, MAC_STR_HEX, baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
	return String(baseMacChr);
}

/**
 * @brief Рассчитывает чексуму объекта настроек
 *  за пример взят следующий код https://github.com/renatoaloi/EtherEncLib/blob/master/checksum.c
 * @param  sett настройки программы
 * @return возвращет чексуму объекта настроек
 */
uint16_t get_checksum(const Settings &sett)
{
	uint8_t *buf = (uint8_t *)&sett;
	uint16_t crc = 0xffff, poly = 0xa001;
	uint16_t i = 0;
	uint16_t len = sizeof(sett) - 2;

	for (i = 0; i < len; i++)
	{
		crc ^= buf[i];
		for (uint8_t j = 0; j < 8; j++)
		{
			if (crc & 0x01)
			{
				crc >>= 1;
				crc ^= poly;
			}
			else
				crc >>= 1;
		}
	}
	LOG_INFO(F("get_checksum crc=") << crc);
	return crc;
}

/**
 * @brief Возвращает протокол ссылки в нижнем регистре
 *
 * @param url ссылка
 * @return строка содержащая название протокола в нижнем регистре http или https
 */
String get_proto(const String &url)
{
	String proto = "";
	int index = url.indexOf(':');
	if (index > 0)
	{
		proto = url.substring(0, index);
		proto.toLowerCase();
	}
	return proto;
}

/**
 * @brief Возвращает признак является ли ссылка https
 *
 * @param url ссылка
 * @return true если ссылка https,
 * @return false если ссылка НЕ https
 *
 */
extern bool is_https(const char *url)
{
	if (url[0])
	{
		String urlStr = String(url);
		return get_proto(urlStr) == PROTO_HTTPS;
	}
	return false;
}

/**
 * @brief убирает в коце строки слэш
 *
 * @param topic стркоа с MQTT топиком
 */
void remove_trailing_slash(String &topic)
{
	if (topic.endsWith(F("/")))
	{
		topic.remove(topic.length() - 1);
	}
}

/**
 * @brief Возвращает признак включена ли передача на сайт Ватериуса
 *
 * @param sett настройки устройства
 * @return true настроена отправка на сайт Ватериус
 * @return false НЕ настроена  отправка на сайт Ватериус
 */
bool is_waterius_site(const Settings &sett)
{
	return sett.waterius_on && sett.waterius_host[0] && sett.waterius_key[0];
}


/**
 * @brief Возвращает признак включена ли передача на веб сервер
 *
 * @param sett настройки устройства
 * @return true настроена отправка на веб сервер
 * @return false НЕ настроена  отправка на веб сервер
 */
bool is_http(const Settings &sett)
{
	return sett.http_on && sett.http_url[0];
}


/**
 * @brief Возвращает признак настроена ли интеграция с MQTT
 *
 * @param sett настройки устройства
 * @return true настроена интеграция с MQTT
 * @return false настроена интеграция с MQTT
 */
bool is_mqtt(const Settings &sett)
{
#ifndef MQTT_DISABLED
	return sett.mqtt_on && sett.mqtt_host[0];
#else
	return false;
#endif
}

/**
 * @brief Возвращает признак настроена ли интеграция с HomeAssistant
 *
 * @param sett настройки устройства
 * @return true настроена интеграция с HomeAssistant
 * @return false настроена интеграция с HomeAssistant
 */
bool is_ha(const Settings &sett)
{
#ifndef MQTT_DISABLED
	return is_mqtt(sett) && sett.mqtt_auto_discovery;
#else
	return false;
#endif
}

/**
 * @brief Возвращает признак будет ли использоваться DHCP
 *
 * @param sett настройки устройства
 * @return true используется DHCP
 * @return false
 */

bool is_dhcp(const Settings &sett)
{
	return sett.dhcp_off == 0;
}

/**
 * @brief Выводит информацию о системе
 *
 */
void log_system_info()
{
	// System info
	LOG_INFO(F("------------ System Info ------------"));
	LOG_INFO(F("Firmware ver: ") << String(FIRMWARE_VERSION));
	LOG_INFO(F("Sketch Size: ") << ESP.getSketchSize());
	LOG_INFO(F("Free Sketch Space: ") << ESP.getFreeSketchSpace());
	LOG_INFO(F("Free memory: ") << ESP.getFreeHeap());
	LOG_INFO(F("Settings size: ") << sizeof(Settings));
	LOG_INFO(F("------------ WiFi Info ------------"));
	LOG_INFO(F("WIFI: SSID: ") << WiFi.SSID());
	LOG_INFO(F("WIFI: BSID: ") << WiFi.BSSIDstr());
	LOG_INFO(F("WIFI: Channel: ") << WiFi.channel());
#ifdef ESP8266
//TODO
	LOG_INFO(F("WIFI: Mode current: ") << wifi_phy_mode_title(WiFi.getPhyMode()));
#endif
	LOG_INFO(F("WIFI: RSSI: ") << WiFi.RSSI() << F("dBm"));
	LOG_INFO(F("------------ IP Info ------------"));
	//TODO LOG_INFO(F("IP: Host name: ") << WiFi.hostname());
	LOG_INFO(F("IP: IP adress: ") << WiFi.localIP().toString());
	LOG_INFO(F("IP: Subnet mask: ") << WiFi.subnetMask());
	LOG_INFO(F("IP: Gateway IP: ") << WiFi.gatewayIP().toString());
	LOG_INFO(F("IP: DNS IP: ") << WiFi.dnsIP(0).toString());
	LOG_INFO(F("IP: MAC Address: ") << WiFi.macAddress());
}

extern void generateSha256Token(char *token, const int token_len, const char *email)
{
	LOG_INFO(F("-- START -- ") << F("Generate SHA256 token from email"));

#ifdef ESP8266
	auto x = BearSSL::HashSHA256();
	if (email != nullptr && strlen(email))
	{
		LOG_INFO(F("E-mail: ") << email);
		x.add(email, strlen(email));
	}

	randomSeed(micros());
	uint32_t salt = rand();
	LOG_INFO(F("salt: ") << salt);
	x.add(&salt, sizeof(salt));

	salt = getChipId();
	x.add(&salt, sizeof(salt));
	LOG_INFO(F("chip id: ") << salt);

	salt = ESP.getFlashChipId();
	x.add(&salt, sizeof(salt));
	LOG_INFO(F("flash id: ") << salt);
	x.end();
	unsigned char *hash = (unsigned char *)x.hash();

	static const char digits[] = "0123456789ABCDEF";

	for (int i = 0; i < x.len() && i < token_len - 1; i += 2, hash++)
	{
		token[i] = digits[*hash >> 4];
		token[i + 1] = digits[*hash & 0xF];
	}
#endif 
#ifdef ESP32
	unsigned char sha256Result[33];
	unsigned char *hash = &sha256Result[0];
	mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
	randomSeed(micros());
	uint32_t salt = rand();
    mbedtls_sha256_update(&ctx, (const unsigned char*)salt, 4);
    mbedtls_sha256_finish(&ctx, sha256Result);
    mbedtls_sha256_free(&ctx);

	static const char digits[] = "0123456789ABCDEF";

	for (int i = 0, j = 0; i < 32 && j < token_len - 1; i++, j+=2)
	{
		token[j] = digits[sha256Result[i] >> 4];
		token[j + 1] = digits[sha256Result[i] & 0xF];
	}
#endif

	LOG_INFO(F("SHA256 token: ") << token);
	LOG_INFO(F("-- END --"));
}

void blink_led(int count, int period, int duty)
{
	pinMode(LED_PIN, OUTPUT);
	for (int i = 0; i < count; i++)
	{
		digitalWrite(LED_PIN, HIGH);
		delay(period - duty);
		digitalWrite(LED_PIN, LOW);
		delay(duty);
	}
}

/**
 * @brief Возвращает тип данных на сервер Ватериуса по названию входа.
 *
 * @param counter_name имя счётчика в интерфейсе ESP
 * @return тип данных на сервере Ватериуса
 */
DataType data_type_by_name(uint8_t counter_name)
{
	switch ((CounterName)counter_name)
	{
	case CounterName::WATER_COLD:
		return DataType::COLD_WATER;

	case CounterName::WATER_HOT:
		return DataType::HOT_WATER;

	case CounterName::ELECTRO:
		return DataType::ELECTRICITY;

	case CounterName::GAS:
		return DataType::GAS_DATA;

	case CounterName::HEAT:
		return DataType::HEATING;

	case CounterName::PORTABLE_WATER:
		return DataType::POTABLE_WATER;

	case CounterName::OTHER:
		return DataType::OTHER_TYPE;
	}
	return DataType::COLD_WATER;
}