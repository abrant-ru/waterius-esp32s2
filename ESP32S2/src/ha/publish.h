/**
 * @file publish.h
 * @brief Функции публикации MQTT
 * @version 0.1
 * @date 2023-02-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef HA_PUBLISH_H_
#define HA_PUBLISH_H_

#include <PubSubClient.h>
#include <ArduinoJson.h>

#define MQTT_CHUNK_SIZE 128
#define PUBLISH_MODE_BIG 0
#define PUBLISH_MODE_CHUNKED 1
#define PUBLISH_MODE_SIMPLE 2
#define DEFAULT_PUBLISH_MODE PUBLISH_MODE_BIG

extern void publish(PubSubClient &mqtt_client, String &topic, String &payload, int mode = DEFAULT_PUBLISH_MODE);
extern void publish_big(PubSubClient &mqtt_client, String &topic, String &payload);
extern void publish_simple(PubSubClient &mqtt_client, String &topic, String &payload);
extern void publish_chunked(PubSubClient &mqtt_client, String &topic, String &payload, unsigned int chunk_size=MQTT_CHUNK_SIZE);

#endif