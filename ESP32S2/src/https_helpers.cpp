#include <WiFi.h>
#include <esp_http_client.h>

#include "Logging.h"
#include "utils.h"

bool post_data(const String &url, const char *key, const char *email, const String &payload)
{
    bool result = false;
    LOG_INFO(F("HTTP: Send JSON POST request"));
    LOG_INFO(F("HTTP: URL:") << url);
    LOG_INFO(F("HTTP: Body:") << payload);
    
    esp_http_client_config_t config = {
        .url = url.c_str(),
        .method = HTTP_METHOD_POST,
        .timeout_ms = SERVER_TIMEOUT,
        .event_handler = NULL,
        //.cert_pem = NULL, // Skip certificate verification
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_header(client, "Content-Type", "application/json");
    if (key[0])
    {
        esp_http_client_set_header(client, "Waterius-Token", key);
    }
    if (email[0])
    {
        esp_http_client_set_header(client, "Waterius-Email", email);
    }

    esp_err_t err = esp_http_client_perform(client);
    result = esp_http_client_get_status_code(client);

    LOG_INFO(F("HTTP: Response code: ") << result);
    if (err == ESP_OK) {
        int content_length = esp_http_client_get_content_length(client);
        char *buffer = (char *)malloc(1024);
        esp_http_client_read_response(client, buffer, 1023);
        String resp(buffer);
        LOG_INFO(F("HTTP: Response body: ") << resp);
    } else {
        LOG_ERROR(F("HTTP: request failed: ") << esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);

    return result;
}
