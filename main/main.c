#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "esp_http_client.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "soc/gpio_reg.h"

// ── WiFi credentials ──────────────────────────────────────────────────────────
#define WIFI_SSID       "Redmi"
#define WIFI_PASS       "12345678ok"
#define WIFI_MAX_RETRY  10

// ── Firebase configuration ────────────────────────────────────────────────────
// Replace these with your actual Firebase project values.
// FIREBASE_HOST    : your Realtime Database URL (no trailing slash, no https://)
// FIREBASE_API_KEY : your Web API key (used for anonymous sign-in via Identity Toolkit)
#define FIREBASE_HOST     "embeddedpro-573f6-default-rtdb.firebaseio.com"
#define FIREBASE_API_KEY  "AIzaSyDDr6BjgGft1NnRrvOHGqe84ZWzmY_LiRI"
#define FIREBASE_BASE     "/smarthome/room001"

// ── Firebase anonymous-auth token state ──────────────────────────────────────
static char fb_id_token[1200]         = {0};
static TickType_t fb_token_obtained_at = 0;
#define TOKEN_REFRESH_INTERVAL_MS  (55 * 60 * 1000)   // refresh every 55 min

// ── Firebase paths ────────────────────────────────────────────────────────────
#define PATH_TEMP       FIREBASE_BASE "/temperature.json"
#define PATH_HUM        FIREBASE_BASE "/humidity.json"
#define PATH_ROOM       FIREBASE_BASE "/room.json"
#define PATH_COUNT      FIREBASE_BASE "/count.json"
#define PATH_COMMAND    FIREBASE_BASE "/command.json"

// ── Pin definitions ───────────────────────────────────────────────────────────
#define DHT_PIN         GPIO_NUM_26
#define PIR_PIN         GPIO_NUM_25
#define IR_OUTER_PIN    GPIO_NUM_13
#define IR_INNER_PIN    GPIO_NUM_14
#define RELAY_PIN       GPIO_NUM_23

// ── Timing ────────────────────────────────────────────────────────────────────
#define POLL_PERIOD_MS          10
#define DEBOUNCE_SAMPLES        5
#define SEQUENCE_TIMEOUT_MS     3000
#define COOLDOWN_MS             2000
#define DHT_INTERVAL_MS         5000
#define COMMAND_POLL_MS         3000    // how often ESP32 checks Firebase for a new command

static const char *TAG = "SmartHome";

// ── Firebase root CA (Google Trust Services) ──────────────────────────────────
// This allows esp_http_client to verify the Firebase TLS certificate.
// Update this cert if it expires (valid until ~2036).
static const char FIREBASE_ROOT_CA[] =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIDdTCCAl2gAwIBAgILBAAAAAABFUtaw5QwDQYJKoZIhvcNAQEFBQAwVzELMAkG\n"
    "A1UEBhMCQkUxGTAXBgNVBAoTEEdsb2JhbFNpZ24gbnYtc2ExEDAOBgNVBAsTB1Jv\n"
    "b3QgQ0ExGzAZBgNVBAMTEkdsb2JhbFNpZ24gUm9vdCBDQTAeFw05ODA5MDExMjAw\n"
    "MDBaFw0yODAxMjgxMjAwMDBaMFcxCzAJBgNVBAYTAkJFMRkwFwYDVQQKExBHbG9i\n"
    "YWxTaWduIG52LXNhMRAwDgYDVQQLEwdSb290IENBMRswGQYDVQQDExJHbG9iYWxT\n"
    "aWduIFJvb3QgQ0EwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDaDuaZ\n"
    "jc6j40+Kfvvxi4Mla+pIH/EqsLmVEQS98GPR4mdmzxzdzxtIK+6NiY6arymAZavp\n"
    "xy0Sy6scTHAHoT0KMM0VjU/43dSMUBUc71DuxC73/OlS8pF94G3VNTCOXkNz8kHp\n"
    "1Wrjsok6Vjk4bwY8iGlbKk3Fp1S4bInMm/k8yuX9ifUSPJJ4ltbcdG6TRGHRjcdG\n"
    "snUOhugZitVtbNV4FpWi6cgKOOvyJBNPc1STE4U6G7weNLWLBYy5d4ux2x8gkasJ\n"
    "U26Qzns3dLlwR5EiUWMWea6xrkEmCMgZK9FGqkjWZCrXgzT/LCrBbBlDSgeF59N8\n"
    "9iFo7+ryUp9/k5DPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNVHRMBAf8E\n"
    "BTADAQH/MB0GA1UdDgQWBBRge2YaRQ2XyolQL30EzTSo//z9SzANBgkqhkiG9w0B\n"
    "AQUFAAOCAQEA1nPnfE920I2/7LqivjTFKDK1fPxsnCwrvQmeU79rXqoRSLblCKOz\n"
    "yj1hTdNGCbM+w6DjY1Ub8rrvrTnhQ7k4o+YviiY776BQVvnGCv04zcQLcFGUl5gE\n"
    "38NflNUVyRRBnMRddWQVDf9VMOyGj/8N7yy5Y0b2qvzfvGn9LhJIZJrglfCm7ymP\n"
    "AbEVtQwdpf5pLGkkeB6zpxxxYu7KyJesF12KwvhHhm4qxFYxldBniYUr+WymXUad\n"
    "DKqC5JlR3XC321Y9YeRq4VzW9v493kHMB65jUr9TU/Qr6cf9tveCX4XSQRjbgbME\n"
    "HMUfpIBvFSDJ3gyICh3WZlXi/EjJKSZp4A==\n"
    "-----END CERTIFICATE-----\n";

// ── WiFi ──────────────────────────────────────────────────────────────────────
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
static int wifi_retry_count = 0;

// ── Global state ──────────────────────────────────────────────────────────────
static volatile bool room_occupied = false;
static int           people_count  = 0;

// ── Directional detection state machine ───────────────────────────────────────
typedef enum {
    DETECT_IDLE,
    DETECT_OUTER_FIRST,
    DETECT_INNER_FIRST,
    DETECT_AWAIT_PIR,
} detect_state_t;

static detect_state_t detect_state      = DETECT_IDLE;
static TickType_t     state_entered_at  = 0;
static TickType_t     last_detection_at = 0;

static bool outer_broken = false, inner_broken = false;
static int  outer_hi = 0, outer_lo = 0;
static int  inner_hi = 0, inner_lo = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Relay
// ─────────────────────────────────────────────────────────────────────────────
static void relay_set(bool on)
{
    gpio_set_level(RELAY_PIN, on ? 1 : 0);
    ESP_LOGI("relay", "Lamp %s", on ? "ON" : "OFF");
}

// ─────────────────────────────────────────────────────────────────────────────
// DHT22 bit-bang driver (unchanged — timing-critical, no HAL)
// ─────────────────────────────────────────────────────────────────────────────
#define _DHT_BIT(p)   (1U << ((p) & 31U))
#define DHT_READ(p)   (((REG_READ(GPIO_IN_REG))  >> (p)) & 1U)
#define DHT_HIGH(p)   REG_WRITE(GPIO_OUT_W1TS_REG,  _DHT_BIT(p))
#define DHT_LOW(p)    REG_WRITE(GPIO_OUT_W1TC_REG,  _DHT_BIT(p))
#define DHT_OUTPUT(p) REG_WRITE(GPIO_ENABLE_W1TS_REG, _DHT_BIT(p))
#define DHT_INPUT(p)  REG_WRITE(GPIO_ENABLE_W1TC_REG, _DHT_BIT(p))

typedef struct { float temperature; float humidity; bool valid; } dht_data_t;

static dht_data_t dht_read(int pin)
{
    dht_data_t result = {0.0f, 0.0f, false};
    uint8_t data[5] = {0};

    DHT_OUTPUT(pin); DHT_LOW(pin);
    vTaskDelay(pdMS_TO_TICKS(20));

    portDISABLE_INTERRUPTS();
    DHT_HIGH(pin); esp_rom_delay_us(40); DHT_INPUT(pin);

    int t = 0;
    while (DHT_READ(pin) == 1) { esp_rom_delay_us(1); if (++t > 200) goto done; }
    t = 0;
    while (DHT_READ(pin) == 0) { esp_rom_delay_us(1); if (++t > 200) goto done; }
    t = 0;
    while (DHT_READ(pin) == 1) { esp_rom_delay_us(1); if (++t > 200) goto done; }

    for (int i = 0; i < 40; i++) {
        t = 0;
        while (DHT_READ(pin) == 0) { esp_rom_delay_us(1); if (++t > 100) goto done; }
        esp_rom_delay_us(35);
        if (DHT_READ(pin) == 1) data[i / 8] |= (1U << (7 - (i % 8)));
        t = 0;
        while (DHT_READ(pin) == 1) { esp_rom_delay_us(1); if (++t > 150) goto done; }
    }

    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) goto done;
    result.humidity    = ((data[0] << 8) | data[1]) * 0.1f;
    result.temperature = (((data[2] & 0x7F) << 8) | data[3]) * 0.1f;
    if (data[2] & 0x80) result.temperature = -result.temperature;
    result.valid = true;

done:
    portENABLE_INTERRUPTS();
    DHT_OUTPUT(pin); DHT_HIGH(pin);
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// IR receivers
// ─────────────────────────────────────────────────────────────────────────────
static void ir_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << IR_OUTER_PIN) | (1ULL << IR_INNER_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

static bool debounce_beam(int gpio, int *hi, int *lo, bool *broken)
{
    int level = gpio_get_level(gpio);
    if (level) { (*hi)++; *lo = 0; }
    else        { (*lo)++; *hi = 0; }
    bool prev = *broken;
    if (!*broken && *hi >= DEBOUNCE_SAMPLES) *broken = true;
    if ( *broken && *lo >= DEBOUNCE_SAMPLES) *broken = false;
    return (!prev && *broken);
}

// ─────────────────────────────────────────────────────────────────────────────
// Firebase HTTP helpers
// ─────────────────────────────────────────────────────────────────────────────

// Shared response buffer — sized for the largest response (anonymous sign-in ~2 KB)
static char http_response_buf[2048];
static int  http_response_len = 0;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        int copy = evt->data_len;
        if (http_response_len + copy >= (int)sizeof(http_response_buf) - 1)
            copy = sizeof(http_response_buf) - 1 - http_response_len;
        if (copy > 0) {
            memcpy(http_response_buf + http_response_len, evt->data, copy);
            http_response_len += copy;
            http_response_buf[http_response_len] = '\0';
        }
    }
    return ESP_OK;
}

static bool firebase_signin_anonymous(void)
{
    char url[128];
    snprintf(url, sizeof(url),
        "https://identitytoolkit.googleapis.com/v1/accounts:signUp?key=%s",
        FIREBASE_API_KEY);

    const char *body = "{\"returnSecureToken\":true}";

    http_response_len = 0;
    http_response_buf[0] = '\0';

    esp_http_client_config_t cfg = {
        .url           = url,
        .method        = HTTP_METHOD_POST,
        .cert_pem      = FIREBASE_ROOT_CA,
        .event_handler = http_event_handler,
        .timeout_ms    = 10000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept-Encoding", "identity"); // force plain JSON, no gzip
    esp_http_client_set_post_field(client, body, strlen(body));
    esp_err_t err = esp_http_client_perform(client);
    int status    = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || status != 200) {
        ESP_LOGE(TAG, "Anonymous sign-in failed: %s (HTTP %d) buf='%.80s'",
                 esp_err_to_name(err), status, http_response_buf);
        return false;
    }

    char *p = strstr(http_response_buf, "\"idToken\":");
    if (!p) {
        ESP_LOGE(TAG, "Sign-in: idToken field not found (len=%d buf='%.80s')",
                 http_response_len, http_response_buf);
        return false;
    }
    p += strlen("\"idToken\":");
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (*p != '"') {
        ESP_LOGE(TAG, "Sign-in: unexpected char after idToken colon: 0x%02x", (unsigned char)*p);
        return false;
    }
    p++; // skip opening quote
    char *end = strchr(p, '"');
    if (!end) {
        ESP_LOGE(TAG, "Sign-in: idToken closing quote missing — response truncated?");
        return false;
    }
    int len = end - p;
    if (len >= (int)sizeof(fb_id_token)) {
        ESP_LOGE(TAG, "Sign-in: idToken too long (%d bytes, max %d)", len, (int)sizeof(fb_id_token) - 1);
        return false;
    }
    strncpy(fb_id_token, p, len);
    fb_id_token[len] = '\0';

    fb_token_obtained_at = xTaskGetTickCount();
    ESP_LOGI(TAG, "Firebase: anonymous sign-in OK (token %d bytes).", len);
    return true;
}

/**
 * firebase_put() — writes a JSON value to a Firebase path via HTTP PATCH.
 * Using PATCH on the specific leaf node (e.g. /temperature.json) is equivalent
 * to a targeted PUT but avoids overwriting sibling nodes.
 *
 * @param path   e.g. "/smarthome/room001/temperature.json"
 * @param json   e.g. "24.5"  or  "\"OCCUPIED\""
 */
static void firebase_put(const char *path, const char *json)
{
    if ((xTaskGetTickCount() - fb_token_obtained_at) >= pdMS_TO_TICKS(TOKEN_REFRESH_INTERVAL_MS))
        firebase_signin_anonymous();

    static char url[1400];
    snprintf(url, sizeof(url), "https://%s%s?auth=%s", FIREBASE_HOST, path, fb_id_token);

    esp_http_client_config_t cfg = {
        .url            = url,
        .method         = HTTP_METHOD_PUT,
        .cert_pem       = FIREBASE_ROOT_CA,
        .event_handler  = http_event_handler,
        .timeout_ms     = 8000,
        .buffer_size_tx = 1200,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json, strlen(json));

    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "firebase_put(%s) failed: %s", path, esp_err_to_name(err));

    esp_http_client_cleanup(client);
}

/**
 * firebase_get() — reads a value from a Firebase path.
 * Stores the raw JSON response in out_buf (null-terminated).
 * Returns true on HTTP 200, false otherwise.
 */
static bool firebase_get(const char *path, char *out_buf, int out_size)
{
    if ((xTaskGetTickCount() - fb_token_obtained_at) >= pdMS_TO_TICKS(TOKEN_REFRESH_INTERVAL_MS))
        firebase_signin_anonymous();

    static char url[1400];
    snprintf(url, sizeof(url), "https://%s%s?auth=%s", FIREBASE_HOST, path, fb_id_token);

    http_response_len = 0;
    http_response_buf[0] = '\0';

    esp_http_client_config_t cfg = {
        .url            = url,
        .method         = HTTP_METHOD_GET,
        .cert_pem       = FIREBASE_ROOT_CA,
        .event_handler  = http_event_handler,
        .timeout_ms     = 8000,
        .buffer_size_tx = 1200,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_err_t err = esp_http_client_perform(client);
    int status    = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || status != 200) {
        ESP_LOGW(TAG, "firebase_get(%s) failed (status %d)", path, status);
        return false;
    }

    strncpy(out_buf, http_response_buf, out_size - 1);
    out_buf[out_size - 1] = '\0';
    return true;
}

/**
 * firebase_delete() — sets a node to null (clears a command after processing).
 */
static void firebase_delete(const char *path)
{
    static char url[1400];
    snprintf(url, sizeof(url), "https://%s%s?auth=%s", FIREBASE_HOST, path, fb_id_token);

    esp_http_client_config_t cfg = {
        .url            = url,
        .method         = HTTP_METHOD_DELETE,
        .cert_pem       = FIREBASE_ROOT_CA,
        .timeout_ms     = 8000,
        .buffer_size_tx = 1200,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

// ─────────────────────────────────────────────────────────────────────────────
// Firebase publish helpers (thin wrappers for clean call sites)
// ─────────────────────────────────────────────────────────────────────────────
static void pub_temperature(float t)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", t);
    firebase_put(PATH_TEMP, buf);
}

static void pub_humidity(float h)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", h);
    firebase_put(PATH_HUM, buf);
}

static void pub_room(const char *status)
{
    // Firebase string values must be quoted JSON strings
    char buf[32];
    snprintf(buf, sizeof(buf), "\"%s\"", status);
    firebase_put(PATH_ROOM, buf);
}

static void pub_count(int count)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%d", count);
    firebase_put(PATH_COUNT, buf);
}

// ─────────────────────────────────────────────────────────────────────────────
// Command polling — called from the main loop every COMMAND_POLL_MS
// ─────────────────────────────────────────────────────────────────────────────
static void poll_command(void)
{
    char raw[128];
    if (!firebase_get(PATH_COMMAND, raw, sizeof(raw))) return;

    // Firebase returns JSON: "LIGHTS_ON" (with quotes) or null
    if (strcmp(raw, "null") == 0 || strlen(raw) < 3) return;

    // Strip surrounding quotes: "LIGHTS_ON" → LIGHTS_ON
    char cmd[64] = {0};
    int len = strlen(raw);
    if (raw[0] == '"' && raw[len - 1] == '"') {
        strncpy(cmd, raw + 1, len - 2);
        cmd[len - 2] = '\0';
    } else {
        strncpy(cmd, raw, sizeof(cmd) - 1);
    }

    ESP_LOGI(TAG, "Command received: %s", cmd);

    if (strcmp(cmd, "LIGHTS_ON") == 0) {
        relay_set(true);
    } else if (strcmp(cmd, "LIGHTS_OFF") == 0) {
        relay_set(false);
    } else if (strcmp(cmd, "STATUS") == 0) {
        pub_room(room_occupied ? "OCCUPIED" : "EMPTY");
        pub_count(people_count);
    }

    // Clear the command node so it isn't processed again
    firebase_delete(PATH_COMMAND);
}

// ─────────────────────────────────────────────────────────────────────────────
// Directional detection (unchanged logic — timing stays on-device)
// ─────────────────────────────────────────────────────────────────────────────
static void detection_poll(void)
{
    TickType_t now = xTaskGetTickCount();
    if ((now - last_detection_at) < pdMS_TO_TICKS(COOLDOWN_MS)) return;

    bool outer_just_broke = debounce_beam(IR_OUTER_PIN, &outer_hi, &outer_lo, &outer_broken);
    bool inner_just_broke = debounce_beam(IR_INNER_PIN, &inner_hi, &inner_lo, &inner_broken);

    switch (detect_state) {

        case DETECT_IDLE:
            if (outer_just_broke) {
                detect_state = DETECT_OUTER_FIRST;
                state_entered_at = now;
                ESP_LOGI(TAG, "Outer beam broke — watching for inner");
            } else if (inner_just_broke) {
                detect_state = DETECT_INNER_FIRST;
                state_entered_at = now;
                ESP_LOGI(TAG, "Inner beam broke — watching for outer");
            }
            break;

        case DETECT_OUTER_FIRST:
            if ((now - state_entered_at) >= pdMS_TO_TICKS(SEQUENCE_TIMEOUT_MS)) {
                detect_state = DETECT_IDLE;
            } else if (inner_just_broke) {
                detect_state = DETECT_AWAIT_PIR;
                state_entered_at = now;
                ESP_LOGI(TAG, "outer→inner — awaiting PIR confirmation");
            }
            break;

        case DETECT_INNER_FIRST:
            if ((now - state_entered_at) >= pdMS_TO_TICKS(SEQUENCE_TIMEOUT_MS)) {
                detect_state = DETECT_IDLE;
            } else if (outer_just_broke) {
                // EXIT confirmed
                people_count = (people_count > 0) ? people_count - 1 : 0;
                pub_count(people_count);
                if (people_count == 0 && room_occupied) {
                    room_occupied = false;
                    pub_room("EMPTY");
                    relay_set(false);
                }
                ESP_LOGI(TAG, "<<< EXIT (people: %d)", people_count);
                last_detection_at = now;
                detect_state = DETECT_IDLE;
            }
            break;

        case DETECT_AWAIT_PIR:
            if ((now - state_entered_at) >= pdMS_TO_TICKS(SEQUENCE_TIMEOUT_MS)) {
                ESP_LOGI(TAG, "PIR timeout — entrance not confirmed");
                detect_state = DETECT_IDLE;
            } else if (gpio_get_level(PIR_PIN) == 1) {
                // ENTRANCE confirmed
                people_count++;
                pub_count(people_count);
                if (!room_occupied) {
                    room_occupied = true;
                    pub_room("OCCUPIED");
                    relay_set(true);
                }
                ESP_LOGI(TAG, ">>> ENTRANCE confirmed (people: %d)", people_count);
                last_detection_at = now;
                detect_state = DETECT_IDLE;
            }
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// WiFi
// ─────────────────────────────────────────────────────────────────────────────
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *event_data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            wifi_retry_count++;
            ESP_LOGW(TAG, "WiFi retry %d/%d", wifi_retry_count, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi connected. IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        wifi_retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t h_any, h_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        wifi_event_handler, NULL, &h_any));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        wifi_event_handler, NULL, &h_ip));

    wifi_config_t wifi_cfg = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    xEventGroupWaitBits(wifi_event_group,
                        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                        pdFALSE, pdFALSE, portMAX_DELAY);
}

// ─────────────────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────────────────
void app_main(void)
{
    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Relay — lamp OFF at boot
    gpio_config_t relay_cfg = {
        .pin_bit_mask = (1ULL << RELAY_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&relay_cfg);
    relay_set(true);                        // self-test flash
    vTaskDelay(pdMS_TO_TICKS(500));
    relay_set(false);

    // PIR
    gpio_config_t pir_cfg = {
        .pin_bit_mask = (1ULL << PIR_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&pir_cfg);

    // DHT22
    gpio_set_pull_mode(DHT_PIN, GPIO_PULLUP_ONLY);
    DHT_OUTPUT(DHT_PIN);
    DHT_HIGH(DHT_PIN);

    // IR
    ir_init();
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "=== Smart Home — Firebase REST ===");
    wifi_init();
    firebase_signin_anonymous();
    ESP_LOGI(TAG, "System ready.");

    TickType_t last_dht_tick     = xTaskGetTickCount();
    TickType_t last_command_tick = xTaskGetTickCount();

    while (1) {
        TickType_t now = xTaskGetTickCount();

        // ── Directional detection (IR + PIR) — every POLL_PERIOD_MS ──────────
        detection_poll();

        // ── DHT22 — every DHT_INTERVAL_MS ─────────────────────────────────────
        if ((now - last_dht_tick) >= pdMS_TO_TICKS(DHT_INTERVAL_MS)) {
            last_dht_tick = now;
            dht_data_t dht = dht_read(DHT_PIN);
            if (dht.valid) {
                pub_temperature(dht.temperature);
                pub_humidity(dht.humidity);
                ESP_LOGI(TAG, "Temp: %.1f C | Hum: %.1f %% | Room: %s | Count: %d",
                         dht.temperature, dht.humidity,
                         room_occupied ? "OCCUPIED" : "EMPTY", people_count);
            }
        }

        // ── Command polling — every COMMAND_POLL_MS ───────────────────────────
        if ((now - last_command_tick) >= pdMS_TO_TICKS(COMMAND_POLL_MS)) {
            last_command_tick = now;
            poll_command();
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}