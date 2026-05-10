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
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_rom_sys.h"
#include "lwip/sockets.h"
#include "soc/gpio_reg.h"

// ── WiFi credentials ──────────────────────────────
#define WIFI_SSID       "Mostafa's iPhone"
#define WIFI_PASS       "10001000"
#define WIFI_MAX_RETRY  10

// ── MQTT broker ───────────────────────────────────
#define MQTT_HOST       "172.20.10.2"
#define MQTT_PORT       1883
#define MQTT_USER       "admin"
#define MQTT_PASS       "10001000"
#define MQTT_CLIENT_ID  "ESP32-SmartHome"
#define MQTT_KEEPALIVE  60

// ── MQTT topics ───────────────────────────────────
#define TOPIC_TEMP      "smarthome/room001/temperature"
#define TOPIC_HUM       "smarthome/room001/humidity"
#define TOPIC_ROOM      "smarthome/room001/room"
#define TOPIC_COUNT     "smarthome/room001/count"
#define TOPIC_COMMAND   "smarthome/room001/command"
#define TOPIC_AC        "smarthome/room001/ac"        // NEW: AC status feedback

// ── Pin definitions ───────────────────────────────
#define DHT_PIN         GPIO_NUM_26
#define PIR_PIN         GPIO_NUM_25
#define IR_OUTER_PIN    GPIO_NUM_13   // outer receiver (outside the door)
#define IR_INNER_PIN    GPIO_NUM_14   // inner receiver (inside the door)
#define IR_TX_PIN       GPIO_NUM_18   // NEW: IR LED transmitter (AC control)

// ── IR TX config ──────────────────────────────────
#define IR_RESOLUTION_HZ  1000000    // 1 MHz → 1 µs per tick
#define IR_CARRIER_HZ     38000      // 38 kHz carrier (standard for AC remotes)

// ── Detection timing ──────────────────────────────
#define POLL_PERIOD_MS      10
#define DEBOUNCE_SAMPLES    5
#define SEQUENCE_TIMEOUT_MS 3000
#define COOLDOWN_MS         2000

static const char *TAG = "SmartHome";

static void mqtt_pub(const char *topic, const char *data);

// ── WiFi ──────────────────────────────────────────
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int wifi_retry_count = 0;

// ── MQTT state ────────────────────────────────────
static int               mqtt_sock = -1;
static SemaphoreHandle_t mqtt_tx_mutex = NULL;
static volatile bool     mqtt_connected = false;
static volatile bool     room_occupied  = false;

// ── IR TX state ───────────────────────────────────
static rmt_channel_handle_t ir_tx_channel  = NULL;
static rmt_encoder_handle_t ir_copy_encoder = NULL;
static volatile bool        ac_is_on       = false;

// ── Directional detection state machine ───────────
typedef enum {
    DETECT_IDLE,
    DETECT_OUTER_FIRST,   // outer broke first → potential entrance
    DETECT_INNER_FIRST,   // inner broke first → potential exit
    DETECT_AWAIT_PIR,     // outer then inner broke → waiting for PIR to confirm entrance
} detect_state_t;

static detect_state_t detect_state      = DETECT_IDLE;
static TickType_t     state_entered_at  = 0;
static TickType_t     last_detection_at = 0;
static int            people_count      = 0;

static bool outer_broken = false, inner_broken = false;
static int  outer_hi = 0, outer_lo = 0;
static int  inner_hi = 0, inner_lo = 0;

// ─────────────────────────────────────────────────
// AC IR raw codes (1 µs resolution, 38 kHz carrier)
//
// !! IMPORTANT !!
// These are PLACEHOLDER timings based on a generic NEC-like protocol.
// You MUST replace them with codes captured from your actual AC remote.
// See ir_capture_task() at the bottom of this file for how to do that.
//
// Each rmt_symbol_word_t = one mark+space pair:
//   .level0 = 1 (LED on),  .duration0 = mark  duration in µs
//   .level1 = 0 (LED off), .duration1 = space duration in µs
// ─────────────────────────────────────────────────

// AC ON command — replace all entries below with your captured output
static const rmt_symbol_word_t ac_on_cmd[] = {
    // Header pulse
    {.level0 = 1, .duration0 = 9000, .level1 = 0, .duration1 = 4500},
    // Data bits (example pattern — replace with real bits)
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 1690},
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 560 },
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 1690},
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 560 },
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 1690},
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 1690},
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 560 },
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 560 },
    // End stop bit
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 0   },
};

// AC OFF command — replace all entries below with your captured output
static const rmt_symbol_word_t ac_off_cmd[] = {
    // Header pulse
    {.level0 = 1, .duration0 = 9000, .level1 = 0, .duration1 = 4500},
    // Data bits (example pattern — replace with real bits)
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 560 },
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 1690},
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 560 },
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 1690},
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 560 },
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 560 },
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 1690},
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 1690},
    // End stop bit
    {.level0 = 1, .duration0 = 560,  .level1 = 0, .duration1 = 0   },
};

// ─────────────────────────────────────────────────
// DHT22 bit-bang driver
// Uses direct register access (same as Arduino) to avoid ESP-IDF HAL overhead
// that would break µs-level timing.  Pin must be 0-31.
// Pull-up is configured once in app_main; direction is toggled here via OE bit.
// ─────────────────────────────────────────────────
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

    DHT_OUTPUT(pin);
    DHT_LOW(pin);
    vTaskDelay(pdMS_TO_TICKS(20));

    portDISABLE_INTERRUPTS();
    DHT_HIGH(pin);
    esp_rom_delay_us(40);
    DHT_INPUT(pin);

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
    DHT_OUTPUT(pin);
    DHT_HIGH(pin);
    return result;
}

// ─────────────────────────────────────────────────
// IR TX (AC control via RMT peripheral)
// ─────────────────────────────────────────────────
static void ir_tx_init(void)
{
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num          = IR_TX_PIN,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = IR_RESOLUTION_HZ,
        .mem_block_symbols = 128,   // AC codes are long; 128 symbols to be safe
        .trans_queue_depth = 4,
        .flags.invert_out  = false,
        .flags.with_dma    = false,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &ir_tx_channel));

    // 38 kHz carrier, 33% duty cycle (standard for IR)
    rmt_carrier_config_t carrier = {
        .frequency_hz = IR_CARRIER_HZ,
        .duty_cycle   = 0.33f,
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(ir_tx_channel, &carrier));

    rmt_copy_encoder_config_t enc_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &ir_copy_encoder));

    ESP_ERROR_CHECK(rmt_enable(ir_tx_channel));
    ESP_LOGI(TAG, "IR TX armed on GPIO%d @ %d Hz carrier", IR_TX_PIN, IR_CARRIER_HZ);
}

static void ir_send(const rmt_symbol_word_t *symbols, size_t symbol_count)
{
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,  // send once
    };
    ESP_ERROR_CHECK(rmt_transmit(
        ir_tx_channel,
        ir_copy_encoder,
        symbols,
        symbol_count * sizeof(rmt_symbol_word_t),
        &tx_config
    ));
    rmt_tx_wait_all_done(ir_tx_channel, pdMS_TO_TICKS(1000));
}

// ─────────────────────────────────────────────────
// IR receivers + directional detection
// ─────────────────────────────────────────────────
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

// Debounce one beam. Returns true on a LOW→HIGH (beam just broke) transition.
static bool debounce_beam(int gpio, int *hi, int *lo, bool *broken)
{
    int level = gpio_get_level(gpio);
    if (level) { (*hi)++; *lo = 0; }
    else        { (*lo)++; *hi = 0; }

    bool prev = *broken;
    if (!*broken && *hi >= DEBOUNCE_SAMPLES) *broken = true;
    if ( *broken && *lo >= DEBOUNCE_SAMPLES) *broken = false;
    return (!prev && *broken);   // just broke
}

static void detection_poll(void)
{
    TickType_t now = xTaskGetTickCount();

    // Ignore everything during cooldown
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
                ESP_LOGI(TAG, "Sequence timeout — reset");
                detect_state = DETECT_IDLE;
            } else if (inner_just_broke) {
                detect_state = DETECT_AWAIT_PIR;
                state_entered_at = now;
                ESP_LOGI(TAG, "Both beams broke outer→inner — awaiting PIR");
            }
            break;

        case DETECT_INNER_FIRST:
            if ((now - state_entered_at) >= pdMS_TO_TICKS(SEQUENCE_TIMEOUT_MS)) {
                ESP_LOGI(TAG, "Sequence timeout — reset");
                detect_state = DETECT_IDLE;
            } else if (outer_just_broke) {
                // EXIT: inner then outer — no PIR needed
                people_count = (people_count > 0) ? people_count - 1 : 0;
                char cnt[12];
                snprintf(cnt, sizeof(cnt), "%d", people_count);
                mqtt_pub(TOPIC_COUNT, cnt);
                if (people_count == 0 && room_occupied) {
                    room_occupied = false;
                    mqtt_pub(TOPIC_ROOM, "EMPTY");
                    // Auto-turn off AC when last person leaves
                    if (ac_is_on) {
                        ESP_LOGI(TAG, "Room empty — auto AC OFF");
                        ir_send(ac_off_cmd, sizeof(ac_off_cmd) / sizeof(ac_off_cmd[0]));
                        ac_is_on = false;
                        mqtt_pub(TOPIC_AC, "OFF");
                    }
                }
                ESP_LOGI(TAG, "<<< EXIT (people in room: %d)", people_count);
                last_detection_at = now;
                detect_state = DETECT_IDLE;
            }
            break;

        case DETECT_AWAIT_PIR:
            if ((now - state_entered_at) >= pdMS_TO_TICKS(SEQUENCE_TIMEOUT_MS)) {
                ESP_LOGI(TAG, "PIR timeout — entrance not confirmed, reset");
                detect_state = DETECT_IDLE;
            } else if (gpio_get_level(PIR_PIN) == 1) {
                // ENTRANCE: outer→inner + PIR confirmed
                people_count++;
                char cnt[12];
                snprintf(cnt, sizeof(cnt), "%d", people_count);
                mqtt_pub(TOPIC_COUNT, cnt);
                if (!room_occupied) {
                    room_occupied = true;
                    mqtt_pub(TOPIC_ROOM, "OCCUPIED");
                }
                ESP_LOGI(TAG, ">>> ENTRANCE confirmed (people in room: %d)", people_count);
                last_detection_at = now;
                detect_state = DETECT_IDLE;
            }
            break;
    }
}

// ─────────────────────────────────────────────────
// WiFi
// ─────────────────────────────────────────────────
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *event_data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        mqtt_connected = false;
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

static void wifi_scan_and_log(void)
{
    wifi_scan_config_t scan_cfg = { .show_hidden = false };
    esp_wifi_scan_start(&scan_cfg, true);

    uint16_t count = 0;
    esp_wifi_scan_get_ap_num(&count);
    if (count == 0) {
        ESP_LOGI(TAG, "WiFi scan: no networks found");
        return;
    }

    wifi_ap_record_t *records = malloc(count * sizeof(wifi_ap_record_t));
    if (!records) return;

    esp_wifi_scan_get_ap_records(&count, records);
    ESP_LOGI(TAG, "WiFi scan: %d network(s) found:", count);
    for (int i = 0; i < count; i++) {
        ESP_LOGI(TAG, "  [%2d] RSSI %4d  %s", i + 1, records[i].rssi, records[i].ssid);
    }
    free(records);
}

static void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    wifi_scan_and_log();

    esp_event_handler_instance_t h_any, h_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        wifi_event_handler, NULL, &h_any));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        wifi_event_handler, NULL, &h_ip));

    wifi_config_t wifi_cfg = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_ERROR_CHECK(esp_wifi_connect());
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                        pdFALSE, pdFALSE, portMAX_DELAY);
}

// ─────────────────────────────────────────────────
// Minimal MQTT 3.1.1 over TCP socket
// ─────────────────────────────────────────────────
static int mqtt_encode_len(uint8_t *buf, int len)
{
    int n = 0;
    do {
        uint8_t b = len & 0x7F;
        len >>= 7;
        if (len) b |= 0x80;
        buf[n++] = b;
    } while (len);
    return n;
}

static int sock_write_all(const uint8_t *data, int len)
{
    int sent = 0;
    while (sent < len) {
        int n = send(mqtt_sock, data + sent, len - sent, 0);
        if (n <= 0) return -1;
        sent += n;
    }
    return sent;
}

static int sock_read_all(uint8_t *buf, int len)
{
    int got = 0;
    while (got < len) {
        int n = recv(mqtt_sock, buf + got, len - got, 0);
        if (n <= 0) return -1;
        got += n;
    }
    return got;
}

static int mqtt_recv_packet(uint8_t *buf, int buf_size, int *out_len)
{
    uint8_t hdr;
    if (sock_read_all(&hdr, 1) < 0) return -1;

    int remaining = 0, shift = 0;
    uint8_t b;
    do {
        if (sock_read_all(&b, 1) < 0) return -1;
        remaining |= (b & 0x7F) << shift;
        shift += 7;
    } while (b & 0x80);

    *out_len = remaining;
    if (remaining > 0) {
        if (remaining > buf_size) {
            uint8_t trash[64];
            int left = remaining;
            while (left > 0) {
                int chunk = left < 64 ? left : 64;
                if (sock_read_all(trash, chunk) < 0) return -1;
                left -= chunk;
            }
        } else {
            if (sock_read_all(buf, remaining) < 0) return -1;
        }
    }
    return hdr & 0xF0;
}

static bool mqtt_send_connect_pkt(void)
{
    uint8_t pl[200];
    int plen = 0;

    pl[plen++] = 0x00; pl[plen++] = 0x04;
    pl[plen++] = 'M'; pl[plen++] = 'Q'; pl[plen++] = 'T'; pl[plen++] = 'T';
    pl[plen++] = 0x04;
    pl[plen++] = 0xC2;
    pl[plen++] = (MQTT_KEEPALIVE >> 8) & 0xFF;
    pl[plen++] = MQTT_KEEPALIVE & 0xFF;

    uint16_t id_len = strlen(MQTT_CLIENT_ID);
    pl[plen++] = id_len >> 8; pl[plen++] = id_len & 0xFF;
    memcpy(pl + plen, MQTT_CLIENT_ID, id_len); plen += id_len;

    uint16_t u_len = strlen(MQTT_USER);
    pl[plen++] = u_len >> 8; pl[plen++] = u_len & 0xFF;
    memcpy(pl + plen, MQTT_USER, u_len); plen += u_len;

    uint16_t pw_len = strlen(MQTT_PASS);
    pl[plen++] = pw_len >> 8; pl[plen++] = pw_len & 0xFF;
    memcpy(pl + plen, MQTT_PASS, pw_len); plen += pw_len;

    uint8_t pkt[210];
    int pos = 0;
    pkt[pos++] = 0x10;
    pos += mqtt_encode_len(pkt + pos, plen);
    memcpy(pkt + pos, pl, plen); pos += plen;
    return sock_write_all(pkt, pos) == pos;
}

static bool mqtt_send_subscribe_pkt(const char *topic)
{
    int tlen = strlen(topic);
    uint8_t pkt[128];
    int pos = 0;
    pkt[pos++] = 0x82;
    pos += mqtt_encode_len(pkt + pos, 2 + 2 + tlen + 1);
    pkt[pos++] = 0x00; pkt[pos++] = 0x01;
    pkt[pos++] = tlen >> 8; pkt[pos++] = tlen & 0xFF;
    memcpy(pkt + pos, topic, tlen); pos += tlen;
    pkt[pos++] = 0x00;
    return sock_write_all(pkt, pos) == pos;
}

static void mqtt_send_pingreq(void)
{
    uint8_t pkt[] = {0xC0, 0x00};
    sock_write_all(pkt, 2);
}

static void mqtt_pub(const char *topic, const char *data)
{
    if (!mqtt_connected) return;
    int tlen = strlen(topic);
    int dlen = strlen(data);
    uint8_t pkt[256];
    int pos = 0;
    pkt[pos++] = 0x30;
    pos += mqtt_encode_len(pkt + pos, 2 + tlen + dlen);
    pkt[pos++] = tlen >> 8; pkt[pos++] = tlen & 0xFF;
    memcpy(pkt + pos, topic, tlen); pos += tlen;
    memcpy(pkt + pos, data,  dlen); pos += dlen;

    xSemaphoreTake(mqtt_tx_mutex, portMAX_DELAY);
    sock_write_all(pkt, pos);
    xSemaphoreGive(mqtt_tx_mutex);
}

// ─────────────────────────────────────────────────
// Command handler
// Supported MQTT commands on TOPIC_COMMAND:
//   LIGHTS_ON  — placeholder for light control
//   LIGHTS_OFF — placeholder for light control
//   STATUS     — republish room occupancy
//   AC_ON      — send IR power-on sequence to AC
//   AC_OFF     — send IR power-off sequence to AC
//   AC_STATUS  — republish current AC state
// ─────────────────────────────────────────────────
static void on_command(const char *cmd)
{
    ESP_LOGI(TAG, "Command received: %s", cmd);

    if (strcmp(cmd, "LIGHTS_ON") == 0) {
        ESP_LOGI(TAG, ">>> Lights ON");

    } else if (strcmp(cmd, "LIGHTS_OFF") == 0) {
        ESP_LOGI(TAG, ">>> Lights OFF");

    } else if (strcmp(cmd, "STATUS") == 0) {
        mqtt_pub(TOPIC_ROOM, room_occupied ? "OCCUPIED" : "EMPTY");

    } else if (strcmp(cmd, "AC_ON") == 0) {
        ESP_LOGI(TAG, ">>> AC ON — sending IR");
        ir_send(ac_on_cmd, sizeof(ac_on_cmd) / sizeof(ac_on_cmd[0]));
        ac_is_on = true;
        mqtt_pub(TOPIC_AC, "ON");

    } else if (strcmp(cmd, "AC_OFF") == 0) {
        ESP_LOGI(TAG, ">>> AC OFF — sending IR");
        ir_send(ac_off_cmd, sizeof(ac_off_cmd) / sizeof(ac_off_cmd[0]));
        ac_is_on = false;
        mqtt_pub(TOPIC_AC, "OFF");

    } else if (strcmp(cmd, "AC_STATUS") == 0) {
        mqtt_pub(TOPIC_AC, ac_is_on ? "ON" : "OFF");
    }
}

static void mqtt_recv_task(void *arg)
{
    uint8_t buf[256];
    int pkt_len;

    while (1) {
        int type = mqtt_recv_packet(buf, sizeof(buf), &pkt_len);
        if (type < 0) {
            ESP_LOGW(TAG, "MQTT recv error — will reconnect");
            break;
        }
        if (type == 0x30 && pkt_len >= 2) {
            int tlen = (buf[0] << 8) | buf[1];
            int dlen = pkt_len - 2 - tlen;
            if (dlen > 0 && (2 + tlen + dlen) <= pkt_len) {
                char cmd[128] = {0};
                int copy = dlen < (int)(sizeof(cmd) - 1) ? dlen : (int)(sizeof(cmd) - 1);
                memcpy(cmd, buf + 2 + tlen, copy);
                on_command(cmd);
            }
        }
    }

    mqtt_connected = false;
    close(mqtt_sock);
    mqtt_sock = -1;
    vTaskDelete(NULL);
}

static bool mqtt_connect(void)
{
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(MQTT_PORT),
    };
    inet_aton(MQTT_HOST, &addr.sin_addr);

    mqtt_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (mqtt_sock < 0) { ESP_LOGE(TAG, "socket() failed"); return false; }

    if (connect(mqtt_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "TCP connect failed");
        close(mqtt_sock); mqtt_sock = -1; return false;
    }

    if (!mqtt_send_connect_pkt()) {
        close(mqtt_sock); mqtt_sock = -1; return false;
    }

    uint8_t ack[4] = {0};
    if (sock_read_all(ack, 4) < 0 || ack[0] != 0x20 || ack[3] != 0x00) {
        ESP_LOGE(TAG, "CONNACK failed (rc=0x%02x)", ack[3]);
        close(mqtt_sock); mqtt_sock = -1; return false;
    }

    mqtt_connected = true;
    ESP_LOGI(TAG, "MQTT connected.");

    xSemaphoreTake(mqtt_tx_mutex, portMAX_DELAY);
    mqtt_send_subscribe_pkt(TOPIC_COMMAND);   // room commands
    mqtt_send_subscribe_pkt(TOPIC_AC);        // direct AC commands
    xSemaphoreGive(mqtt_tx_mutex);
    ESP_LOGI(TAG, "Subscribed to: %s, %s", TOPIC_COMMAND, TOPIC_AC);

    xTaskCreate(mqtt_recv_task, "mqtt_recv", 4096, NULL, 5, NULL);
    return true;
}

// ─────────────────────────────────────────────────
// ONE-SHOT IR capture task
//
// HOW TO USE:
//   1. Temporarily call:
//        xTaskCreate(ir_capture_task, "ir_cap", 4096, NULL, 5, NULL);
//      at the end of app_main() (after ir_tx_init()).
//   2. Flash and open the serial monitor.
//   3. Point your AC remote at IR_OUTER_PIN and press the ON button.
//   4. Copy the printed symbol array from the log.
//   5. Paste it into ac_on_cmd[] above.
//   6. Repeat for the OFF button → ac_off_cmd[].
//   7. Remove this task call before your final build.
// ─────────────────────────────────────────────────
static void ir_capture_task(void *arg)
{
    ESP_LOGI(TAG, "[CAPTURE] Point your remote at GPIO%d and press a button...", IR_OUTER_PIN);

    rmt_channel_handle_t rx_ch = NULL;
    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num          = IR_OUTER_PIN,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = IR_RESOLUTION_HZ,
        .mem_block_symbols = 256,
    };
    if (rmt_new_rx_channel(&rx_cfg, &rx_ch) != ESP_OK) {
        ESP_LOGE(TAG, "[CAPTURE] Failed to create RX channel");
        vTaskDelete(NULL);
        return;
    }
    rmt_enable(rx_ch);

    rmt_symbol_word_t raw_symbols[256];
    rmt_receive_config_t recv_cfg = {
        .signal_range_min_ns =  1250,      // ignore glitches < 1.25 µs
        .signal_range_max_ns = 12000000,   // stop after 12 ms silence
    };

    // Non-blocking receive — poll until data arrives (max 10 s)
    rmt_receive(rx_ch, raw_symbols, sizeof(raw_symbols), &recv_cfg);

    size_t received_symbols = 0;
    for (int wait = 0; wait < 1000; wait++) {   // 10 s total
        vTaskDelay(pdMS_TO_TICKS(10));
        // Simple heuristic: a real frame ends with a symbol whose duration1 == 0
        // Check if any symbols appeared (duration0 > 0 on first entry)
        if (raw_symbols[0].duration0 > 0) {
            // Count non-zero symbols
            for (received_symbols = 0; received_symbols < 256; received_symbols++) {
                if (raw_symbols[received_symbols].duration0 == 0 &&
                    raw_symbols[received_symbols].duration1 == 0) break;
            }
            break;
        }
    }

    if (received_symbols == 0) {
        ESP_LOGW(TAG, "[CAPTURE] No signal received — check wiring / remote battery");
    } else {
        ESP_LOGI(TAG, "[CAPTURE] Got %d symbols. Copy into ac_on_cmd[] or ac_off_cmd[]:", (int)received_symbols);
        printf("static const rmt_symbol_word_t ac_REPLACE_cmd[] = {\n");
        for (size_t i = 0; i < received_symbols; i++) {
            printf("    {.level0=1,.duration0=%-5u .level1=0,.duration1=%-5u},\n",
                   raw_symbols[i].duration0, raw_symbols[i].duration1);
        }
        printf("};\n");
    }

    rmt_disable(rx_ch);
    rmt_del_channel(rx_ch);
    vTaskDelete(NULL);
}

// ─────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    mqtt_tx_mutex = xSemaphoreCreateMutex();

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

    // IR receivers (door beams)
    ir_init();
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "IR beams armed — outer GPIO%d, inner GPIO%d",
             IR_OUTER_PIN, IR_INNER_PIN);

    // IR transmitter (AC control)
    ir_tx_init();

    ESP_LOGI(TAG, "=== Smart Home System ===");
    wifi_init();
    mqtt_connect();

    // ── Uncomment the line below ONLY to capture your AC remote codes ──────
    // xTaskCreate(ir_capture_task, "ir_cap", 4096, NULL, 5, NULL);
    // ────────────────────────────────────────────────────────────────────────

    TickType_t last_dht_tick  = xTaskGetTickCount();
    TickType_t last_ping_tick = xTaskGetTickCount();
    TickType_t last_reconnect = xTaskGetTickCount() - pdMS_TO_TICKS(6000);

    ESP_LOGI(TAG, "System ready. Send AC_ON or AC_OFF to %s", TOPIC_COMMAND);

    while (1) {
        TickType_t now = xTaskGetTickCount();

        // ── Reconnect if dropped ─────────────────────────────────────────────
        if (!mqtt_connected && (now - last_reconnect) >= pdMS_TO_TICKS(5000)) {
            last_reconnect = now;
            ESP_LOGI(TAG, "Reconnecting MQTT...");
            mqtt_connect();
        }

        // ── Keepalive ping ───────────────────────────────────────────────────
        if (mqtt_connected && (now - last_ping_tick) >= pdMS_TO_TICKS(30000)) {
            last_ping_tick = now;
            xSemaphoreTake(mqtt_tx_mutex, portMAX_DELAY);
            mqtt_send_pingreq();
            xSemaphoreGive(mqtt_tx_mutex);
        }

        // ── Directional detection (IR beams + PIR) ──────────────────────────
        detection_poll();

        // ── DHT22 every 5 s ──────────────────────────────────────────────────
        if ((now - last_dht_tick) >= pdMS_TO_TICKS(5000)) {
            last_dht_tick = now;
            dht_data_t dht = dht_read(DHT_PIN);
            if (dht.valid) {
                char temp_str[10], hum_str[10];
                snprintf(temp_str, sizeof(temp_str), "%.1f", dht.temperature);
                snprintf(hum_str,  sizeof(hum_str),  "%.1f", dht.humidity);
                mqtt_pub(TOPIC_TEMP, temp_str);
                mqtt_pub(TOPIC_HUM,  hum_str);
                ESP_LOGI(TAG, "Temp: %s C | Hum: %s %% | Room: %s | AC: %s",
                         temp_str, hum_str,
                         room_occupied ? "OCCUPIED" : "EMPTY",
                         ac_is_on      ? "ON"       : "OFF");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}
