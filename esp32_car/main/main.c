/*
 * GestureDrive car firmware (ESP32-WROOM, ESP-IDF)
 *
 * Bluetooth Classic SPP server named "GestureDrive-Car". Laptop 2's bridge connects to it and
 * sends one command per line: FORWARD, BACKWARD, LEFT, RIGHT or STOP.
 * The L298N is driven with LEDC PWM on ENA/ENB.
 *
 * Safety: the motors stop if no command arrives within CMD_TIMEOUT_MS, when Bluetooth
 * disconnects, or when an unknown command is received.
 */
#include <ctype.h>
#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

/* ---------------- user settings ---------------- */
#define DEVICE_NAME     "GestureDrive-Car"

/* L298N pins: change these to match your wiring */
#define PIN_ENA         GPIO_NUM_14   /* left motor PWM  (remove the ENA jumper on the L298N) */
#define PIN_IN1         GPIO_NUM_27   /* left motor dir  */
#define PIN_IN2         GPIO_NUM_26
#define PIN_IN3         GPIO_NUM_25   /* right motor dir */
#define PIN_IN4         GPIO_NUM_33
#define PIN_ENB         GPIO_NUM_32   /* right motor PWM (remove the ENB jumper on the L298N) */

/* Set to 1 if a motor spins the wrong way (or swap its two wires) */
#define INVERT_LEFT     0
#define INVERT_RIGHT    0

/* Duty cycle 0-1023 (10-bit). L298N drops ~2 V, so very low duties may not move the car. */
#define DRIVE_DUTY      700           /* forward / backward */
#define TURN_DUTY       600           /* pivot turns        */

#define CMD_TIMEOUT_MS  1000          /* stop if no command for this long (sender repeats 5x/s) */
/* ----------------------------------------------- */

static const char *TAG = "car";

typedef enum { CMD_STOP = 0, CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT } car_cmd_t;
static const char *CMD_NAMES[] = {"STOP", "FORWARD", "BACKWARD", "LEFT", "RIGHT"};

static QueueHandle_t s_cmd_queue;
static volatile bool s_bt_connected = false;

/* ---------------- motors ---------------- */
#define PWM_MODE        LEDC_LOW_SPEED_MODE
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_CH_LEFT     LEDC_CHANNEL_0
#define PWM_CH_RIGHT    LEDC_CHANNEL_1

static void motors_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_IN1) | (1ULL << PIN_IN2) | (1ULL << PIN_IN3) | (1ULL << PIN_IN4),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    ledc_timer_config_t timer = {
        .speed_mode = PWM_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = PWM_TIMER,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .speed_mode = PWM_MODE,
        .timer_sel = PWM_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0,
    };
    ch.channel = PWM_CH_LEFT;
    ch.gpio_num = PIN_ENA;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
    ch.channel = PWM_CH_RIGHT;
    ch.gpio_num = PIN_ENB;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
}

/* speed: -1023..1023 (negative = backward, 0 = stop) */
static void set_motor(ledc_channel_t ch, gpio_num_t in_a, gpio_num_t in_b, int speed, bool invert)
{
    if (invert) speed = -speed;
    if (speed > 0) {
        gpio_set_level(in_a, 1);
        gpio_set_level(in_b, 0);
    } else if (speed < 0) {
        gpio_set_level(in_a, 0);
        gpio_set_level(in_b, 1);
    } else {
        gpio_set_level(in_a, 0);  /* both low = coast */
        gpio_set_level(in_b, 0);
    }
    int duty = speed < 0 ? -speed : speed;
    if (duty > 1023) duty = 1023;
    ledc_set_duty(PWM_MODE, ch, duty);
    ledc_update_duty(PWM_MODE, ch);
}

static void drive(int left, int right)
{
    set_motor(PWM_CH_LEFT, PIN_IN1, PIN_IN2, left, INVERT_LEFT);
    set_motor(PWM_CH_RIGHT, PIN_IN3, PIN_IN4, right, INVERT_RIGHT);
}

static void apply_command(car_cmd_t cmd)
{
    switch (cmd) {
    case CMD_FORWARD:  drive(DRIVE_DUTY, DRIVE_DUTY);   break;
    case CMD_BACKWARD: drive(-DRIVE_DUTY, -DRIVE_DUTY); break;
    case CMD_LEFT:     drive(-TURN_DUTY, TURN_DUTY);    break;  /* pivot left  */
    case CMD_RIGHT:    drive(TURN_DUTY, -TURN_DUTY);    break;  /* pivot right */
    default:           drive(0, 0);                     break;
    }
}

/* ---------------- command parsing ---------------- */
static car_cmd_t parse_command(const char *s)
{
    for (int i = 0; i < 5; i++) {
        if (strcmp(s, CMD_NAMES[i]) == 0) return (car_cmd_t)i;
    }
    ESP_LOGW(TAG, "Unknown command '%s' -> STOP", s);
    return CMD_STOP;
}

/* Bytes can arrive split or several lines at once, so collect until '\n' */
static void handle_rx(const uint8_t *data, uint16_t len)
{
    static char line[32];
    static size_t n = 0;

    for (uint16_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\n' || c == '\r') {
            if (n > 0) {
                line[n] = '\0';
                for (size_t k = 0; k < n; k++) line[k] = (char)toupper((unsigned char)line[k]);
                car_cmd_t cmd = parse_command(line);
                xQueueOverwrite(s_cmd_queue, &cmd);  /* only the newest command matters */
                n = 0;
            }
        } else if (n < sizeof(line) - 1) {
            line[n++] = c;
        } else {
            n = 0;  /* line too long: drop it */
        }
    }
}

/* ---------------- control task (owns the motors + watchdog) ---------------- */
static void control_task(void *arg)
{
    car_cmd_t current = CMD_STOP;
    int64_t last_rx_us = 0;
    bool timed_out = true;

    apply_command(CMD_STOP);
    for (;;) {
        car_cmd_t cmd;
        if (xQueueReceive(s_cmd_queue, &cmd, pdMS_TO_TICKS(50)) == pdTRUE) {
            last_rx_us = esp_timer_get_time();
            timed_out = false;
            if (cmd != current) {
                current = cmd;
                apply_command(current);
                ESP_LOGI(TAG, "-> %s", CMD_NAMES[current]);
            }
        }

        bool stale = (esp_timer_get_time() - last_rx_us) > (int64_t)CMD_TIMEOUT_MS * 1000;
        if ((!s_bt_connected || stale) && !timed_out) {
            timed_out = true;
            current = CMD_STOP;
            apply_command(CMD_STOP);
            ESP_LOGW(TAG, "Watchdog: %s -> STOP", s_bt_connected ? "no command" : "BT disconnected");
        }
    }
}

/* ---------------- Bluetooth ---------------- */
static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "SPP_SERVER");
        } else {
            ESP_LOGE(TAG, "SPP init failed: %d", param->init.status);
        }
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            esp_bt_gap_set_device_name(DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            ESP_LOGI(TAG, "SPP server ready on RFCOMM channel %d as '%s'", param->start.scn, DEVICE_NAME);
        }
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        s_bt_connected = true;
        ESP_LOGI(TAG, "Bridge connected");
        break;
    case ESP_SPP_CLOSE_EVT:
        s_bt_connected = false;
        ESP_LOGW(TAG, "Bridge disconnected");
        break;
    case ESP_SPP_DATA_IND_EVT:
        handle_rx(param->data_ind.data, param->data_ind.len);
        break;
    default:
        break;
    }
}

static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Paired with %s", param->auth_cmpl.device_name);
        } else {
            ESP_LOGE(TAG, "Pairing failed: %d", param->auth_cmpl.stat);
        }
        break;
    case ESP_BT_GAP_CFM_REQ_EVT:  /* Secure Simple Pairing: accept automatically ("Just Works") */
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_PIN_REQ_EVT: {  /* legacy pairing fallback: PIN 1234 */
        esp_bt_pin_code_t pin = {'1', '2', '3', '4'};
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
        break;
    }
    default:
        break;
    }
}

static void bluetooth_init(void)
{
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));  /* Classic only: free BLE RAM */

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    esp_bluedroid_config_t bd_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    bd_cfg.ssp_en = true;
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bd_cfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_callback));
    ESP_ERROR_CHECK(esp_spp_register_callback(spp_callback));

    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;  /* no screen/keyboard -> "Just Works" pairing */
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(iocap));

    esp_bt_pin_code_t pin = {'1', '2', '3', '4'};
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin);

    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0,
    };
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp_cfg));

    const uint8_t *mac = esp_bt_dev_get_address();
    ESP_LOGI(TAG, "Bluetooth MAC: %02X:%02X:%02X:%02X:%02X:%02X  <- put this in the bridge script",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();  /* Bluetooth stores pairing keys in NVS */
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    motors_init();
    s_cmd_queue = xQueueCreate(1, sizeof(car_cmd_t));
    xTaskCreate(control_task, "control", 4096, NULL, 5, NULL);

    bluetooth_init();
}
