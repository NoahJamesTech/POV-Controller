#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "oled_draw.h"

static const char *TAG = "POVMotor";

// ── MAC address of the POVDisplay board ───────────────────────────────────────
static uint8_t receiverMac[ESP_NOW_ETH_ALEN] = {0x94, 0xA9, 0x90, 0x37, 0x2F, 0x6C};

// ── ESC PWM output ────────────────────────────────────────────────────────────
// TODO: set this to the actual GPIO pin connected to the ESC signal wire
#define ESC_PWM_PIN       1

// ── OLED (SSD1306, 128x64 over I2C) ─────────────────────────────────────────
#define OLED_I2C_SCL_PIN    3
#define OLED_I2C_SDA_PIN    4
#define OLED_I2C_PORT       I2C_NUM_0
#define OLED_I2C_FREQ_HZ    400000
#define OLED_I2C_ADDR_8BIT  0x78
#define OLED_I2C_ADDR_7BIT  (OLED_I2C_ADDR_8BIT >> 1)
#define OLED_SEG_REMAP_CMD  0xA1
#define OLED_COM_SCAN_CMD   0xC0

// ── ESC PWM parameters ───────────────────────────────────────────────────────
// 50Hz, 1.0-2.0ms pulse width (standard ESC servo signal)
#define ESC_PWM_FREQ_HZ     50
#define ESC_PWM_RESOLUTION   LEDC_TIMER_14_BIT
#define ESC_MIN_PULSE_US   1000
#define ESC_MAX_PULSE_US   2000
#define INITIAL_SPIN_COUNT 3
#define ESC_RAMP_RPM_PER_SEC 300
#define MOTOR_UPDATE_MS      50

#define ESC_TIMER_TOP      ((1U << ESC_PWM_RESOLUTION) - 1U)
#define ESC_PERIOD_US      (1000000U / ESC_PWM_FREQ_HZ)

static const uint32_t ESC_DUTY_MIN = (ESC_TIMER_TOP * ESC_MIN_PULSE_US) / ESC_PERIOD_US;
static const uint32_t ESC_DUTY_MAX = (ESC_TIMER_TOP * ESC_MAX_PULSE_US) / ESC_PERIOD_US;

static volatile uint16_t gTargetRpm;
static volatile uint16_t gCurrentRpm;
static volatile bool gMotorInitDone;

// ── OLED state ───────────────────────────────────────────────────────────────
static i2c_master_bus_handle_t sOledI2cBus = NULL;
static i2c_master_dev_handle_t sOledI2cDev = NULL;
static bool sOledReady = false;

static void oledInit(void)
{
    i2c_master_bus_config_t busCfg = {
        .i2c_port = OLED_I2C_PORT,
        .sda_io_num = OLED_I2C_SDA_PIN,
        .scl_io_num = OLED_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&busCfg, &sOledI2cBus));

    i2c_device_config_t devCfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_I2C_ADDR_7BIT,
        .scl_speed_hz = OLED_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(sOledI2cBus, &devCfg, &sOledI2cDev));
    oledDrawAttachDevice(sOledI2cDev);

    const uint8_t initSeq[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, OLED_SEG_REMAP_CMD, OLED_COM_SCAN_CMD, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0x2E, 0xAF,
    };
    for (size_t i = 0; i < sizeof(initSeq); i++) {
        ESP_ERROR_CHECK(oledDrawSendCmd(initSeq[i]));
    }

    ESP_ERROR_CHECK(oledDrawClear());
    sOledReady = true;
    ESP_LOGI(TAG, "OLED SSD1306 ready on I2C SCL=%d SDA=%d addr=0x%02X(7-bit from 0x%02X) remap=0x%02X com=0x%02X",
             OLED_I2C_SCL_PIN, OLED_I2C_SDA_PIN, OLED_I2C_ADDR_7BIT,
             OLED_I2C_ADDR_8BIT,
             OLED_SEG_REMAP_CMD, OLED_COM_SCAN_CMD);
}

static void displayTask(void *arg)
{
    uint16_t lastCurrentRpm = 0xFFFF;
    uint16_t lastTargetRpm = 0xFFFF;
    int8_t lastTrend = 0;
    bool lastInitDone = true;

    while (1) {
        bool initDone = gMotorInitDone;
        uint16_t currentRpm = gCurrentRpm;
        uint16_t targetRpm = gTargetRpm;
        int8_t trend = 0;

        if (!initDone) {
            if (sOledReady && lastInitDone) {
                oledDrawLineHuge(0, "INIT");
                oledDrawLine(3, "");
                oledDrawLine(4, "");
                oledDrawLine(5, "");
                oledDrawLineBig(6, "");
            }
            lastInitDone = false;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (!lastInitDone) {
            lastCurrentRpm = 0xFFFF;
            lastTargetRpm = 0xFFFF;
            lastTrend = 0;
            lastInitDone = true;
        }

        if (lastCurrentRpm != 0xFFFF) {
            if (currentRpm > lastCurrentRpm) {
                trend = 1;
            } else if (currentRpm < lastCurrentRpm) {
                trend = -1;
            }
        }

        if (currentRpm != lastCurrentRpm || targetRpm != lastTargetRpm || trend != lastTrend) {
            char lineCurrentBig[40];
            char lineTarget[22];
            char trendChar = (trend > 0) ? 'v' : (trend < 0) ? '^' : '-';

            snprintf(lineCurrentBig, sizeof(lineCurrentBig), "%4u %c", currentRpm, trendChar);
            snprintf(lineTarget, sizeof(lineTarget), "TGT %4u", targetRpm);

            if (sOledReady) {
                oledDrawLineHuge(0, lineCurrentBig);
                oledDrawLine(3, "");
                oledDrawLine(4, "");
                oledDrawLine(5, "");
                oledDrawLineBig(6, lineTarget);
            }

            lastCurrentRpm = currentRpm;
            lastTargetRpm = targetRpm;
            lastTrend = trend;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ── ESC duty from RPM ─────────────────────────────────────────────────────────
// Maps target RPM (0-3000) linearly to ESC duty (min-max).
static uint32_t rpmToEscDuty(uint16_t rpm)
{
    if (rpm == 0) return ESC_DUTY_MIN;
    if (rpm >= 3000) return ESC_DUTY_MAX;
    return ESC_DUTY_MIN +
           (uint32_t)((uint64_t)(ESC_DUTY_MAX - ESC_DUTY_MIN) * rpm / 3000);
}

static void escInitialSpin(void)
{
    uint32_t duty = ESC_DUTY_MIN +
                    (uint32_t)(((uint64_t)(ESC_DUTY_MAX - ESC_DUTY_MIN) * 25U) / 100U);

    for (int i = 0; i < INITIAL_SPIN_COUNT; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(250));

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, ESC_DUTY_MIN);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// ── ESP-NOW packet v2 (shared with display) ───────────────────────────────────
#define POV_MSG_CONTROL  0x01
#define POV_MSG_STATUS   0x02

typedef struct __attribute__((packed)) {
    uint8_t  msgType;
    uint8_t  stripOn;
    uint8_t  mode;
    uint8_t  brightness;
    uint16_t targetRpm;
    uint16_t actualRpm;
    uint8_t  motorStatus;
    uint8_t  reserved;
} povPacketV2T;

// ── ESP-NOW callbacks ─────────────────────────────────────────────────────────
static void espnowSendCb(const esp_now_send_info_t *txInfo, esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "Send FAIL");
    }
}

static void espnowRecvCb(const esp_now_recv_info_t *info,
                            const uint8_t *data, int len)
{
    if (len != sizeof(povPacketV2T)) return;
    const povPacketV2T *pkt = (const povPacketV2T *)data;

    if (pkt->msgType == POV_MSG_CONTROL) {
        uint16_t currentRpm = gCurrentRpm;
        if (pkt->targetRpm != gTargetRpm) {
            ESP_LOGI(TAG, "RPM change cmd: current=%u -> requested=%u",
                     currentRpm, pkt->targetRpm);
        }
        gTargetRpm = pkt->targetRpm;
    }
}

// ── Motor output task — updates ESC PWM from target RPM ───────────────────────
static void motorTask(void *arg)
{
    uint16_t lastRpm = 0xFFFF;  // force initial update

    // Hold minimum throttle so ESC can arm.
    gMotorInitDone = false;
    gTargetRpm = 0;
    gCurrentRpm = 0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, ESC_DUTY_MIN);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ESP_LOGI(TAG, "ESC idle for arming: 7000ms");
    vTaskDelay(pdMS_TO_TICKS(7000));
    ESP_LOGW(TAG, "ESC initialSpin enabled: remove prop before use");
    escInitialSpin();
    gMotorInitDone = true;
    ESP_LOGI(TAG, "INIT complete; entering normal RPM control");

    while (1) {
        uint16_t targetRpm = gTargetRpm;
        uint16_t rpm = gCurrentRpm;

        uint32_t rampStep = (ESC_RAMP_RPM_PER_SEC * MOTOR_UPDATE_MS) / 1000U;
        if (rampStep == 0) {
            rampStep = 1;
        }

        if (rpm < targetRpm) {
            uint32_t next = rpm + rampStep;
            rpm = (next > targetRpm) ? targetRpm : (uint16_t)next;
        } else if (rpm > targetRpm) {
            uint32_t next = (rpm > rampStep) ? (rpm - rampStep) : 0;
            rpm = (next < targetRpm) ? targetRpm : (uint16_t)next;
        }

        if (rpm != lastRpm) {
            uint32_t duty = rpmToEscDuty(rpm);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            lastRpm = rpm;
            gCurrentRpm = rpm;
            ESP_LOGI(TAG, "ESC duty: %u (RPM %u -> target %u)", (unsigned)duty, rpm, targetRpm);
        }

        vTaskDelay(pdMS_TO_TICKS(MOTOR_UPDATE_MS));
    }
}

// ── Peripheral init ───────────────────────────────────────────────────────────
static void escPwmInit(void)
{
    ledc_timer_config_t timerCfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = ESC_PWM_RESOLUTION,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = ESC_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timerCfg));

    ledc_channel_config_t chCfg = {
        .gpio_num   = ESC_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = ESC_DUTY_MIN,  // start at idle
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chCfg));

    ESP_LOGI(TAG, "ESC PWM: %dHz on GPIO%d, duty range %u-%u",
             ESC_PWM_FREQ_HZ, ESC_PWM_PIN, ESC_DUTY_MIN, ESC_DUTY_MAX);
}

static void wifiInit(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void espnowInit(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnowSendCb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnowRecvCb));

    esp_now_peer_info_t peer = {
        .channel = 0,
        .ifidx   = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, receiverMac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

// ── Entry point ───────────────────────────────────────────────────────────────
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    escPwmInit();
    oledInit();
    wifiInit();

    // Print this board's MAC so you can paste it into the display's controller_mac
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, ">>> Controller MAC: {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, ">>> Configured Display MAC: {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}",
             receiverMac[0], receiverMac[1], receiverMac[2],
             receiverMac[3], receiverMac[4], receiverMac[5]);
    if (memcmp(mac, receiverMac, ESP_NOW_ETH_ALEN) == 0) {
        ESP_LOGW(TAG, "Configured display MAC matches controller MAC; update receiverMac to the display board MAC");
    }

    espnowInit();

    xTaskCreate(motorTask, "motor", 2048, NULL, 5, NULL);
    xTaskCreate(displayTask, "display", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "Motor controller ready (open-loop, waiting for target RPM via ESP-NOW)");
}