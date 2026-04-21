#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/param.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "esp_rom_sys.h"

static const char *TAG = "POVMotor";

// ── MAC address of the POVDisplay board ───────────────────────────────────────
static uint8_t receiverMac[ESP_NOW_ETH_ALEN] = {0x9C, 0x13, 0x9E, 0xF4, 0x22, 0x45};

// ── ESC PWM output ────────────────────────────────────────────────────────────
#define ESC_PWM_PIN       2

// ── ESC PWM parameters ───────────────────────────────────────────────────────
// 50Hz, 1.0-2.0ms pulse width (standard ESC servo signal)
#define ESC_PWM_FREQ_HZ     50
#define ESC_PWM_RESOLUTION   LEDC_TIMER_14_BIT
#define ESC_MIN_PULSE_US   1000
#define ESC_MAX_PULSE_US   2000
#define INITIAL_SPIN_COUNT 3
#define MOTOR_UPDATE_MS      50
#define ENABLE_MOTOR_INIT_SEQUENCE 0

#define RPM_CTRL_KP          0.80f
#define RPM_CTRL_KI          0.18f
#define RPM_CTRL_KD          0.01f

// P .8 / I .18 / D .001


#define RPM_SLEW_UP_PER_SEC      240U
#define RPM_SLEW_DOWN_PER_SEC    360U
#define DUTY_SLEW_PER_SEC        1800U
#define DUTY_START_THRESHOLD      1020U

// ── 3-phase encoder inputs ───────────────────────────────────────────────────
#define ENCODER_A_PIN        6
#define ENCODER_B_PIN        4
#define ENCODER_C_PIN        5
#define ZERO_PULSE_PIN        12
#define ZERO_PULSE_WIDTH_US   300
#define ENCODER_PHASE_COUNT  3U 
#define ENCODER_COUNT_BOTH_EDGES 1
#define ENCODER_COUNTS_PER_MOTOR_ROTATION 42U
#define MOTOR_ROTATIONS_PER_BLADE_ROTATION 7U
// User calibration: 42 counts per motor rotation, 7 motor rotations per blade
// rotation, spread across 3 phases => 42*7 total pulses per blade rotation.
#define ENCODER_PULSES_PER_BLADE_ROTATION \
    (ENCODER_COUNTS_PER_MOTOR_ROTATION * MOTOR_ROTATIONS_PER_BLADE_ROTATION)

#define ESC_TIMER_TOP      ((1U << ESC_PWM_RESOLUTION) - 1U)
#define ESC_PERIOD_US      (1000000U / ESC_PWM_FREQ_HZ)

static const uint32_t ESC_DUTY_MIN = (ESC_TIMER_TOP * ESC_MIN_PULSE_US) / ESC_PERIOD_US;
static const uint32_t ESC_DUTY_MAX = (ESC_TIMER_TOP * ESC_MAX_PULSE_US) / ESC_PERIOD_US;

static volatile uint16_t gTargetRpm;
static volatile uint16_t gCurrentRpm;
static volatile bool gMotorInitDone;
static volatile uint32_t gEncoderPulseCount;
static volatile int64_t gEncoderPulseTotalSigned;
static volatile int64_t gTotalDegrees;
static volatile uint8_t gEncoderPrevState;
static volatile bool gEncoderPrevStateValid;
static portMUX_TYPE gEncoderMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t gZeroPulseTaskHandle = NULL;

static void IRAM_ATTR encoderPulseIsr(void *arg)
{
    (void)arg;
    bool rotationBoundaryHit = false;

    uint8_t state = ((uint8_t)gpio_get_level(ENCODER_A_PIN) << 2) |
                    ((uint8_t)gpio_get_level(ENCODER_B_PIN) << 1) |
                    (uint8_t)gpio_get_level(ENCODER_C_PIN);

    portENTER_CRITICAL_ISR(&gEncoderMux);
    gEncoderPulseCount++;

    static const uint8_t hallSeq[6] = {0x1, 0x5, 0x4, 0x6, 0x2, 0x3};
    int8_t prevIdx = -1;
    int8_t currIdx = -1;
    for (int i = 0; i < 6; i++) {
        if (hallSeq[i] == gEncoderPrevState) {
            prevIdx = (int8_t)i;
        }
        if (hallSeq[i] == state) {
            currIdx = (int8_t)i;
        }
    }

    int8_t directionStep = 0;
    if (gEncoderPrevStateValid && prevIdx >= 0 && currIdx >= 0 && state != gEncoderPrevState) {
        if (currIdx == (int8_t)((prevIdx + 1) % 6)) {
            directionStep = 1;
        } else if (currIdx == (int8_t)((prevIdx + 5) % 6)) {
            directionStep = -1;
        }
    }

    if (directionStep != 0) {
        gEncoderPulseTotalSigned += directionStep;
        if ((gEncoderPulseTotalSigned % (int64_t)ENCODER_PULSES_PER_BLADE_ROTATION) == 0) {
            rotationBoundaryHit = true;
        }
    }

    gEncoderPrevState = state;
    gEncoderPrevStateValid = true;
    portEXIT_CRITICAL_ISR(&gEncoderMux);

    if (rotationBoundaryHit && gZeroPulseTaskHandle != NULL) {
        BaseType_t highTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(gZeroPulseTaskHandle, &highTaskWoken);
        if (highTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

static uint32_t encoderConsumePulses(void)
{
    uint32_t pulses;
    portENTER_CRITICAL(&gEncoderMux);
    pulses = gEncoderPulseCount;
    gEncoderPulseCount = 0;
    portEXIT_CRITICAL(&gEncoderMux);
    return pulses;
}

static int64_t encoderGetTotalPulsesSigned(void)
{
    int64_t totalPulses;
    portENTER_CRITICAL(&gEncoderMux);
    totalPulses = gEncoderPulseTotalSigned;
    portEXIT_CRITICAL(&gEncoderMux);
    return totalPulses;
}

static uint16_t pulsesToBladeRpm(uint32_t pulses, uint32_t windowMs)
{
    if (windowMs == 0 || ENCODER_PULSES_PER_BLADE_ROTATION == 0) {
        return 0;
    }

    uint64_t rpm = ((uint64_t)pulses * 60000ULL) /
                   ((uint64_t)ENCODER_PULSES_PER_BLADE_ROTATION * windowMs);
    if (rpm > 65535ULL) {
        rpm = 65535ULL;
    }
    return (uint16_t)rpm;
}
static uint32_t rpmToEscDuty(uint16_t rpm)
{
    if (rpm == 0) return ESC_DUTY_MIN;
    uint32_t duty = 1003U + ((uint32_t)rpm * 106U) / 1000U;
    if (duty > ESC_DUTY_MAX) duty = ESC_DUTY_MAX;
    return duty;
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

static inline void zeroPulseFire(void)
{
    gpio_set_level(ZERO_PULSE_PIN, 1);
    esp_rom_delay_us(ZERO_PULSE_WIDTH_US);
    gpio_set_level(ZERO_PULSE_PIN, 0);
}

static void zeroPulseTask(void *arg)
{
    (void)arg;
    while (1) {
        uint32_t pendingPulses = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        for (uint32_t i = 0; i < pendingPulses; i++) {
            zeroPulseFire();
        }
    }
}

// ── ESP-NOW packet v2 (shared with display) ───────────────────────────────────
#define POV_MSG_CONTROL  0x01
#define POV_MSG_STATUS   0x02
#define POV_ARROW_STEADY 0x00
#define POV_ARROW_UP     0x01
#define POV_ARROW_DOWN   0x02

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
    (void)txInfo;
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "Send FAIL (status=%d)", (int)status);
    }
}

static void espnowRecvCb(const esp_now_recv_info_t *info,
                            const uint8_t *data, int len)
{
    (void)info;
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

static void espnowSendStatus(uint16_t targetRpm, uint16_t actualRpm, bool initDone, uint8_t arrowDir)
{
    povPacketV2T pkt = {
        .msgType = POV_MSG_STATUS,
        .stripOn = 1,
        .mode = 0,
        .brightness = 0,
        .targetRpm = targetRpm,
        .actualRpm = actualRpm,
        .motorStatus = initDone ? 1 : 0,
        .reserved = arrowDir,
    };

    esp_err_t err = esp_now_send(receiverMac, (const uint8_t *)&pkt, sizeof(pkt));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Status send failed: %s", esp_err_to_name(err));
    }
}

// ── Motor output task — updates ESC PWM from target RPM ───────────────────────
static void motorTask(void *arg)
{
    uint32_t lastDuty = 0xFFFFFFFFU;
    uint16_t filteredRpm = 0;
    float integralTerm = 0.0f;
    float prevError = 0.0f;
    uint16_t commandedRpm = 0;
    const float dtSec = (float)MOTOR_UPDATE_MS / 1000.0f;
    const float dutyMin = (float)ESC_DUTY_MIN;
    const float dutyMax = (float)ESC_DUTY_MAX;
    const float dutySpan = dutyMax - dutyMin;
    uint32_t rpmStepUp = (RPM_SLEW_UP_PER_SEC * MOTOR_UPDATE_MS) / 1000U;
    uint32_t rpmStepDown = (RPM_SLEW_DOWN_PER_SEC * MOTOR_UPDATE_MS) / 1000U;
    uint32_t dutyStepMax = (DUTY_SLEW_PER_SEC * MOTOR_UPDATE_MS) / 1000U;
    int logDivider = 0;
    TickType_t lastStatusTick = 0;
    uint16_t lastSentTargetRpm = 0xFFFF;
    uint16_t lastSentActualRpm = 0xFFFF;
    bool lastSentInitDone = false;
    uint16_t prevFilteredRpm = 0;

    if (rpmStepUp == 0) rpmStepUp = 1;
    if (rpmStepDown == 0) rpmStepDown = 1;
    if (dutyStepMax == 0) dutyStepMax = 1;

    // Hold minimum throttle so ESC can arm.
    gMotorInitDone = false;
    gTargetRpm = 0;
    gCurrentRpm = 0;
    encoderConsumePulses();
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, ESC_DUTY_MIN);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    if (ENABLE_MOTOR_INIT_SEQUENCE) {
        ESP_LOGI(TAG, "ESC idle for arming: 7000ms");
        vTaskDelay(pdMS_TO_TICKS(7000));
        ESP_LOGW(TAG, "ESC initialSpin enabled: remove prop before use");
        escInitialSpin();
        ESP_LOGI(TAG, "INIT complete; entering normal RPM control");
    } else {
        ESP_LOGW(TAG, "Motor init sequence disabled; entering normal RPM control immediately");
    }
    gMotorInitDone = true;
    espnowSendStatus(gTargetRpm, gCurrentRpm, gMotorInitDone, POV_ARROW_STEADY);
    lastStatusTick = xTaskGetTickCount();
    lastSentTargetRpm = gTargetRpm;
    lastSentActualRpm = gCurrentRpm;
    lastSentInitDone = gMotorInitDone;

    while (1) {
        uint16_t requestedRpm = gTargetRpm;
        uint32_t pulses = encoderConsumePulses();
        int64_t totalPulsesSigned = encoderGetTotalPulsesSigned();
        gTotalDegrees = (totalPulsesSigned * 360LL) / ENCODER_PULSES_PER_BLADE_ROTATION;
        uint16_t measuredRpm = pulsesToBladeRpm(pulses, MOTOR_UPDATE_MS);

        if (commandedRpm < requestedRpm) {
            uint32_t next = commandedRpm + rpmStepUp;
            commandedRpm = (next > requestedRpm) ? requestedRpm : (uint16_t)next;
        } else if (commandedRpm > requestedRpm) {
            uint32_t next = (commandedRpm > rpmStepDown) ? (commandedRpm - rpmStepDown) : 0U;
            commandedRpm = (next < requestedRpm) ? requestedRpm : (uint16_t)next;
        }

        // 1st-order LPF to smooth pulse quantization at low speed.
        filteredRpm = (uint16_t)(((uint32_t)filteredRpm * 3U + measuredRpm) / 4U);
        gCurrentRpm = filteredRpm;

        uint8_t arrowDir = POV_ARROW_STEADY;
        if (filteredRpm > prevFilteredRpm) {
            arrowDir = POV_ARROW_UP;
        } else if (filteredRpm < prevFilteredRpm) {
            arrowDir = POV_ARROW_DOWN;
        }
        prevFilteredRpm = filteredRpm;

        uint32_t dutyCmd;
        if (commandedRpm == 0) {
            integralTerm = 0.0f;
            prevError = 0.0f;
            dutyCmd = ESC_DUTY_MIN;
        } else {
            float error = (float)commandedRpm - (float)filteredRpm;
            float dError = (error - prevError) / dtSec;
            prevError = error;

            float ffDuty = (float)rpmToEscDuty(commandedRpm);
            if (ffDuty < (float)DUTY_START_THRESHOLD) {
                ffDuty = (float)DUTY_START_THRESHOLD;
            }

            if ((error > 0.0f && integralTerm < 0.0f) ||
                (error < 0.0f && integralTerm > 0.0f)) {
                integralTerm *= 0.70f;
            }

            integralTerm += error * dtSec;
            if (RPM_CTRL_KI > 0.0f) {
                float iLimit = dutySpan / RPM_CTRL_KI;
                if (integralTerm > iLimit) {
                    integralTerm = iLimit;
                } else if (integralTerm < -iLimit) {
                    integralTerm = -iLimit;
                }
            }

            float dutyF = ffDuty + (RPM_CTRL_KP * error) + (RPM_CTRL_KI * integralTerm) + (RPM_CTRL_KD * dError);
            if (dutyF < dutyMin) {
                dutyF = dutyMin;
                if (error < 0.0f) {
                    integralTerm -= error * dtSec;
                }
            } else if (dutyF > dutyMax) {
                dutyF = dutyMax;
                if (error > 0.0f) {
                    integralTerm -= error * dtSec;
                }
            }

            dutyCmd = (uint32_t)dutyF;        
        }

        uint32_t duty = dutyCmd;
        if (lastDuty != 0xFFFFFFFFU) {
            bool startupKick = (lastDuty <= ESC_DUTY_MIN && dutyCmd >= DUTY_START_THRESHOLD);
            if (!startupKick) {
                if (duty > lastDuty) {
                    uint32_t up = duty - lastDuty;
                    if (up > dutyStepMax) {
                        duty = lastDuty + dutyStepMax;
                    }
                } else {
                    uint32_t down = lastDuty - duty;
                    if (down > dutyStepMax) {
                        duty = lastDuty - dutyStepMax;
                    }
                }
            }
        }

        if (duty != lastDuty) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            lastDuty = duty;
        }

        logDivider++;
        if (logDivider >= 10) {
            logDivider = 0;
            ESP_LOGI(TAG,
                     "CL RPM: req=%u cmd=%u measured=%u duty=%u pulses=%u totalDeg=%" PRId64,
                     requestedRpm, commandedRpm, filteredRpm,
                     (unsigned)lastDuty, (unsigned)pulses, gTotalDegrees);
        }

        TickType_t nowTick = xTaskGetTickCount();
        bool statusIntervalElapsed = (nowTick - lastStatusTick) >= pdMS_TO_TICKS(100);
        bool statusChanged = (requestedRpm != lastSentTargetRpm) ||
                             (filteredRpm != lastSentActualRpm) ||
                             (gMotorInitDone != lastSentInitDone);
        if (statusIntervalElapsed || statusChanged) {
            espnowSendStatus(requestedRpm, filteredRpm, gMotorInitDone, arrowDir);
            lastStatusTick = nowTick;
            lastSentTargetRpm = requestedRpm;
            lastSentActualRpm = filteredRpm;
            lastSentInitDone = gMotorInitDone;
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

static void encoderInit(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << ENCODER_A_PIN) |
                        (1ULL << ENCODER_B_PIN) |
                        (1ULL << ENCODER_C_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = ENCODER_COUNT_BOTH_EDGES ? GPIO_INTR_ANYEDGE : GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_A_PIN, encoderPulseIsr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_B_PIN, encoderPulseIsr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENCODER_C_PIN, encoderPulseIsr, NULL));

    gEncoderPrevState = ((uint8_t)gpio_get_level(ENCODER_A_PIN) << 2) |
                        ((uint8_t)gpio_get_level(ENCODER_B_PIN) << 1) |
                        (uint8_t)gpio_get_level(ENCODER_C_PIN);
    gEncoderPrevStateValid = true;
    gEncoderPulseTotalSigned = 0;
    gTotalDegrees = 0;

    ESP_LOGI(TAG,
             "Encoder ready: A=%d B=%d C=%d, edge=%s, pulses/rev=%u (42 motor counts * 7 motor/blade across %u phases)",
             ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_C_PIN,
             ENCODER_COUNT_BOTH_EDGES ? "both" : "rising",
             ENCODER_PULSES_PER_BLADE_ROTATION, ENCODER_PHASE_COUNT);
}

static void zeroPulseInit(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << ZERO_PULSE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(ZERO_PULSE_PIN, 0);
    ESP_LOGI(TAG, "Zero-angle pulse output ready on GPIO%d (%uus high)",
             ZERO_PULSE_PIN, ZERO_PULSE_WIDTH_US);
}

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    esp_err_t err;
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "Starting OTA...");

    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "Next update partition not found!");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTAPartition");
        return ESP_FAIL;
    }

    err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTABegin");
        return err;
    }

    int received = 0;
    int remaining = req->content_len;
    char buf[1024];

    while (remaining > 0) {
        if ((received = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "Failed receiving OTA data");
            esp_ota_abort(update_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive");
            return ESP_FAIL;
        }
        err = esp_ota_write(update_handle, buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
            esp_ota_abort(update_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTAWrite");
            return err;
        }
        remaining -= received;
    }

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTAEnd");
        return err;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTABoot");
        return err;
    }

    ESP_LOGI(TAG, "OTA successful, rebooting...");
    httpd_resp_sendstr(req, "OTA Complete. Rebooting...\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t ota_uri = {
            .uri       = "/update",
            .method    = HTTP_POST,
            .handler   = ota_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &ota_uri);
    }
    return server;
}

static void wifiInit(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default netifs for AP and STA so both interfaces can be started
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = "POV Controller",
            .ssid_len = strlen("POV Controller"),
            .password = "1234512345",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .channel = 1 // Ensure this matches your ESP-NOW peer's channel
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Start HTTP server for OTA
    start_webserver();
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
    encoderInit();
    zeroPulseInit();
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

    xTaskCreate(zeroPulseTask, "zero_pulse", 2048, NULL, 6, &gZeroPulseTaskHandle);
    xTaskCreate(motorTask, "motor", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Motor controller ready (closed-loop RPM, waiting for target RPM via ESP-NOW)");
}