#include <string.h>
#include "oled_draw.h"

#define OLED_WIDTH 128
#define OLED_PAGES 8

static i2c_master_dev_handle_t sOledI2cDev = NULL;

typedef struct {
    char c;
    uint8_t cols[5];
} glyph5x7_t;

static const glyph5x7_t sGlyphs[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'0', {0x3E, 0x45, 0x49, 0x51, 0x3E}},
    {'1', {0x00, 0x21, 0x7F, 0x01, 0x00}},
    {'2', {0x21, 0x43, 0x45, 0x49, 0x31}},
    {'3', {0x42, 0x41, 0x51, 0x69, 0x46}},
    {'4', {0x0C, 0x14, 0x24, 0x7F, 0x04}},
    {'5', {0x72, 0x51, 0x51, 0x51, 0x4E}},
    {'6', {0x1E, 0x29, 0x49, 0x49, 0x06}},
    {'7', {0x40, 0x47, 0x48, 0x50, 0x60}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x30, 0x49, 0x49, 0x4A, 0x3C}},
    {'A', {0x1F, 0x24, 0x44, 0x24, 0x1F}},
    {'C', {0x1E, 0x21, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'G', {0x1E, 0x21, 0x49, 0x49, 0x2E}},
    {'I', {0x41, 0x41, 0x7F, 0x41, 0x41}},
    {'M', {0x7F, 0x10, 0x08, 0x10, 0x7F}},
    {'N', {0x7F, 0x10, 0x08, 0x04, 0x7F}},
    {'P', {0x7F, 0x48, 0x48, 0x48, 0x30}},
    {'R', {0x7F, 0x48, 0x4C, 0x4A, 0x31}},
    {'T', {0x40, 0x40, 0x7F, 0x40, 0x40}},
    {'U', {0x7E, 0x01, 0x01, 0x01, 0x7E}},
    {'V', {0x70, 0x0E, 0x01, 0x0E, 0x70}},
    {'v', {0x10, 0x20, 0x7C, 0x20, 0x10}},
    {'^', {0x10, 0x08, 0x7C, 0x08, 0x10}},
};

static const uint8_t *findGlyph(char c)
{
    for (size_t i = 0; i < sizeof(sGlyphs) / sizeof(sGlyphs[0]); i++) {
        if (sGlyphs[i].c == c) {
            return sGlyphs[i].cols;
        }
    }
    return sGlyphs[0].cols;
}

void oledDrawAttachDevice(i2c_master_dev_handle_t dev)
{
    sOledI2cDev = dev;
}

esp_err_t oledDrawSendCmd(uint8_t cmd)
{
    uint8_t payload[2] = {0x00, cmd};
    return i2c_master_transmit(sOledI2cDev, payload, sizeof(payload), -1);
}

static esp_err_t ssd1306SetCursor(uint8_t page, uint8_t col)
{
    esp_err_t err = oledDrawSendCmd((uint8_t)(0xB0 | (page & 0x07)));
    if (err != ESP_OK) return err;
    err = oledDrawSendCmd((uint8_t)(0x00 | (col & 0x0F)));
    if (err != ESP_OK) return err;
    return oledDrawSendCmd((uint8_t)(0x10 | ((col >> 4) & 0x0F)));
}

esp_err_t oledDrawLine(uint8_t page, const char *text)
{
    uint8_t data[1 + OLED_WIDTH];
    data[0] = 0x40;
    memset(&data[1], 0x00, OLED_WIDTH);

    int x = 0;
    for (size_t i = 0; text[i] != '\0' && x <= (OLED_WIDTH - 6); i++) {
        const uint8_t *glyph = findGlyph(text[i]);
        for (int col = 0; col < 5; col++) {
            data[1 + x++] = glyph[col];
        }
        data[1 + x++] = 0x00;
    }

    esp_err_t err = ssd1306SetCursor(page, 0);
    if (err != ESP_OK) return err;
    return i2c_master_transmit(sOledI2cDev, data, sizeof(data), -1);
}

static void expandCol2x(uint8_t src, uint8_t *top, uint8_t *bottom)
{
    uint16_t expanded = 0;
    for (int bit = 0; bit < 8; bit++) {
        if ((src >> bit) & 0x01) {
            int outBit = bit * 2;
            expanded |= (uint16_t)(1U << outBit);
            if (outBit + 1 < 16) {
                expanded |= (uint16_t)(1U << (outBit + 1));
            }
        }
    }
    *top = (uint8_t)(expanded & 0xFF);
    *bottom = (uint8_t)((expanded >> 8) & 0xFF);
}

static void expandCol3x(uint8_t src, uint8_t *page0, uint8_t *page1, uint8_t *page2)
{
    uint32_t expanded = 0;
    for (int bit = 0; bit < 8; bit++) {
        if ((src >> bit) & 0x01) {
            int outBit = bit * 3;
            expanded |= (uint32_t)(1UL << outBit);
            expanded |= (uint32_t)(1UL << (outBit + 1));
            expanded |= (uint32_t)(1UL << (outBit + 2));
        }
    }
    *page0 = (uint8_t)(expanded & 0xFF);
    *page1 = (uint8_t)((expanded >> 8) & 0xFF);
    *page2 = (uint8_t)((expanded >> 16) & 0xFF);
}

esp_err_t oledDrawLineBig(uint8_t pageStart, const char *text)
{
    if (pageStart >= (OLED_PAGES - 1)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t topData[1 + OLED_WIDTH];
    uint8_t bottomData[1 + OLED_WIDTH];
    topData[0] = 0x40;
    bottomData[0] = 0x40;
    memset(&topData[1], 0x00, OLED_WIDTH);
    memset(&bottomData[1], 0x00, OLED_WIDTH);

    int x = 0;
    for (size_t i = 0; text[i] != '\0' && x <= (OLED_WIDTH - 12); i++) {
        const uint8_t *glyph = findGlyph(text[i]);

        for (int col = 0; col < 5; col++) {
            uint8_t topCol = 0;
            uint8_t bottomCol = 0;
            expandCol2x(glyph[col], &topCol, &bottomCol);

            topData[1 + x] = topCol;
            bottomData[1 + x] = bottomCol;
            x++;

            topData[1 + x] = topCol;
            bottomData[1 + x] = bottomCol;
            x++;
        }

        topData[1 + x] = 0x00;
        bottomData[1 + x] = 0x00;
        x++;
        topData[1 + x] = 0x00;
        bottomData[1 + x] = 0x00;
        x++;
    }

    esp_err_t err = ssd1306SetCursor(pageStart, 0);
    if (err != ESP_OK) return err;
    err = i2c_master_transmit(sOledI2cDev, topData, sizeof(topData), -1);
    if (err != ESP_OK) return err;
    err = ssd1306SetCursor((uint8_t)(pageStart + 1), 0);
    if (err != ESP_OK) return err;
    return i2c_master_transmit(sOledI2cDev, bottomData, sizeof(bottomData), -1);
}

esp_err_t oledDrawLineHuge(uint8_t pageStart, const char *text)
{
    if (pageStart >= (OLED_PAGES - 2)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t page0Data[1 + OLED_WIDTH];
    uint8_t page1Data[1 + OLED_WIDTH];
    uint8_t page2Data[1 + OLED_WIDTH];
    page0Data[0] = 0x40;
    page1Data[0] = 0x40;
    page2Data[0] = 0x40;
    memset(&page0Data[1], 0x00, OLED_WIDTH);
    memset(&page1Data[1], 0x00, OLED_WIDTH);
    memset(&page2Data[1], 0x00, OLED_WIDTH);

    int x = 0;
    for (size_t i = 0; text[i] != '\0' && x <= (OLED_WIDTH - 18); i++) {
        const uint8_t *glyph = findGlyph(text[i]);

        for (int col = 0; col < 5; col++) {
            uint8_t p0 = 0;
            uint8_t p1 = 0;
            uint8_t p2 = 0;
            expandCol3x(glyph[col], &p0, &p1, &p2);

            page0Data[1 + x] = p0;
            page1Data[1 + x] = p1;
            page2Data[1 + x] = p2;
            x++;
            page0Data[1 + x] = p0;
            page1Data[1 + x] = p1;
            page2Data[1 + x] = p2;
            x++;
            page0Data[1 + x] = p0;
            page1Data[1 + x] = p1;
            page2Data[1 + x] = p2;
            x++;
        }

        page0Data[1 + x] = 0x00;
        page1Data[1 + x] = 0x00;
        page2Data[1 + x] = 0x00;
        x++;
        page0Data[1 + x] = 0x00;
        page1Data[1 + x] = 0x00;
        page2Data[1 + x] = 0x00;
        x++;
        page0Data[1 + x] = 0x00;
        page1Data[1 + x] = 0x00;
        page2Data[1 + x] = 0x00;
        x++;
    }

    esp_err_t err = ssd1306SetCursor(pageStart, 0);
    if (err != ESP_OK) return err;
    err = i2c_master_transmit(sOledI2cDev, page0Data, sizeof(page0Data), -1);
    if (err != ESP_OK) return err;

    err = ssd1306SetCursor((uint8_t)(pageStart + 1), 0);
    if (err != ESP_OK) return err;
    err = i2c_master_transmit(sOledI2cDev, page1Data, sizeof(page1Data), -1);
    if (err != ESP_OK) return err;

    err = ssd1306SetCursor((uint8_t)(pageStart + 2), 0);
    if (err != ESP_OK) return err;
    return i2c_master_transmit(sOledI2cDev, page2Data, sizeof(page2Data), -1);
}

esp_err_t oledDrawClear(void)
{
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        esp_err_t err = oledDrawLine(page, "");
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}
