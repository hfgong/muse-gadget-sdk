/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * LilyGO T-Display-S3 AMOLED 1.91, the non-touch QSPI variant: rectangular
 * 536x240 RM67162 AMOLED, one button (BOOT), a battery divider on GPIO4 and
 * no PMU, speaker or microphone. The RM67162 answers the same QSPI framing
 * and DCS commands as the SH8601 (0x02 register writes, 0x32 pixel writes,
 * 0x51 brightness), so the SH8601 driver carries it with the vendor init
 * sequence from LilyGO's LilyGo-AMOLED-Series; MADCTL 0x60 in that sequence
 * turns the 240x536 panel landscape.
 *
 * BOOT is the whole keyboard: a press wakes the screen and confirms pairing
 * (without a mic, replies are text), and holding it powers off - with a
 * battery on the back there is no other way to turn the board off. Deep
 * sleep is "off" (no PMU to open a BATFET); BOOT wakes it. Without touch
 * the settings menu is unreachable, as on the LCD-1.9 when its touch is
 * absent. Pins from LilyGo-AMOLED-Series' BOARD_AMOLED_191.
 */
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/usb_serial_jtag.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "muse_board.h"
#include "muse_input.h"
#include "muse_ui.h"
#include "muse_lcd_bands.h"
#include "muse_mem.h"

#include "esp_lcd_sh8601.h"

static const char *TAG = "board";

/* Pins (LilyGo-AMOLED-Series, BOARD_AMOLED_191 / RM67162_AMOLED). */
#define LCD_CS GPIO_NUM_6
#define LCD_SCLK GPIO_NUM_47
#define LCD_D0 GPIO_NUM_18
#define LCD_D1 GPIO_NUM_7
#define LCD_D2 GPIO_NUM_48
#define LCD_D3 GPIO_NUM_5
#define LCD_RST GPIO_NUM_17
#define LCD_EN GPIO_NUM_38            /* AMOLED power; high to run */
#define LCD_H_RES 536
#define LCD_V_RES 240
#define LCD_HOST SPI2_HOST

/* The divider halves the cell voltage into GPIO4 (LilyGO reads it the same
 * way and doubles it back). */
#define BAT_ADC ADC_CHANNEL_3

#define PWR_HOLD_MS 3000              /* hold BOOT this long to power off */

#define DRAW_BUF_LINES 60             /* four bands to the screen (muse_lcd_bands.h) */
#define LCD_CHUNK_BYTES (LCD_H_RES * 8 * 2)

static esp_lcd_panel_handle_t s_panel;
static esp_lcd_panel_io_handle_t s_io;
static muse_gpio_button_t s_boot;
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static uint8_t s_lcd_level = 0xA0;
static TickType_t s_boot_down_at;

/* RM67162 init, from LilyGo-AMOLED-Series initSequence.cpp (rm67162_cmd):
 * ELVSS/OVSS levels on their vendor pages, then landscape MADCTL, RGB565,
 * display on. Brightness (0x51) is sent after init like the UI will. */
static const sh8601_lcd_init_cmd_t s_init_cmds[] = {
    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0x11, NULL, 0, 120},                 /* sleep out */
    {0xFE, (uint8_t[]){0x05}, 1, 0},
    {0x05, (uint8_t[]){0x05}, 1, 0},      /* OVSS -3.95 V */
    {0xFE, (uint8_t[]){0x01}, 1, 0},
    {0x73, (uint8_t[]){0x25}, 1, 0},      /* OVSS level -4.0 V */
    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0x36, (uint8_t[]){0x60}, 1, 0},      /* landscape: 536 wide */
    {0x3A, (uint8_t[]){0x55}, 1, 0},      /* RGB565 */
    {0x29, NULL, 0, 20},                  /* display on */
};

static esp_err_t init(void)
{
    /* Panel power first, so the controller is up before display_start resets it. */
    gpio_config_t en_cfg = {
        .pin_bit_mask = 1ULL << LCD_EN,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&en_cfg), TAG, "lcd en");
    gpio_set_level(LCD_EN, 1);

    ESP_RETURN_ON_ERROR(muse_gpio_button_init(&s_boot, GPIO_NUM_0), TAG, "boot button");
    /* BOOT is held at flashing time; don't count a held button as a press. */
    s_boot.pressed = gpio_get_level(GPIO_NUM_0) == 0;

    const adc_oneshot_unit_init_cfg_t adc_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&adc_cfg, &s_adc), TAG, "adc");
    const adc_oneshot_chan_cfg_t ch_cfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, BAT_ADC, &ch_cfg), TAG, "adc channel");
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = BAT_ADC,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali) != ESP_OK) {
        ESP_LOGW(TAG, "adc calibration unavailable; battery mV is uncalibrated");
    }
#endif
    return ESP_OK;
}

/* An RM67162 command over QSPI, sent between flush bands so it never lands in
 * the middle of a pixel write. */
static void lcd_cmd(uint8_t cmd, const void *param, size_t n)
{
    esp_lcd_panel_io_tx_param(s_io, (0x02 << 24) | (cmd << 8), param, n);
}

static void send_brightness(void *level)
{
    s_lcd_level = *(uint8_t *)level;
    lcd_cmd(0x51, &s_lcd_level, 1);
}

static void send_sleep(void *sleep)
{
    lcd_cmd(*(bool *)sleep ? 0x10 : 0x11, NULL, 0);
}

static void send_off(void *arg)
{
    esp_lcd_panel_disp_on_off(s_panel, false);
}

/* muse_lcd_bands_register takes over the panel IO's color-done callback and,
 * with the bands, needs the bus set up on the same core as the UI task. */
static lv_display_t *display_start(lv_indev_t **touch)
{
    const spi_bus_config_t bus =
        SH8601_PANEL_BUS_QSPI_CONFIG(LCD_SCLK, LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_CHUNK_BYTES);
    if (spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) {
        ESP_LOGE(TAG, "spi bus");
        return NULL;
    }
    const esp_lcd_panel_io_spi_config_t io_cfg = SH8601_PANEL_IO_QSPI_CONFIG(LCD_CS, NULL, NULL);
    if (esp_lcd_new_panel_io_spi(LCD_HOST, &io_cfg, &s_io) != ESP_OK) {
        ESP_LOGE(TAG, "panel io");
        return NULL;
    }
    /* Non-const: esp_lcd_panel_dev_config_t.vendor_config is a plain void *. */
    sh8601_vendor_config_t vendor = {
        .init_cmds = s_init_cmds,
        .init_cmds_size = sizeof(s_init_cmds) / sizeof(s_init_cmds[0]),
        .flags.use_qspi_interface = 1,
    };
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor,
    };
    if (esp_lcd_new_panel_sh8601(s_io, &panel_cfg, &s_panel) != ESP_OK) {
        ESP_LOGE(TAG, "rm67162 over sh8601");
        return NULL;
    }
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    lcd_cmd(0x51, &s_lcd_level, 1);   /* visible until the UI sets its own */

    esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    adapter_cfg.task_core_id = MUSE_UI_CORE;
    adapter_cfg.task_priority = MUSE_UI_PRIORITY;
    if (esp_lv_adapter_init(&adapter_cfg) != ESP_OK) {
        return NULL;
    }
    const esp_lv_adapter_display_config_t disp_cfg = {
        .panel = s_panel,
        .panel_io = s_io,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_OTHER,
            .rotation = ESP_LV_ADAPTER_ROTATE_0,   /* MADCTL already landscape */
            .hor_res = LCD_H_RES,
            .ver_res = LCD_V_RES,
        },
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE,
    };
    lv_display_t *disp = muse_lcd_bands_register(disp_cfg, DRAW_BUF_LINES, LCD_CHUNK_BYTES);
    if (!disp) {
        return NULL;
    }
    if (esp_lv_adapter_start() != ESP_OK) {
        return NULL;
    }
    *touch = NULL;   /* no touch on this variant; Settings are unreachable */
    return disp;
}

static bool display_lock(int timeout_ms)
{
    return esp_lv_adapter_lock(timeout_ms) == ESP_OK;
}

/* AMOLED: no backlight pin, brightness is the panel's 0x51 command. */
static void set_brightness(int pct)
{
    uint8_t level = (uint8_t)(pct * 255 / 100);
    muse_lcd_bands_run(send_brightness, &level);
}

/* SLPIN/SLPOUT: GRAM is kept. */
static void panel_sleep(bool sleep)
{
    muse_lcd_bands_run(send_sleep, &sleep);
    vTaskDelay(pdMS_TO_TICKS(120));   /* settle before the next command */
}

static void display_pause(bool pause)
{
    if (pause) {
        esp_lv_adapter_pause(-1);
    } else {
        esp_lv_adapter_resume();
    }
}

/*
 * BOOT is the only button. A press wakes the screen and confirms pairing
 * (without a mic there is nothing to talk into), and holding it PWR_HOLD_MS
 * asks for power off - with a battery attached this is the only off switch.
 * The hold fires while the button is still down, so the power-off screen is
 * seen before the finger lifts, like the StickC Plus2's PWR button.
 */
static unsigned poll_buttons(void)
{
    unsigned ev = muse_gpio_button_poll(&s_boot);
    if (ev & MUSE_BTN_TALK_PRESS) {
        s_boot_down_at = xTaskGetTickCount();
    } else if (s_boot.pressed && s_boot_down_at &&
               xTaskGetTickCount() - s_boot_down_at >= pdMS_TO_TICKS(PWR_HOLD_MS)) {
        s_boot_down_at = 0;
        muse_input_request_power_off();
    }
    if (!s_boot.pressed) {
        s_boot_down_at = 0;
    }
    return ev;
}

static void wait_buttons(int timeout_ms)
{
    muse_gpio_buttons_wait((muse_gpio_button_t *const[]){ &s_boot }, 1, timeout_ms);
}

/* The divider halves the cell voltage; double the calibrated reading back.
 * No percentage: on a charger the divider reads the charge voltage, which
 * looks like a full cell, and there is no charge-state line to tell. */
static esp_err_t read_power(muse_power_t *out)
{
    int sum = 0;
    for (int i = 0; i < 8; i++) {
        int v;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc, BAT_ADC, &v), TAG, "adc read");
        sum += v;
    }
    int raw = sum / 8;
    int mv;
    if (s_cali) {
        ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(s_cali, raw, &mv), TAG, "adc cali");
    } else {
        mv = raw * 3300 / 4096;
    }
    out->battery_mv = mv * 2;
    out->battery_pct = -1;
    out->charging = false;   /* no charge-state line reaches the chip */
    out->usb = usb_serial_jtag_is_connected();
    return ESP_OK;
}

static esp_err_t power_off(void)
{
    set_brightness(0);
    muse_lcd_bands_run(send_off, NULL);
    panel_sleep(true);
    gpio_set_level(LCD_EN, 0);
    /* No PMU; deep sleep is the closest thing to off. BOOT wakes it. */
    ESP_RETURN_ON_ERROR(esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0), TAG, "boot wake");
    esp_deep_sleep_start();
    return ESP_FAIL;
}

static const muse_board_t s_board = {
    .name = "LilyGO T-Display-S3 AMOLED 1.91",
    .width = LCD_H_RES,
    .height = LCD_V_RES,
    .round = false,
    .touch = false,
    .diagonal_in = 1.91f,
    .talk_button = "boot",
    /* BOOT is beside the USB port. */
    .talk_hint = { LV_ALIGN_TOP_LEFT, 20, 16 },
    .frame_ms = 40,
    .init = init,
    .display_start = display_start,
    .display_lock = display_lock,
    .display_unlock = esp_lv_adapter_unlock,
    .set_brightness = set_brightness,
    .panel_sleep = panel_sleep,
    .display_pause = display_pause,
    /* No speaker or microphone on this board: audio_init stays NULL and the
     * replies are text on screen. */
    .poll_buttons = poll_buttons,
    .wait_buttons = wait_buttons,
    .read_power = read_power,
    .power_off = power_off,
};

/* Home Link's app_main starts Muse with this board (main/main.c). */
const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
