// Slat Watch — ESP32-2432S028R ("CYD", ILI9341 320x240), ESP-IDF 6.1
//
// Port of slat_watch.html. The 200x200 SVG is rescaled to a 240x240 square
// (x1.2) centred on the 320x240 panel. Rendering is done in horizontal strips
// (240 x STRIP_H px, RGB565) double-buffered over SPI DMA, so no full
// framebuffer is needed.
//
// Time comes from NTP over WiFi (configure with `idf.py menuconfig` ->
// "Slat Watch"). The watch fast-forwards until the first sync, then switches
// to real time. BOOT button (GPIO0) toggles fast-forward / real time.

#include <math.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9341.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif_sntp.h"
#include "nvs_flash.h"

static const char *TAG = "slat";

// ---------- user config ----------
// WiFi / NTP / TZ come from Kconfig (main/Kconfig.projbuild).
// Panel orientation: both mirrors flipped together = 180deg rotation.
// Flipping only one reverses the direction of the hands.
#define MIRROR_X    true
#define MIRROR_Y    true

// ---------- CYD pins (HSPI) ----------
#define PIN_SCLK 14
#define PIN_MOSI 13
#define PIN_CS   15
#define PIN_DC    2
#define PIN_BL   21
#define PIN_BTN   0
#define LCD_HZ   (40 * 1000 * 1000)
#define LCD_W    320
#define LCD_H    240

// ---------- geometry (original values x1.2) ----------
#define SZ      240                         // square drawing area
#define X0      ((LCD_W - SZ) / 2)          // left offset on panel
#define CTR     120.0f
static const float R = 106, RMC = 94, RH = 72, RM = 86;   // hour circle, minute circle, hand tips
static const float SIG = 3.6f, XW = 28.8f;                // gaussian width, crossfade width
static const float RA = 3.6f, RL = 14.4f, RW = 5, RD = 16.8f; // ripple amp, wavelength, speed, decay
#define NS     80                           // slats per side
#define PITCH   3
#define OFF     1
#define SLAT_W  2

#define STRIP_H 24                          // 240/24 = 10 strips per frame
#define WHITE   0xFFFF
#define BLACK   0x0000
#define GREY    0xFBDE                      // #ddd as byte-swapped RGB565

// ---------- math (same as the JS) ----------
static inline float rest(float c, float r) {
    float o = fabsf(c - CTR);
    return o < r ? CTR - sqrtf(r * r - o * o) : CTR;
}
static inline float clampf(float v, float a, float b) { return v < a ? a : v > b ? b : v; }
static inline float ripple(float d, float ph) {      // ph = t*RW, pre-wrapped
    float a = fabsf(d);
    return RA * cosf(a * 2 * (float)M_PI / RL - ph) * expf(-a / RD) * fminf(1, a / (2 * SIG));
}
static inline float bump(float b, float full, float d, float k) {
    return full <= b ? b : b + (full - b) * k * expf(-d * d / (2 * SIG * SIG));
}

// ---------- display ----------
static esp_lcd_panel_handle_t panel;
static SemaphoreHandle_t buf_free;
static uint16_t *bufs[2];
static int16_t lT[NS], lB[NS], lL[NS], lR[NS];

static bool on_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *e, void *ctx) {
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(buf_free, &hp);
    return hp == pdTRUE;
}

static void lcd_init(void) {
    gpio_set_direction(PIN_BL, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_BL, 0);

    spi_bus_config_t bus = {
        .sclk_io_num = PIN_SCLK, .mosi_io_num = PIN_MOSI, .miso_io_num = -1,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = SZ * STRIP_H * 2 + 8,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = PIN_DC, .cs_gpio_num = PIN_CS, .pclk_hz = LCD_HZ,
        .lcd_cmd_bits = 8, .lcd_param_bits = 8, .spi_mode = 0,
        .trans_queue_depth = 4, .on_color_trans_done = on_done,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io));

    esp_lcd_panel_dev_config_t pc = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io, &pc, &panel));
    esp_lcd_panel_reset(panel);
    esp_lcd_panel_init(panel);
    esp_lcd_panel_swap_xy(panel, true);
    esp_lcd_panel_mirror(panel, MIRROR_X, MIRROR_Y);
    esp_lcd_panel_disp_on_off(panel, true);

    buf_free = xSemaphoreCreateCounting(2, 2);
    for (int i = 0; i < 2; i++) {
        bufs[i] = heap_caps_malloc(SZ * STRIP_H * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        assert(bufs[i]);
    }

    // grey side margins, drawn once
    for (int y = 0; y < LCD_H; y += STRIP_H) {
        xSemaphoreTake(buf_free, portMAX_DELAY);
        xSemaphoreTake(buf_free, portMAX_DELAY);   // hold both slots
        uint16_t *b = bufs[0];
        for (int i = 0; i < X0 * STRIP_H; i++) b[i] = GREY;
        esp_lcd_panel_draw_bitmap(panel, 0, y, X0, y + STRIP_H, b);
        esp_lcd_panel_draw_bitmap(panel, LCD_W - X0, y, LCD_W, y + STRIP_H, b);
        xSemaphoreTake(buf_free, portMAX_DELAY);   // wait for both transfers
        xSemaphoreTake(buf_free, portMAX_DELAY);
        xSemaphoreGive(buf_free); xSemaphoreGive(buf_free);
    }
    gpio_set_level(PIN_BL, 1);
}

// fill rect (x,y,w,h) in 240-space, clipped to strip [y0, y0+STRIP_H)
static inline void fill(uint16_t *buf, int y0, int x, int y, int w, int h) {
    int ya = y > y0 ? y : y0, yb = y + h < y0 + STRIP_H ? y + h : y0 + STRIP_H;
    int xa = x < 0 ? 0 : x, xb = x + w > SZ ? SZ : x + w;
    if (ya >= yb || xa >= xb) return;
    for (int yy = ya; yy < yb; yy++) {
        uint16_t *row = buf + (yy - y0) * SZ;
        for (int xx = xa; xx < xb; xx++) row[xx] = BLACK;
    }
}

static void render_frame(void) {
    static int which = 0;
    for (int y0 = 0; y0 < SZ; y0 += STRIP_H) {
        xSemaphoreTake(buf_free, portMAX_DELAY);
        uint16_t *b = bufs[which]; which ^= 1;
        for (int i = 0; i < SZ * STRIP_H; i++) b[i] = WHITE;
        for (int i = 0; i < NS; i++) {
            int p = OFF + i * PITCH;
            fill(b, y0, p, 0, SLAT_W, lT[i]);
            fill(b, y0, p, SZ - lB[i], SLAT_W, lB[i]);
            fill(b, y0, 0, p, lL[i], SLAT_W);
            fill(b, y0, SZ - lR[i], p, lR[i], SLAT_W);
        }
        esp_lcd_panel_draw_bitmap(panel, X0, y0, X0 + SZ, y0 + STRIP_H, b);
    }
}

static void compute(float sim, float ph) {
    float m = fmodf(sim, 60);
    float ha = sim * 0.5f * (float)M_PI / 180, ma = m * 6 * (float)M_PI / 180;
    float hx = CTR + RH * sinf(ha), hy = CTR - RH * cosf(ha);
    float mx = CTR + RM * sinf(ma), my = CTR - RM * cosf(ma);
    float kT = clampf(1 + (hy - CTR) / XW, 0, 1), kB = clampf(1 + (CTR - hy) / XW, 0, 1);
    float kL = clampf(1 + (mx - CTR) / XW, 0, 1), kR = clampf(1 + (CTR - mx) / XW, 0, 1);
    for (int i = 0; i < NS; i++) {
        float c = OFF + i * PITCH + 1;
        float b = fmaxf(1, rest(c, R) + ripple(c - hx, ph));       // hour: top/bottom
        lT[i] = (int16_t)floorf(bump(b, hy, c - hx, kT));
        lB[i] = (int16_t)floorf(bump(b, SZ - hy, c - hx, kB));
        b = fmaxf(1, rest(c, RMC) + ripple(c - my, ph));           // minute: left/right
        lL[i] = (int16_t)floorf(bump(b, mx, c - my, kL));
        lR[i] = (int16_t)floorf(bump(b, SZ - mx, c - my, kR));
    }
}

// ---------- WiFi + NTP ----------
static volatile bool time_valid = false;

static void on_sync(struct timeval *tv) {
    time_valid = true;
    ESP_LOGI(TAG, "NTP sync");
}

static void wifi_evt(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED)) {
        if (id == WIFI_EVENT_STA_DISCONNECTED) ESP_LOGW(TAG, "WiFi lost, retrying");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "WiFi up, " IPSTR, IP2STR(&((ip_event_got_ip_t *)data)->ip_info.ip));
        esp_netif_sntp_start();
    }
}

static void ntp_start(void) {
    setenv("TZ", CONFIG_SLAT_TZ, 1);
    tzset();
    if (!CONFIG_SLAT_WIFI_SSID[0]) { ESP_LOGW(TAG, "no WiFi SSID set, NTP disabled"); return; }

    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // SNTP: started once we have an IP; re-syncs every CONFIG_LWIP_SNTP_UPDATE_DELAY
    esp_sntp_config_t sc = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_SLAT_NTP_SERVER);
    sc.start = false;
    sc.sync_cb = on_sync;
    sc.smooth_sync = true;                  // adjtime() slews later corrections, no jumps
    ESP_ERROR_CHECK(esp_netif_sntp_init(&sc));

    wifi_init_config_t wc = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wc));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_evt, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_evt, NULL));
    wifi_config_t cfg = {0};
    strncpy((char *)cfg.sta.ssid, CONFIG_SLAT_WIFI_SSID, sizeof cfg.sta.ssid);
    strncpy((char *)cfg.sta.password, CONFIG_SLAT_WIFI_PASS, sizeof cfg.sta.password);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    ESP_ERROR_CHECK(esp_wifi_start());      // STA_START event -> connect
}

void app_main(void) {
    lcd_init();
    ntp_start();

    gpio_config_t btn = { .pin_bit_mask = 1ULL << PIN_BTN, .mode = GPIO_MODE_INPUT,
                          .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&btn);

    bool fast = true, prev_btn = true, auto_switched = false;
    float sim = 600;                        // minutes, 0..720
    int64_t last = esp_timer_get_time();
    int last_logged = -1;

    for (;;) {
        int64_t now = esp_timer_get_time();
        float dt = (now - last) / 1e6f; last = now;
        // wrap ripple phase in double so float precision doesn't decay with uptime
        float ph = (float)fmod(now / 1e6 * RW, 2 * M_PI);

        bool lvl = gpio_get_level(PIN_BTN);
        if (prev_btn && !lvl) { fast = !fast; auto_switched = true; ESP_LOGI(TAG, "%s", fast ? "fast forward" : "real time"); }
        prev_btn = lvl;
        if (time_valid && !auto_switched) { auto_switched = true; fast = false; }

        if (fast) sim = fmodf(sim + dt * 2, 720);
        else {
            struct timeval tv; struct tm tm;
            gettimeofday(&tv, NULL); localtime_r(&tv.tv_sec, &tm);
            sim = (tm.tm_hour % 12) * 60 + tm.tm_min + (tm.tm_sec + tv.tv_usec / 1e6f) / 60;
        }

        compute(sim, ph);
        render_frame();

        int hm = (int)sim;
        if (hm != last_logged) {
            last_logged = hm;
            ESP_LOGI(TAG, "%d:%02d", (hm / 60) ? hm / 60 : 12, hm % 60);
        }
        vTaskDelay(1);                      // let IDLE run / feed the watchdog
    }
}
