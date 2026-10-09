// Slat Watch — ESP32-2432S028R ("CYD", ILI9341 320x240), ESP-IDF 6.1
//
// v2 of the slat_watch.html port: three slat families at 120deg (hour 0deg,
// minute 120deg, second 240deg) filling the full 320x240 panel. Rendering is
// per-pixel in horizontal strips (320 x STRIP_H px, RGB565) double-buffered
// over SPI DMA, so no full framebuffer is needed.
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
#include "esp_attr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif_sntp.h"
#include "nvs_flash.h"

static const char *TAG = "slat";

// ---------- user config ----------
// ---- hardcoded WiFi (WPA2/WPA3). Leave WIFI_SSID "" to use menuconfig values.
#define WIFI_SSID   ""
#define WIFI_PASS   ""

// NTP server / TZ come from Kconfig (main/Kconfig.projbuild).
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

// ---------- geometry (v2: three slat families at 120deg, full 320x240) ----------
// Each family is a set of parallel 2px slats at a 3px pitch covering the whole
// screen. Slats grow in from both screen edges to a rest circle around the
// centre, ripple, and stretch across to the tip of their hand.
//   hour   = 0deg   (vertical slats, as in the original)
//   minute = 120deg
//   second = 240deg
#define CX      160.0f
#define CY      120.0f
static const float SIG = 3.6f, XW = 28.8f;                    // gaussian width, crossfade width
static const float RA = 3.6f, RL = 14.4f, RW = 5, RD = 16.8f; // ripple amp, wavelength, speed, decay
#define PITCH   3.0f
#define SLAT_W  2.0f
#define UMAX    201.0f                      // > half-diagonal (200), so slats cover every pixel
#define NSL     135                         // ceil(2*UMAX/PITCH)
#define FAST_RATE 2.0f                      // fast-forward speed, clock minutes per second

typedef struct {
    float ang_deg;                          // slat orientation
    float ring, tip;                        // rest-circle radius, hand-tip radius
    float nx, ny, dx, dy;                   // normal (selects slat), direction (along slat)
    float eT[NSL], eB[NSL];                 // per slat: black where v<=eT or v>=eB
} family_t;

static family_t fam[3] = {
    { .ang_deg =   0, .ring = 106, .tip = 66  },    // hour
    { .ang_deg = 120, .ring = 112, .tip = 90  },    // minute
    { .ang_deg = 240, .ring = 116, .tip = 104 },    // second
};

#define STRIP_H 24                          // 240/24 = 10 strips per frame
#define WHITE   0xFFFF
#define BLACK   0x0000

// ---------- math (same functions as the JS) ----------
static inline float clampf(float v, float a, float b) { return v < a ? a : v > b ? b : v; }
static inline float ripple(float d, float ph) {      // ph = t*RW, pre-wrapped
    float a = fabsf(d);
    return RA * cosf(a * 2 * (float)M_PI / RL - ph) * expf(-a / RD) * fminf(1, a / (2 * SIG));
}

// Same model as the original, expressed in slat coordinates (u across, v along,
// both centred on the screen). Original top length b = rest + ripple becomes
// end point v = -sqrt(r^2-o^2) + ripple; bottom mirrored. The bump pulls the
// end toward the hand tip's v, weighted by gaussian(u - hand u) and crossfade k.
static void compute_family(family_t *f, float hand_deg, float ph) {
    float ha = hand_deg * (float)M_PI / 180;
    float px = f->tip * sinf(ha), py = -f->tip * cosf(ha);   // tip, relative to centre
    float hu = px * f->nx + py * f->ny, hv = px * f->dx + py * f->dy;
    float kT = clampf(1 + hv / XW, 0, 1), kB = clampf(1 - hv / XW, 0, 1);
    for (int i = 0; i < NSL; i++) {
        float c = -UMAX + i * PITCH + SLAT_W / 2;              // slat centre line
        float o = fabsf(c), s = o < f->ring ? sqrtf(f->ring * f->ring - o * o) : 0;
        float d = c - hu, rp = ripple(d, ph);
        float g = expf(-d * d / (2 * SIG * SIG));
        float t = -s + rp, b = s - rp;
        if (hv > t) t += (hv - t) * kT * g;
        if (hv < b) b -= (b - hv) * kB * g;
        f->eT[i] = t; f->eB[i] = b;
    }
}

static void compute(float sim, float ph) {
    float sec = fmodf(sim * 60, 60), m = fmodf(sim, 60);
    compute_family(&fam[0], sim * 0.5f, ph);
    compute_family(&fam[1], m * 6, ph);
    compute_family(&fam[2], sec * 6, ph);
}

// ---------- display ----------
static esp_lcd_panel_handle_t panel;
static SemaphoreHandle_t buf_free;
static uint16_t *bufs[2];

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
        .max_transfer_sz = LCD_W * STRIP_H * 2 + 8,
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
        bufs[i] = heap_caps_malloc(LCD_W * STRIP_H * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        assert(bufs[i]);
    }
    for (int f = 0; f < 3; f++) {
        float a = fam[f].ang_deg * (float)M_PI / 180;
        fam[f].nx = cosf(a);  fam[f].ny = sinf(a);
        fam[f].dx = -sinf(a); fam[f].dy = cosf(a);
    }
    gpio_set_level(PIN_BL, 1);
}

// Per-pixel: for each family, find the slat under the pixel (u) and test the
// position along it (v) against that slat's end points. u and v step linearly
// across a row, so the inner loop is adds, one multiply and a compare.
static void IRAM_ATTR render_strip(uint16_t *buf, int y0) {
    for (int y = y0; y < y0 + STRIP_H; y++) {
        uint16_t *row = buf + (y - y0) * LCD_W;
        float ry = y + 0.5f - CY;
        float u[3], v[3];
        for (int f = 0; f < 3; f++) {
            float rx = 0.5f - CX;
            u[f] = rx * fam[f].nx + ry * fam[f].ny + UMAX;      // shifted to >= 0
            v[f] = rx * fam[f].dx + ry * fam[f].dy;
        }
        for (int x = 0; x < LCD_W; x++) {
            uint16_t px = WHITE;
            for (int f = 0; f < 3; f++) {
                float uu = u[f], vv = v[f];
                u[f] += fam[f].nx; v[f] += fam[f].dx;
                if (px == BLACK) continue;
                int i = (int)(uu * (1.0f / PITCH));
                if (uu - i * PITCH >= SLAT_W) continue;          // gap between slats
                if (vv <= fam[f].eT[i] || vv >= fam[f].eB[i]) px = BLACK;
            }
            row[x] = px;
        }
    }
}

static void render_frame(void) {
    static int which = 0;
    for (int y0 = 0; y0 < LCD_H; y0 += STRIP_H) {
        xSemaphoreTake(buf_free, portMAX_DELAY);
        uint16_t *b = bufs[which]; which ^= 1;
        render_strip(b, y0);
        esp_lcd_panel_draw_bitmap(panel, 0, y0, LCD_W, y0 + STRIP_H, b);
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
    const char *ssid = WIFI_SSID[0] ? WIFI_SSID : CONFIG_SLAT_WIFI_SSID;
    const char *pass = WIFI_SSID[0] ? WIFI_PASS : CONFIG_SLAT_WIFI_PASS;
    if (!ssid[0]) { ESP_LOGW(TAG, "no WiFi SSID set, NTP disabled"); return; }

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
    wifi_config_t cfg = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,   // accept WPA2, WPA2/WPA3 mixed, WPA3
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,           // WPA3: hunting-and-pecking + H2E
            .pmf_cfg = { .capable = true, .required = false },
        },
    };
    strncpy((char *)cfg.sta.ssid, ssid, sizeof cfg.sta.ssid);
    strncpy((char *)cfg.sta.password, pass, sizeof cfg.sta.password);
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

        if (fast) sim = fmodf(sim + dt * FAST_RATE, 720);
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
