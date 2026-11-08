// badge os: panel driver, drawing, buttons, Wi-Fi, ranging, settings and the home screen; the apps are app_*.c
// behind badge.h.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_flash.h"
#include "esp_random.h"
#include "freertos/semphr.h"
#include <stddef.h>
#include "usb/usb_host.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "badge.h"

static const char *TAG = "os";

// ---- Panel: Pervasive Displays 9.7" black/white/red, 960x672, two CoGs.
// The left CoG (master) drives columns 0-479, the right CoG (slave) columns 480-959.
#define HALF (PLANE / 2)  // what one CoG gets of one plane
#define REVERSE_BITS 1    // stock driver: reverse unless /data/hwconf display_pn == "84"

enum { CS_L = 13, CS_R = 3, BUSY_L = 12, BUSY_R = 8, DC = 10, RST = 11, SCK = 21, MOSI = 47 };
enum { LEFT = 1, RIGHT = 2, BOTH = 3 };

static spi_device_handle_t spi;

void ms(int t) { vTaskDelay(pdMS_TO_TICKS(t) + 1); } // at least t ms (1 kHz tick)

// One S3 SPI transaction carries at most 32 KB, so long data goes out in 4 KB pieces (CS stays low).
static void tx(const uint8_t *d, size_t n)
{
    for (size_t k; n; d += k, n -= k) {
        k = MIN(n, 4096);
        spi_transaction_t t = {.length = 8 * k, .tx_buffer = d};
        ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
    }
}

// Index byte with DC low, then data with DC high, to the CoGs in `cog`.
static void cmd(int cog, uint8_t index, const uint8_t *data, size_t n)
{
    gpio_set_level(DC, 0);
    if (cog & LEFT) gpio_set_level(CS_L, 0);
    if (cog & RIGHT) gpio_set_level(CS_R, 0);
    tx(&index, 1);
    gpio_set_level(CS_L, 1);
    gpio_set_level(CS_R, 1);
    gpio_set_level(DC, 1);
    if (cog & LEFT) gpio_set_level(CS_L, 0);
    if (cog & RIGHT) gpio_set_level(CS_R, 0);
    tx(data, n);
    gpio_set_level(CS_L, 1);
    gpio_set_level(CS_R, 1);
}
#define CMD(cog, index, ...) \
    cmd(cog, index, (const uint8_t[]){__VA_ARGS__}, sizeof((const uint8_t[]){__VA_ARGS__}))

// Each CoG pulls its BUSY low while it works. Logs how long each side stayed busy (-1: timeout).
static void wait_ready(const char *what, int timeout_ms)
{
    int64_t t0 = esp_timer_get_time(), t = 0;
    int l = -1, r = -1;
    while ((l < 0 || r < 0) && t < timeout_ms * 1000LL) {
        t = esp_timer_get_time() - t0;
        if (l < 0 && gpio_get_level(BUSY_L)) l = t / 1000;
        if (r < 0 && gpio_get_level(BUSY_R)) r = t / 1000;
        if (l < 0 || r < 0) ms(10);
    }
    ESP_LOGI(TAG, "%s: BUSY left %d ms, right %d ms", what, l, r);
}

static uint8_t rev(uint8_t b)
{
    b = b >> 4 | b << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    return (b & 0xAA) >> 1 | (b & 0x55) << 1;
}

// Rows y0..y1 of one plane into one CoG's RAM: its 60 bytes of every row, bit-reversed, the RAM pointer counting
// down from x 0x3B (RAM_RW {x1, y0 lo, 0x14 | y0 >> 8}: panel_lab experiment 02). plane NULL = all white.
static void send_rows(int cog, uint8_t index, const uint8_t *plane, int y0, int y1)
{
    static DMA_ATTR uint8_t buf[HALF];
    const uint8_t *src = plane ? plane + (cog == RIGHT ? W / 16 : 0) : NULL;
    int n = 0;
    for (int y = y0; y <= y1; y++)
        for (int x = 0; x < W / 16; x++) {
            uint8_t b = src ? src[y * (W / 8) + x] : 0;
            buf[n++] = REVERSE_BITS ? rev(b) : b;
        }
    CMD(cog, 0x12, 0x3B, y0 & 0xFF, 0x14 | y0 >> 8); // RAM_RW
    cmd(cog, index, buf, n);
}

// A panel session: epd_open (reset, frame data, register sequence, DC-DC up), pulls, epd_close. Same sequence as
// PDi EPD_Driver_GU_large and the stock epd.py, plus what the badge team documented for this B1 CoG
// (2026-Billboard_Badge/documentation/itc97-driver-reference.pdf): the 0x70 booster setting and the 0xB0 waveform.

// Short waveform for the B1 CoG: data/rendered_luts.py "lutb_base" from the badge team.
// 5 tables (paper, ink, red, unused, VCOM) x 10 groups of {level, on, off, ph2, on, off, ph2, repeat};
// a level byte is 2 bits per phase: 0 GND, 1 VDH, 2 VDL, 3 VDHR. A pull runs a fixed ~1070 ms
// whatever the table asks for, so one pull is a light dose and three make a clean image.
static const uint8_t lutb[400] = {
    // table 0: paper
    0x99, 0x0A, 0x0A, 0x01, 0x0A, 0x0A, 0x01, 0x03,
    0xAA, 0x2D, 0x03, 0x01, 0x2D, 0x03, 0x01, 0x03,
    0xAA, 0x2D, 0x03, 0x01, 0x2D, 0x03, 0x01, 0x03,
    0xAA, 0x2D, 0x03, 0x01, 0x2D, 0x03, 0x01, 0x03,
    0xAA, 0x2D, 0x03, 0x01, 0x2D, 0x03, 0x01, 0x03,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    // table 1: ink
    0x66, 0x0A, 0x0A, 0x01, 0x0A, 0x0A, 0x01, 0x03,
    0x44, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x44, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x44, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x44, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    // table 2: red
    0xEE, 0x0A, 0x0A, 0x01, 0x0A, 0x0A, 0x01, 0x03,
    0xAA, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0xFF, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    // table 3: unused
    0x00, 0x0A, 0x0A, 0x01, 0x0A, 0x0A, 0x01, 0x03,
    0x00, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x00, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x00, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x00, 0x2D, 0x01, 0x01, 0x2D, 0x01, 0x01, 0x03,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    // table 4: VCOM
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
};

// PDi Wide_Large's waits for this CoG family: the session opens in 348 ms and 0xB0 still latches.
static void epd_reset(void)
{
    gpio_set_level(RST, 1);
    ms(2);
    gpio_set_level(RST, 0);
    ms(4);
    gpio_set_level(RST, 1);
    ms(20);
    gpio_set_level(CS_L, 1);
    gpio_set_level(CS_R, 1);
    ms(5);
    CMD(BOTH, 0x70, 0x71, 0x36); // B1 booster setting. Without it a refresh draws ~2 A (badge team errata 9)
}

// Frame into CoG RAM: black into 0x10, 0x11 (red) all white. black NULL = all white.
static void epd_load(const uint8_t *black)
{
    CMD(BOTH, 0x13, 0x00, 0x3B, 0x00, 0x00, 0x9F, 0x02); // DUW: 60 bytes x 672 lines
    CMD(BOTH, 0x90, 0x00, 0x3B, 0x00, 0xA9);             // DRFW: whole panel
    for (int cog = LEFT; cog <= RIGHT; cog++) {
        send_rows(cog, 0x10, black, 0, H - 1);
        send_rows(cog, 0x11, NULL, 0, H - 1);
    }
}

static void dcdc_on(void)
{
    CMD(BOTH, 0x51, 0x50, 0x01); // DC-DC soft start
    for (int v = 1; v <= 4; v++) {
        CMD(BOTH, 0x09, 0x1F);
        CMD(BOTH, 0x51, 0x50, v);
        CMD(BOTH, 0x09, 0x9F);
        ms(2);
    }
    for (int v = 1; v <= 10; v++) {
        CMD(BOTH, 0x09, 0x1F);
        CMD(BOTH, 0x51, 0x0A, v);
        CMD(BOTH, 0x09, 0x9F);
        ms(2);
    }
    for (int v = 3; v <= 10; v++) {
        CMD(BOTH, 0x09, 0x7F);
        CMD(BOTH, 0x51, 0x0A, v);
        CMD(BOTH, 0x09, 0xFF);
        ms(2);
    }
    for (int v = 9; v >= 2; v--) {
        CMD(BOTH, 0x09, 0x7F);
        CMD(BOTH, 0x51, v, 0x0A);
        CMD(BOTH, 0x09, 0xFF);
        ms(2);
    }
    CMD(BOTH, 0x09, 0xFF);
    ms(100);
    wait_ready("dc-dc on", 10000); // never trigger into a booster that is still ramping
}

// After reset (and the frame data for a full session): the register sequence, a waveform table into 0xB0 instead
// of the panel's own 33 s one, DC-DC up.
static void epd_start(const uint8_t *lut)
{
    int hi = 2, lo = 10; // load strobe waits
    CMD(BOTH, 0x05, 0x7D);
    ms(50);
    CMD(BOTH, 0x05, 0x00);
    ms(1);
    CMD(BOTH, 0xD8, 0x80); // MS_SYNC
    CMD(BOTH, 0xD6, 0x00); // BVSS
    CMD(BOTH, 0xA7, 0x10);
    ms(hi);
    CMD(BOTH, 0xA7, 0x00);
    ms(lo);
    CMD(BOTH, 0x03, 0x00, 0x11); // OSC
    for (int cog = LEFT; cog <= RIGHT; cog++) {
        CMD(cog, 0x44, 0x00);
        CMD(cog, 0x45, 0x80);
        CMD(cog, 0xA7, 0x10);
        ms(hi);
        CMD(cog, 0xA7, 0x00);
        ms(lo);
        CMD(cog, 0x44, 0x06);
        CMD(cog, 0x45, 0x84); // temperature 0x50 + 2 * C: 26 C as the badge team uses
        CMD(cog, 0xA7, 0x10);
        ms(hi);
        CMD(cog, 0xA7, 0x00);
        ms(lo); // this strobe commits the OTP waveform; 0xB0 latches 10 ms after it (experiment 02)
    }
    CMD(BOTH, 0x60, 0x25); // TCON
    CMD(LEFT, 0x61, 0x01); // STV_DIR, master only
    CMD(BOTH, 0x02, 0x00); // VCOM
    cmd(BOTH, 0xB0, lut, 400); // one chance per session: rewriting it re-latches the slow one
    dcdc_on();
}

// A full session on the badge team's black/white table.
static void epd_open(const uint8_t *black)
{
    epd_reset();
    epd_load(black);
    epd_start(lutb);
}

// Run the latched waveform once on what is in RAM. Returns ms spent.
static int epd_pull(int timeout_ms)
{
    int64_t t0 = esp_timer_get_time();
    CMD(BOTH, 0x15, 0x3C);
    ms(5);
    wait_ready("pull", timeout_ms);
    return (esp_timer_get_time() - t0) / 1000;
}

static void epd_close(void)
{
    CMD(BOTH, 0x09, 0x7F); // DC-DC soft shutdown
    CMD(BOTH, 0x05, 0x7D);
    CMD(BOTH, 0x09, 0x00);
    ms(200);
    wait_ready("dc-dc off", 2000);
    gpio_set_level(DC, 0);
    gpio_set_level(CS_L, 0);
    gpio_set_level(RST, 0);
    gpio_set_level(CS_L, 1);
}

static void epd_init(void)
{
    gpio_set_level(DC, 1);
    gpio_set_level(RST, 1);
    gpio_set_level(CS_L, 1);
    gpio_set_level(CS_R, 1);
    gpio_config(&(gpio_config_t){.pin_bit_mask = BIT64(DC) | BIT64(RST) | BIT64(CS_L) | BIT64(CS_R),
                                 .mode = GPIO_MODE_OUTPUT});
    gpio_config(&(gpio_config_t){.pin_bit_mask = BIT64(BUSY_L) | BIT64(BUSY_R), .mode = GPIO_MODE_INPUT});
    spi_bus_config_t bus = {.mosi_io_num = MOSI, .sclk_io_num = SCK, .miso_io_num = -1,
                            .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 4096};
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t dev = {.clock_speed_hz = 20000000, .mode = 0, .spics_io_num = -1, .queue_size = 1};
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &spi));
}

// ---- Calm partial updates (panel_lab's driver)
// epd_partial(black)  black/white frame, no flash: only pixels that differ from what the panel shows are driven.
//                     One pull, ~0.48 s, however much changed. Ink comes out dark red; erased pixels leave a light
//                     ghost. Keeps the panel session (DC-DC) open for the next call.
// epd_sleep()         ends the partial session with the normal DC-DC soft shutdown.
// part_shown must match the film before the first epd_partial: ui_wipe leaves both all white.

#define PART_DRIVE 10 // frames of VDH/VDL per change (of 20 per pull): 10 for the film's health (owner); 6 is the least that reads clearly
_Static_assert(PART_DRIVE >= 1 && PART_DRIVE <= 19, "the pull has 20 frames and the last one must be GND");

static EXT_RAM_BSS_ATTR uint8_t part_shown[PLANE]; // what the film shows; after every pull the CoG's 0x11 holds the same
static int part_prev[2][4];       // per CoG, the byte box {x0, x1, y0, y1} the last pull changed; x1 < x0: none
static int part_n = -1;           // pulls in the open partial session, -1 = no session

// Transition table: old frame in 0x11, new frame in 0x10, table = (old << 1) | new. Table 1 (white->black) VDH and
// table 2 (black->white) VDL for the first PART_DRIVE frames, then GND; tables 0, 3 and VCOM at GND. Every group runs
// only its phases c and d (SRab 0): 10 groups x 2 frames = 20 frames = 475 ms, the shortest pull this CoG runs
// (a zero TP, RP 0 and SRab = SRcd = 0 all hang it). The last frame is GND, so no pixel keeps a charge.
static void part_table(uint8_t *t)
{
    for (int k = 0; k < 50; k++) memcpy(t + 8 * k, (const uint8_t[]){0, 1, 1, 0, 1, 1, 1, 1}, 8);
    for (int f = 0; f < PART_DRIVE; f++) {
        int g = f / 2, shift = f % 2 ? 0 : 2; // phase c: bits 3-2, phase d: bits 1-0
        t[(10 + g) * 8] |= 1 << shift;        // VDH
        t[(20 + g) * 8] |= 2 << shift;        // VDL
    }
}

// Session open on the transition table, then both RAMs = what the film shows.
static void part_open(void)
{
    static uint8_t lut[400];
    part_table(lut);
    epd_reset();
    epd_start(lut);
    CMD(BOTH, 0x13, 0x00, 0x3B, 0x00, 0x00, 0x9F, 0x02); // DUW whole panel
    CMD(BOTH, 0x90, 0x00, 0x3B, 0x00, 0xA9);             // DRFW whole panel; a shorter y1 copies the top rows down
    for (int cog = LEFT; cog <= RIGHT; cog++) send_rows(cog, 0x10, part_shown, 0, H - 1), send_rows(cog, 0x11, part_shown, 0, H - 1);
    for (int c = 0; c < 2; c++) part_prev[c][0] = 1, part_prev[c][1] = 0;
    part_n = 0;
}

// Byte box {x0, x1, y0, y1} of one CoG's half where frames a and b differ; x1 < x0 if they do not.
static void part_box(const uint8_t *a, const uint8_t *b, int cog, int *box)
{
    int off = cog == RIGHT ? W / 16 : 0;
    box[0] = W / 16, box[1] = -1, box[2] = H, box[3] = -1;
    for (int y = 0; y < H; y++) {
        const uint8_t *p = a + y * (W / 8) + off, *q = b + y * (W / 8) + off;
        if (!memcmp(p, q, W / 16)) continue;
        for (int x = 0; x < W / 16; x++)
            if (p[x] != q[x]) box[0] = MIN(box[0], x), box[1] = MAX(box[1], x);
        box[2] = MIN(box[2], y), box[3] = y;
    }
}

// Box b of one CoG's half of src into its 0x10 RAM: DUW + RAM_RW window (RAM x = 59 - screen byte, experiment 02),
// the rows in screen order, bit-reversed.
static void part_window(int cog, const uint8_t *src, const int *b)
{
    static DMA_ATTR uint8_t row[W / 16];
    int x0 = b[0], x1 = b[1], y0 = b[2], y1 = b[3], cs = cog == LEFT ? CS_L : CS_R;
    uint8_t index = 0x10;
    CMD(cog, 0x13, 59 - x1, 59 - x0, y0 & 0xFF, y0 >> 8, y1 & 0xFF, y1 >> 8);
    CMD(cog, 0x12, 59 - x0, y0 & 0xFF, 0x14 | y0 >> 8);
    gpio_set_level(DC, 0);
    gpio_set_level(cs, 0);
    tx(&index, 1);
    gpio_set_level(cs, 1);
    gpio_set_level(DC, 1);
    gpio_set_level(cs, 0);
    for (int y = y0; y <= y1; y++) {
        const uint8_t *s = src + y * (W / 8) + (cog == RIGHT ? W / 16 : 0);
        for (int x = x0; x <= x1; x++) row[x - x0] = REVERSE_BITS ? rev(s[x]) : s[x];
        tx(row, x1 - x0 + 1);
    }
    gpio_set_level(cs, 1);
}

static void epd_sleep(void)
{
    if (part_n >= 0) epd_close();
    part_n = -1;
}

static void epd_partial(const uint8_t *black)
{
    int64_t t0 = esp_timer_get_time();
    int now[2][4];
    bool any = false;
    for (int c = 0; c < 2; c++) part_box(black, part_shown, c + 1, now[c]), any |= now[c][1] >= now[c][0];
    if (!any) return ms(1); // nothing changed; still yield, apps call this in tight loops
    if (part_n == 60) epd_sleep(); // a fresh session every 60 pulls: sessions of 60 are proven, longer ones are not
    if (part_n < 0) part_open();
    // The CoG swaps 0x10 and 0x11 after every pull, so 0x11 already holds what the film shows and 0x10 holds the frame
    // before it. Rewrite 0x10 where that frame or the new one differs from the film: this pull's box and the last one.
    for (int c = 0; c < 2; c++) {
        int w[4], *p = part_prev[c];
        memcpy(w, now[c], sizeof w);
        if (p[1] >= p[0]) {
            if (w[1] < w[0]) memcpy(w, p, sizeof w);
            else w[0] = MIN(w[0], p[0]), w[1] = MAX(w[1], p[1]), w[2] = MIN(w[2], p[2]), w[3] = MAX(w[3], p[3]);
        }
        if (w[1] >= w[0]) part_window(c + 1, black, w);
        memcpy(p, now[c], sizeof now[c]);
    }
    CMD(BOTH, 0x13, 0x00, 0x3B, 0x00, 0x00, 0x9F, 0x02); // DUW back to the whole panel
    int pull = epd_pull(3000);
    memcpy(part_shown, black, PLANE);
    part_n++;
    ESP_LOGI(TAG, "partial: pull %d ms, total %d ms", pull, (int)((esp_timer_get_time() - t0) / 1000));
}

// ---- Drawing into fb
EXT_RAM_BSS_ATTR uint8_t fb[PLANE]; // in PSRAM: internal RAM is needed for Wi-Fi and TLS (Online update)

void box(int x, int y, int w, int h, bool ink)
{
    for (int r = MAX(y, 0); r < MIN(y + h, H); r++)
        for (int c = MAX(x, 0); c < MIN(x + w, W); c++)
            if (ink) fb[r * (W / 8) + c / 8] |= 0x80 >> (c & 7);
            else fb[r * (W / 8) + c / 8] &= ~(0x80 >> (c & 7));
}

void outline(int x, int y, int w, int h, int t)
{
    box(x, y, w, t, true), box(x, y + h - t, w, t, true), box(x, y, t, h, true), box(x + w - t, y, t, h, true);
}

#include "font8x8_basic.h" // public domain IBM VGA 8x8 font: one byte per row, bit 0 = leftmost pixel

void text(int x, int y, int scale, const char *s, bool ink)
{
    for (; *s; s++, x += 8 * scale)
        for (int r = 0; r < 8; r++)
            for (int c = 0; c < 8; c++)
                if (font8x8_basic[*s & 0x7F][r] >> c & 1) box(x + c * scale, y + r * scale, scale, scale, ink);
}

// ---- Panel frames: apps draw into fb, ui_frame shows it with one calm partial update (only changed pixels move).
static const char *bar_app = "Home"; // the name in the top bar
static const char *keys[4];           // ui_keys: what BTN_PLUS, BTN_MINUS, BTN_A, BTN_B do now
static volatile bool in_app;          // an app is running: MENU + and MENU - held together go home
static SemaphoreHandle_t panel_lock;  // one panel user at a time; going home waits for the frame in flight
static void draw_bar(void);

void ui_keys(const char *plus, const char *minus, const char *a, const char *b)
{
    keys[BTN_PLUS] = plus, keys[BTN_MINUS] = minus, keys[BTN_A] = a, keys[BTN_B] = b;
}

void ui_frame(int pulls)
{
    (void)pulls; // a partial update is one pull whatever was asked
    xSemaphoreTake(panel_lock, portMAX_DELAY);
    draw_bar();
    epd_partial(fb);
    xSemaphoreGive(panel_lock);
}

// Partial updates leave light ghosts of erased ink; a scene change shakes the film with the short black/white
// waveform (all black, then all white, ~2.8 s) and leaves the panel and the driver agreeing on white.
void ui_wipe(void)
{
    xSemaphoreTake(panel_lock, portMAX_DELAY);
    epd_sleep();
    memset(fb, 0xFF, sizeof fb);
    epd_open(fb);
    epd_pull(5000);
    epd_load(NULL); // both planes: after the pull 0x11 holds the black frame, which would select red
    epd_pull(5000);
    epd_close();
    memset(fb, 0, sizeof fb);
    memset(part_shown, 0, sizeof part_shown);
    xSemaphoreGive(panel_lock);
}

void ui_close(void)
{
    xSemaphoreTake(panel_lock, portMAX_DELAY);
    epd_sleep();
    xSemaphoreGive(panel_lock);
}

// MENU + and MENU - held together in an app: home, by a restart (apps never return by themselves), between frames
// so the panel is never cut off mid-refresh.
static void go_home(void)
{
    ESP_LOGI(TAG, "MENU + and MENU - held: going home");
    xSemaphoreTake(panel_lock, portMAX_DELAY);
    epd_sleep();
    esp_restart();
}

// ---- Buttons, active low with pull-ups, polled every 20 ms; each new press goes into a queue.
static QueueHandle_t btnq;
static volatile int held;

static void buttons(void *arg)
{
    static const int pin[4] = {40, 15, 39, 16}; // BTN_PLUS, BTN_MINUS, BTN_A, BTN_B (GPIOs of the printed names)
    gpio_config(&(gpio_config_t){.pin_bit_mask = BIT64(40) | BIT64(15) | BIT64(39) | BIT64(16), .mode = GPIO_MODE_INPUT,
                                 .pull_up_en = GPIO_PULLUP_ENABLE});
    for (int was = 0, both = 0;; ms(20)) {
        int now = 0;
        for (int k = 0; k < 4; k++) now |= (!gpio_get_level(pin[k])) << k;
        for (int k = 0; k < 4; k++)
            if (now & ~was & 1 << k) xQueueSend(btnq, &k, 0);
        held = was = now;
        both = (now & 3) == 3 ? both + 20 : 0; // MENU + and MENU - together, held 0.4 s so a game cannot slip into it
        if (both == 400 && in_app) go_home();
    }
}

int btn_get(int timeout_ms)
{
    int k;
    return xQueueReceive(btnq, &k, timeout_ms < 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms)) ? k : -1;
}

void btn_flush(void) { xQueueReset(btnq); }

int btn_held(void) { return held; }

// ---- Wi-Fi: ranging (below) keeps the radio busy on every screen; an app that needs it otherwise takes it over with
// wifi_on / wifi_connect, and the home screen gives it back to ranging.
#if __has_include("wifi_secret.h")
#include "wifi_secret.h" // WIFI_SSID / WIFI_PASS of the event Wi-Fi, gitignored; the release workflow writes it
#else
#define WIFI_SSID ""
#define WIFI_PASS ""
#endif
static EventGroupHandle_t wifi_ev; // bit 0: got an IP, bit 1: an FTM report (ranging)
static volatile int ftm_cm;        // last FTM result in cm, -1 failed
static volatile bool wifi_join;    // keep (re)joining the badge Wi-Fi

static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == IP_EVENT) xEventGroupSetBits(wifi_ev, 1);
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) xEventGroupClearBits(wifi_ev, 1);
    if (base == WIFI_EVENT && id == WIFI_EVENT_FTM_REPORT) { // ranging: one FTM session done
        wifi_event_ftm_report_t *r = data;
        ftm_cm = r->status == FTM_STATUS_SUCCESS ? (int)r->dist_est : -1;
        ESP_LOGI(TAG, "ftm %02x%02x: status %d, rtt %u ns, %u cm", r->peer_mac[4], r->peer_mac[5], r->status,
                 (unsigned)r->rtt_est, (unsigned)r->dist_est);
        xEventGroupSetBits(wifi_ev, 2);
    }
    if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED) && wifi_join)
        esp_wifi_connect();
}

void wifi_on(void)
{
    ranging_stop(); // an app that wants the radio gets it: ranging is back at the next home screen / reboot
    if (!wifi_ev) { // the driver is set up once and kept; on/off only starts and stops the radio
        wifi_ev = xEventGroupCreate();
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();
        wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&init));
        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, NULL);
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi, NULL);
    }
    esp_wifi_start(); // already running: harmless error
}

bool wifi_connect(int timeout_ms)
{
    wifi_config_t c = {0};
    cfg_get("ssid", (char *)c.sta.ssid, sizeof c.sta.ssid); // Settings app, else the event Wi-Fi
    cfg_get("pass", (char *)c.sta.password, sizeof c.sta.password);
    if (!c.sta.ssid[0]) return false;
    wifi_on();
    esp_wifi_set_config(WIFI_IF_STA, &c);
    wifi_join = true;
    esp_wifi_set_ps(WIFI_PS_NONE); // modem sleep slows downloads a lot (Online update); the radio is busy anyway
    esp_wifi_connect(); // not started yet: harmless error, the STA_START event connects
    return xEventGroupWaitBits(wifi_ev, 1, false, true, pdMS_TO_TICKS(timeout_ms)) & 1;
}

void wifi_off(void)
{
    if (!wifi_ev) return;
    ranging_stop();
    wifi_join = false;
    esp_wifi_disconnect();
    esp_wifi_stop();
    xEventGroupClearBits(wifi_ev, 1);
}

// ---- Ranging: badges running badge os find each other with ESP-NOW beacons on channel 1 and measure distances with
// Wi-Fi FTM (802.11mc round-trip time). Each beacon carries the sender's own ranges, so every badge also learns the
// distances between the others. The OS starts it at boot; it needs Wi-Fi in APSTA mode, the hidden SoftAP being this
// badge's FTM responder, so wifi_on/wifi_connect/wifi_off stop it. Background: RADAR_RESEARCH.md.
#define RNG_CH 1
#define RNG_FRAMES 16       // FTM frames per session (API: 0, 16, 24, 32, 64): two bursts of 8
#define RNG_OFFSET_CM 0     // what FTM reads at 0 m, subtracted: 0 on our badges (checked side by side)
#define RNG_KEEP 128        // pairs kept, own and shared: n badges share up to n * 8 (about 80 with 10 badges)
#define RNG_SHARE 8         // own ranges per beacon
#define RNG_STALE_US 30000000LL // a pair not heard of for 30 s is dropped

typedef struct __attribute__((packed)) {
    char magic[4]; // "BOSR"
    uint8_t ap[6]; // the sender's SoftAP MAC: where to aim FTM
    uint8_t n;
    struct __attribute__((packed)) {
        uint8_t mac[6];
        int16_t cm;
    } r[RNG_SHARE];
} rng_beacon_t;

// a -> b. Own entries (a = me) also keep b's SoftAP MAC and the last 4 FTM results (the minimum is the best guess:
// multipath only ever makes the path longer).
static struct {
    uint8_t a[6], b[6], ap[6];
    char name[13]; // own entries: b's username from its beacon, "" if none
    int16_t cm, hist[4];
    int64_t seen, at; // last beacon about this pair, last distance
} rng[RNG_KEEP];
static uint8_t rng_me[6];
static SemaphoreHandle_t rng_lock, rng_done;
static volatile bool rng_run;

static int rng_slot(const uint8_t *a, const uint8_t *b) // under rng_lock: the entry for a -> b, else the oldest, reset
{
    int old = 0;
    for (int i = 0; i < RNG_KEEP; i++) {
        if (!memcmp(rng[i].a, a, 6) && !memcmp(rng[i].b, b, 6)) return i;
        if (rng[i].seen < rng[old].seen) old = i;
    }
    memset(&rng[old], 0, sizeof rng[old]);
    memcpy(rng[old].a, a, 6), memcpy(rng[old].b, b, 6);
    rng[old].cm = rng[old].hist[0] = rng[old].hist[1] = rng[old].hist[2] = rng[old].hist[3] = -1;
    return old;
}

static void rng_recv(const esp_now_recv_info_t *info, const uint8_t *d, int len)
{
    const rng_beacon_t *m = (const void *)d;
    int head = offsetof(rng_beacon_t, r);
    if (len < head || memcmp(m->magic, "BOSR", 4) || m->n > RNG_SHARE || len < head + m->n * (int)sizeof m->r[0]) return;
    int64_t now = esp_timer_get_time();
    xSemaphoreTake(rng_lock, portMAX_DELAY);
    int i = rng_slot(rng_me, info->src_addr); // a badge nearby: range it
    if (!rng[i].seen) ESP_LOGI(TAG, "ranging: new badge %02x%02x", info->src_addr[4], info->src_addr[5]);
    memcpy(rng[i].ap, m->ap, 6);
    rng[i].seen = now;
    int tail = head + m->n * (int)sizeof m->r[0]; // the username follows the ranges (badges before v0.7 send none)
    if (len >= tail + 12) memcpy(rng[i].name, d + tail, 12), rng[i].name[12] = 0;
    for (int k = 0; k < m->n; k++) { // what it measured
        int j = rng_slot(info->src_addr, m->r[k].mac);
        rng[j].cm = m->r[k].cm, rng[j].seen = rng[j].at = now;
    }
    xSemaphoreGive(rng_lock);
}

static void rng_task(void *arg)
{
    static const uint8_t all[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    int64_t beacon = 0;
    for (int next = 0; rng_run;) {
        int64_t now = esp_timer_get_time();
        if (now >= beacon) {
            static uint8_t buf[sizeof(rng_beacon_t) + 12];
            rng_beacon_t m = {.magic = {'B', 'O', 'S', 'R'}};
            esp_wifi_get_mac(WIFI_IF_AP, m.ap);
            xSemaphoreTake(rng_lock, portMAX_DELAY);
            for (int i = 0; i < RNG_KEEP && m.n < RNG_SHARE; i++)
                if (!memcmp(rng[i].a, rng_me, 6) && rng[i].cm >= 0 && now - rng[i].at < RNG_STALE_US)
                    memcpy(m.r[m.n].mac, rng[i].b, 6), m.r[m.n++].cm = rng[i].cm;
            xSemaphoreGive(rng_lock);
            int len = offsetof(rng_beacon_t, r) + m.n * sizeof m.r[0];
            char name[13] = "";
            cfg_get("name", name, sizeof name);
            memcpy(buf, &m, len);
            memcpy(buf + len, name, 12); // username, zero padded
            esp_now_send(all, buf, len + 12);
            beacon = now + 800000 + esp_random() % 400000; // ~1 s, jittered so two badges do not keep colliding
        }
        // FTM to the next badge heard from in the last 10 s, round robin
        uint8_t b[6], ap[6];
        int found = -1;
        xSemaphoreTake(rng_lock, portMAX_DELAY);
        for (int k = 0; k < RNG_KEEP && found < 0; k++) {
            int i = (next + k) % RNG_KEEP;
            if (!memcmp(rng[i].a, rng_me, 6) && now - rng[i].seen < 10000000) found = i, memcpy(b, rng[i].b, 6), memcpy(ap, rng[i].ap, 6);
        }
        xSemaphoreGive(rng_lock);
        if (found < 0) {
            ms(100);
            continue;
        }
        next = found + 1;
        wifi_ftm_initiator_cfg_t cfg = {.channel = RNG_CH, .frm_count = RNG_FRAMES, .burst_period = 2};
        memcpy(cfg.resp_mac, ap, 6);
        xEventGroupClearBits(wifi_ev, 2);
        esp_err_t e = esp_wifi_ftm_initiate_session(&cfg);
        if (e != ESP_OK) ESP_LOGW(TAG, "ftm %02x%02x: start failed: %s", ap[4], ap[5], esp_err_to_name(e));
        if (e == ESP_OK &&
            (xEventGroupWaitBits(wifi_ev, 2, true, true, pdMS_TO_TICKS(2000)) & 2) && ftm_cm >= 0) {
            xSemaphoreTake(rng_lock, portMAX_DELAY);
            int i = rng_slot(rng_me, b), cm = MAX(0, ftm_cm - RNG_OFFSET_CM);
            memmove(rng[i].hist + 1, rng[i].hist, 3 * sizeof rng[i].hist[0]);
            rng[i].hist[0] = cm;
            for (int k = 1; k < 4; k++)
                if (rng[i].hist[k] >= 0) cm = MIN(cm, rng[i].hist[k]);
            rng[i].cm = cm, rng[i].at = esp_timer_get_time();
            if (!rng[i].seen) rng[i].seen = rng[i].at;
            xSemaphoreGive(rng_lock);
        } else {
            esp_wifi_ftm_end_session();
        }
        ms(50 + esp_random() % 100); // air for the other badges' bursts
    }
    xSemaphoreGive(rng_done);
    vTaskDelete(NULL);
}

void ranging_start(void)
{
    if (rng_run) return;
    if (!rng_lock) rng_lock = xSemaphoreCreateMutex(), rng_done = xSemaphoreCreateBinary();
    wifi_on();
    esp_wifi_stop();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    wifi_config_t ap = {.ap = {.ssid = "badge-os", .ssid_len = 8, .channel = RNG_CH, .authmode = WIFI_AUTH_OPEN,
                               .ssid_hidden = 1, .max_connection = 1, .beacon_interval = 100, .ftm_responder = true}};
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE); // modem sleep would drop beacons
    esp_wifi_set_channel(RNG_CH, WIFI_SECOND_CHAN_NONE);
    esp_read_mac(rng_me, ESP_MAC_WIFI_STA);
    memset(rng, 0, sizeof rng);
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_register_recv_cb(rng_recv);
    esp_now_add_peer(&(esp_now_peer_info_t){.peer_addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, .channel = RNG_CH,
                                            .ifidx = WIFI_IF_STA});
    rng_run = true;
    xTaskCreate(rng_task, "ranging", 4096, NULL, 4, NULL);
}

void ranging_stop(void)
{
    if (!rng_run) return;
    rng_run = false;
    xSemaphoreTake(rng_done, pdMS_TO_TICKS(5000));
    esp_now_deinit();
    esp_wifi_stop();
    esp_wifi_set_mode(WIFI_MODE_STA); // as wifi_on() left it
}

int ranging_get(range_t *out, int max)
{
    if (!rng_lock) return 0;
    int n = 0;
    int64_t now = esp_timer_get_time();
    xSemaphoreTake(rng_lock, portMAX_DELAY);
    for (int i = 0; i < RNG_KEEP && n < max; i++) {
        if (!rng[i].seen || now - rng[i].seen > RNG_STALE_US) continue;
        memcpy(out[n].a, rng[i].a, 6), memcpy(out[n].b, rng[i].b, 6);
        out[n].cm = rng[i].cm;
        out[n++].age_ms = (now - (rng[i].cm >= 0 ? rng[i].at : rng[i].seen)) / 1000;
    }
    xSemaphoreGive(rng_lock);
    return n;
}

void ranging_me(uint8_t mac[6]) { esp_read_mac(mac, ESP_MAC_WIFI_STA); }

bool ranging_name(const uint8_t mac[6], char name[13])
{
    bool ok = false;
    *name = 0;
    if (!rng_lock) return false;
    xSemaphoreTake(rng_lock, portMAX_DELAY);
    for (int i = 0; i < RNG_KEEP && !ok; i++)
        if (!memcmp(rng[i].a, rng_me, 6) && !memcmp(rng[i].b, mac, 6) && rng[i].name[0]) strcpy(name, rng[i].name), ok = true;
    xSemaphoreGive(rng_lock);
    return ok;
}

void badge_id(char id[5])
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(id, "%02X%02X", mac[4], mac[5]); // as Radar shows it
}

// ---- Settings (the Settings app writes them): short strings in NVS namespace "cfg"; the Badge app's picture
// raw in flash at IMG_ADDR, right after the coredump partition. partitions.csv lists it as "img", but badges updated
// over the air keep their old partition table, so the code uses the address, not the table.
#define IMG_ADDR 0x630000
#define IMG_MAGIC 0x31474D49 // "IMG1", written last: a cut-off write reads as no picture

int cfg_get(const char *key, char *out, int n)
{
    nvs_handle_t h;
    size_t len = n;
    if (nvs_open("cfg", NVS_READONLY, &h) == ESP_OK) {
        esp_err_t e = nvs_get_str(h, key, out, &len);
        nvs_close(h);
        if (e == ESP_OK) return strlen(out);
    }
    const char *def = !strcmp(key, "ssid") ? WIFI_SSID : !strcmp(key, "pass") ? WIFI_PASS : NULL; // event Wi-Fi
    if (n) *out = 0;
    if (!def) return -1;
    snprintf(out, n, "%s", def);
    return strlen(out);
}

bool cfg_set(const char *key, const char *val)
{
    nvs_handle_t h;
    if (nvs_open("cfg", NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t e = val ? nvs_set_str(h, key, val) : nvs_erase_key(h, key);
    if (e == ESP_ERR_NVS_NOT_FOUND) e = ESP_OK;
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e == ESP_OK;
}

bool cfg_get_image(uint8_t *img)
{
    uint32_t magic = 0;
    return esp_flash_read(NULL, &magic, IMG_ADDR, 4) == ESP_OK && magic == IMG_MAGIC &&
           esp_flash_read(NULL, img, IMG_ADDR + 4, IMG_BYTES) == ESP_OK;
}

bool cfg_set_image(const uint8_t *img)
{
    if (esp_flash_erase_region(NULL, IMG_ADDR, (4 + IMG_BYTES + 4095) & ~4095) != ESP_OK) return false;
    uint32_t magic = IMG_MAGIC;
    return !img || (esp_flash_write(NULL, img, IMG_ADDR + 4, IMG_BYTES) == ESP_OK &&
                    esp_flash_write(NULL, &magic, IMG_ADDR, 4) == ESP_OK);
}

// ---- SAO add-ons share one I2C bus: GPIO17 SDA / GPIO18 SCL, 5.1k pull-ups on the badge (DEVICE_INFO.md).
// The knob add-on is an AS5600 magnetic encoder at 0x36 (its analog out is on the port's GPIO1, a NeoPixel on GPIO2).
static i2c_master_bus_handle_t sao_bus;
static i2c_master_dev_handle_t knob;

static void sao_init(void) // boot: open the bus and log what answers on it
{
    if (i2c_new_master_bus(&(i2c_master_bus_config_t){.i2c_port = -1, .sda_io_num = 17, .scl_io_num = 18,
                                                      .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7},
                           &sao_bus) != ESP_OK) {
        ESP_LOGE(TAG, "SAO I2C bus failed");
        return;
    }
    for (int a = 0x08; a < 0x78; a++)
        if (i2c_master_probe(sao_bus, a, 20) == ESP_OK) ESP_LOGI(TAG, "SAO I2C device at 0x%02x", a);
}

// The AS5600 also drives its angle as a voltage (0-3.3 V over a turn) on its port's GPIO1: read that when the knob
// does not answer on I2C. SAO1's GPIO1 is GPIO2 (ADC1). A floating pin follows the pull resistors, a driven one does
// not: that tells a plugged knob from an empty slot.
static int knob_analog(int *up_mv, int *down_mv)
{
    enum { PIN = 2 };
    static adc_oneshot_unit_handle_t adc;
    static adc_cali_handle_t cali;
    static adc_channel_t ch;
    if (!adc) {
        adc_unit_t unit;
        adc_oneshot_io_to_channel(PIN, &unit, &ch);
        adc_oneshot_new_unit(&(adc_oneshot_unit_init_cfg_t){.unit_id = unit}, &adc);
        adc_oneshot_config_channel(adc, ch, &(adc_oneshot_chan_cfg_t){.atten = ADC_ATTEN_DB_12});
        adc_cali_create_scheme_curve_fitting(&(adc_cali_curve_fitting_config_t){.unit_id = unit, .chan = ch,
                                                                                .atten = ADC_ATTEN_DB_12}, &cali);
    }
    if (!cali) return -1;
    int mv[2];
    for (int k = 0; k < 2; k++) { // with the pull-up, then with the pull-down
        gpio_set_pull_mode(PIN, k ? GPIO_PULLDOWN_ONLY : GPIO_PULLUP_ONLY);
        int sum = 0, v;
        for (int i = 0; i < 4; i++) sum += adc_oneshot_get_calibrated_result(adc, cali, ch, &v) == ESP_OK ? v : 0;
        mv[k] = sum / 4;
    }
    gpio_set_pull_mode(PIN, GPIO_FLOATING);
    if (up_mv) *up_mv = mv[0], *down_mv = mv[1];
    if (abs(mv[0] - mv[1]) > 300) return -1; // floating: no knob
    return MIN((mv[0] + mv[1]) / 2 * 4096 / 3300, 4095);
}

int knob_angle(void)
{
    static int64_t next_probe;
    if (!sao_bus) return knob_analog(NULL, NULL);
    if (!knob) { // not seen on I2C yet: look again at most every 2 s, use the analog out meanwhile
        if (esp_timer_get_time() < next_probe) return knob_analog(NULL, NULL);
        next_probe = esp_timer_get_time() + 2000000;
        if (i2c_master_probe(sao_bus, 0x36, 20) != ESP_OK) return knob_analog(NULL, NULL);
        i2c_master_bus_add_device(sao_bus, &(i2c_device_config_t){.dev_addr_length = I2C_ADDR_BIT_LEN_7,
                                                                   .device_address = 0x36, .scl_speed_hz = 100000},
                                  &knob);
        ESP_LOGI(TAG, "AS5600 knob found");
    }
    uint8_t reg = 0x0B, d[2]; // STATUS: without a magnet (MD, bit 5) RAW ANGLE is noise, so there is no knob
    if (i2c_master_transmit_receive(knob, &reg, 1, d, 1, 20) != ESP_OK || !(d[0] & 0x20)) return -1;
    reg = 0x0C; // RAW ANGLE, 12 bits, high byte first
    if (i2c_master_transmit_receive(knob, &reg, 1, d, 2, 20) != ESP_OK) return -1;
    return (d[0] & 0x0F) << 8 | d[1];
}

// ---- Battery: GPIO14 (BAT_SENS) reads half of the LDO input through a 10k/10k divider. On battery that is the
// LiPo voltage; on USB the battery is cut off from it (power path) and it reads about VBUS - 0.3 V.
int battery_pct(void)
{
    static adc_oneshot_unit_handle_t adc;
    static adc_cali_handle_t cali;
    static adc_channel_t ch;
    if (!adc) {
        adc_unit_t unit;
        adc_oneshot_io_to_channel(14, &unit, &ch);
        adc_oneshot_new_unit(&(adc_oneshot_unit_init_cfg_t){.unit_id = unit}, &adc);
        adc_oneshot_config_channel(adc, ch, &(adc_oneshot_chan_cfg_t){.atten = ADC_ATTEN_DB_12});
        adc_cali_create_scheme_curve_fitting(&(adc_cali_curve_fitting_config_t){.unit_id = unit, .chan = ch,
                                                                                .atten = ADC_ATTEN_DB_12}, &cali);
    }
    static int last = -2; // a failed read (ADC2 is shared with Wi-Fi, which ranging keeps busy) returns the last good one
    int mv, sum = 0;
    for (int k = 0; k < 8; k++) {
        if (!cali || adc_oneshot_get_calibrated_result(adc, cali, ch, &mv) != ESP_OK) return last;
        sum += mv;
    }
    int vin = sum / 8 * 2;
    if (vin > 4400) return last = -1;
    // ponytail: a typical LiPo rest-voltage curve, not this badge's cell; good to ~10 %
    static const int v[] = {3300, 3500, 3600, 3700, 3800, 3900, 4000, 4100, 4200};
    static const int p[] = {0, 3, 10, 25, 45, 62, 77, 90, 100};
    if (vin <= v[0]) return last = 0;
    for (int i = 1; i < 9; i++)
        if (vin < v[i]) return last = p[i - 1] + (p[i] - p[i - 1]) * (vin - v[i - 1]) / (v[i] - v[i - 1]);
    return last = 100;
}

// ---- Top bar (rows 0..TOP-1) on every screen, drawn over fb by ui_frame. The buttons sit two in each top corner,
// MENU + above A on the left, MENU - above B on the right: each label (ui_keys) sits beside its button, with a tab at
// the edge pointing at it. In the middle: the app's name, then the username, OS version and battery.
static void draw_bar(void)
{
    static const int ky[4] = {4, 4, 26, 26}; // label rows of BTN_PLUS, BTN_MINUS, BTN_A, BTN_B
    static int pct = -2;
    static int64_t next;
    static char name[13];
    if (esp_timer_get_time() >= next) { // every 30 s, so the bar sits still
        pct = battery_pct();
        cfg_get("name", name, sizeof name);
        next = esp_timer_get_time() + 30000000;
    }
    char mid[64], bat[16] = "USB";
    int n = snprintf(mid, sizeof mid, "%s%sv%s", name, *name ? "  " : "", esp_app_get_description()->version);
    if (pct >= 0) snprintf(bat, sizeof bat, "%d%%", pct);
    int x = (W - n * 16 - (pct == -2 ? 0 : 32 + 38 + (int)strlen(bat) * 16)) / 2; // centre: text, battery, its label
    box(0, 0, W, TOP, false);
    for (int k = 0; k < 4; k++) {
        if (!keys[k] || !*keys[k]) continue;
        bool left = k == BTN_PLUS || k == BTN_A;
        box(left ? 0 : W - 6, ky[k], 6, 16, true);
        text(left ? 14 : W - 14 - (int)strlen(keys[k]) * 16, ky[k], 2, keys[k], true);
    }
    text((W - (int)strlen(bar_app) * 16) / 2, 4, 2, bar_app, true);
    text(x, 26, 2, mid, true);
    if (pct != -2) { // a battery outline filled to the charge (full on USB), then "87%" or "USB"
        x += n * 16 + 32;
        outline(x, 27, 30, 14, 2);
        box(x + 30, 31, 3, 6, true);
        box(x + 4, 31, pct < 0 ? 22 : 22 * pct / 100, 6, true);
        text(x + 38, 26, 2, bat, true);
    }
    box(0, TOP - 4, W, 2, true);
}

// ---- Home screen: iOS-style grid of icons. MENU + / MENU - move left / right, A or B opens the app.
#define ICON 176 // icon square, px
#define PITCH 224 // icon to icon
#define X0 56     // (960 - 4 * 176 - 3 * 48) / 2: four icons per row
#define Y0 80 // below the top bar
#define ROW 300

static void icon_base(int x, int y) // black square with rounded corners
{
    enum { R = 28 };
    box(x, y, ICON, ICON, true);
    for (int r = 0; r < R; r++)
        for (int c = 0; c < R; c++)
            if ((R - c) * (R - c) + (R - r) * (R - r) > R * R)
                box(x + c, y + r, 1, 1, false), box(x + ICON - 1 - c, y + r, 1, 1, false),
                    box(x + c, y + ICON - 1 - r, 1, 1, false), box(x + ICON - 1 - c, y + ICON - 1 - r, 1, 1, false);
}

static void icon_update(int x, int y) // download arrow into a tray
{
    icon_base(x, y);
    box(x + 78, y + 32, 20, 60, false);
    for (int i = 0; i < 36; i++) box(x + 88 - (36 - i), y + 92 + i, 2 * (36 - i), 1, false);
    box(x + 36, y + 136, 104, 14, false);
}

static void icon_pong(int x, int y) // two paddles and a ball
{
    icon_base(x, y);
    box(x + 28, y + 40, 16, 64, false);
    box(x + 132, y + 76, 16, 64, false);
    box(x + 80, y + 68, 20, 20, false);
    for (int k = 0; k < 6; k++) box(x + 86, y + 18 + k * 26, 6, 12, false);
}

static void icon_snake(int x, int y) // a snake of white squares heading for a dot
{
    icon_base(x, y);
    static const int cells[][2] = {{1, 4}, {2, 4}, {3, 4}, {3, 3}, {3, 2}, {4, 2}, {5, 2}};
    for (int i = 0; i < 7; i++) box(x + cells[i][0] * 24 + 2, y + cells[i][1] * 24 + 2, 20, 20, false);
    box(x + 5 * 24 + 8, y + 5 * 24 + 8, 8, 8, false); // food
}

static void icon_life(int x, int y) // a glider
{
    icon_base(x, y);
    static const int cells[][2] = {{1, 0}, {2, 1}, {0, 2}, {1, 2}, {2, 2}};
    for (int i = 0; i < 5; i++) box(x + 28 + cells[i][0] * 40 + 2, y + 28 + cells[i][1] * 40 + 2, 36, 36, false);
}

static void icon_aliens(int x, int y) // a flying saucer over a gun
{
    icon_base(x, y);
    box(x + 72, y + 40, 32, 16, false);  // dome
    box(x + 40, y + 56, 96, 16, false);  // saucer
    box(x + 56, y + 72, 64, 8, false);
    box(x + 84, y + 100, 8, 24, false);  // shot
    box(x + 64, y + 136, 48, 12, false); // gun
    box(x + 82, y + 128, 12, 8, false);
}

static void icon_blocks(int x, int y) // a falling L over a stacked floor
{
    icon_base(x, y);
    static const int cells[][2] = {{2, 0}, {2, 1}, {2, 2}, {3, 2}, {0, 4}, {1, 4}, {3, 4}, {4, 4}, {0, 5}, {1, 5}, {2, 5}, {4, 5}};
    for (int i = 0; i < 12; i++) box(x + 28 + cells[i][0] * 24 + 2, y + 16 + cells[i][1] * 24 + 2, 20, 20, false);
}

static void icon_rocks(int x, int y) // a little ship among rocks
{
    icon_base(x, y);
    for (int r = 0; r < 24; r++) box(x + 88 - r / 2, y + 76 + r, r + 1, 1, false); // ship, nose up
    outline(x + 28, y + 28, 40, 36, 6), outline(x + 116, y + 40, 32, 28, 6), outline(x + 112, y + 120, 44, 36, 6);
    box(x + 36, y + 124, 20, 20, false);
}

static void icon_source(int x, int y) // three QR finder squares and some modules
{
    icon_base(x, y);
    static const int f[][2] = {{28, 28}, {100, 28}, {28, 100}};
    for (int i = 0; i < 3; i++) {
        box(x + f[i][0], y + f[i][1], 48, 48, false);
        box(x + f[i][0] + 8, y + f[i][1] + 8, 32, 32, true);
        box(x + f[i][0] + 16, y + f[i][1] + 16, 16, 16, false);
    }
    static const int m[][2] = {{100, 100}, {124, 100}, {112, 112}, {136, 124}, {100, 136}, {124, 136}};
    for (int i = 0; i < 6; i++) box(x + m[i][0], y + m[i][1], 12, 12, false);
}

static void icon_feed(int x, int y) // news feed: a dot and two arcs from the bottom left corner
{
    icon_base(x, y);
    int cx = x + 44, cy = y + ICON - 44;
    for (int r = 0; r < ICON; r++)
        for (int k = 0; k < ICON; k++) {
            int dx = x + k - cx, dy = cy - (y + r), d2 = dx * dx + dy * dy;
            if (dx >= -12 && dy >= -12 && (d2 < 14 * 14 || (d2 > 50 * 50 && d2 < 68 * 68) || (d2 > 86 * 86 && d2 < 104 * 104)))
                box(x + k, y + r, 1, 1, false);
        }
}

static void icon_badge(int x, int y) // a name tag: a white card with a black band and a name line
{
    icon_base(x, y);
    box(x + 24, y + 40, 128, 96, false);
    box(x + 24, y + 40, 128, 28, true);
    box(x + 40, y + 96, 96, 10, true);
}

static void icon_settings(int x, int y) // a gear
{
    icon_base(x, y);
    int c = ICON / 2;
    for (int r = 0; r < ICON; r++)
        for (int k = 0; k < ICON; k++) {
            int dx = k - c, dy = r - c, d2 = dx * dx + dy * dy;
            bool tooth = d2 < 64 * 64 && (abs(dx) < 12 || abs(dy) < 12 || abs(dx - dy) < 16 || abs(dx + dy) < 16);
            if ((d2 < 46 * 46 || tooth) && d2 > 20 * 20) box(x + k, y + r, 1, 1, false);
        }
}

static void icon_weather(int x, int y) // a sun behind a cloud
{
    icon_base(x, y);
    for (int r = 0; r < ICON; r++)
        for (int k = 0; k < ICON; k++) {
            int sx = k - 112, sy = r - 64, a = k - 70, b = r - 108, d = k - 112, e = r - 112;
            if (sx * sx + sy * sy < 34 * 34 || a * a + b * b < 30 * 30 || d * d + e * e < 24 * 24) box(x + k, y + r, 1, 1, false);
        }
    box(x + 40, y + 112, 96, 24, false);
}

static void icon_news(int x, int y) // a page of headlines
{
    icon_base(x, y);
    box(x + 32, y + 28, 112, 120, false);
    box(x + 44, y + 40, 88, 16, true);
    for (int k = 0; k < 4; k++) box(x + 44, y + 68 + k * 18, k == 3 ? 56 : 88, 6, true);
}

static void icon_radar(int x, int y) // rings, a sweep line and two blips
{
    icon_base(x, y);
    int c = ICON / 2;
    for (int r = 0; r < ICON; r++)
        for (int k = 0; k < ICON; k++) {
            int d2 = (k - c) * (k - c) + (r - c) * (r - c);
            if ((d2 > 66 * 66 && d2 < 70 * 70) || (d2 > 42 * 42 && d2 < 46 * 46) || d2 < 6 * 6) box(x + k, y + r, 1, 1, false);
        }
    for (int t = 0; t < 48; t++) box(x + c + t, y + c - t, 4, 4, false); // sweep, to the top right
    box(x + c - 52, y + c - 20, 12, 12, false), box(x + c + 20, y + c + 36, 12, 12, false);
}

typedef struct {
    const char *name;
    void (*icon)(int x, int y);
    void (*run)(void);
} app_t;

static void icon_scanner(int x, int y) // a shield with a check: security scan
{
    icon_base(x, y);
    int cx = x + ICON / 2;
    outline(cx - 44, y + 34, 88, 64, 5);          // shield body
    box(cx - 44, y + 98, 88, 8, true);            // base band 1
    box(cx - 44, y + 106, 88, 8, true);           // base band 2
    // check mark (secure)
    box(cx - 22, y + 56, 10, 24, true);
    box(cx - 12, y + 80, 10, 10, true);
    box(cx - 2, y + 70, 10, 20, true);
    box(cx + 8, y + 56, 10, 16, true);
}

static const app_t apps[] = {
    {"Badge", icon_badge, app_badge}, // the name tag first: what a badge is for
    {"Update", icon_update, app_update},
    {"Pong", icon_pong, app_pong},
    {"Snake", icon_snake, app_snake},
    {"Life", icon_life, app_life},
    {"SpaceInv", icon_aliens, app_aliens},
    {"Tetris", icon_blocks, app_blocks},
    {"Asteroids", icon_rocks, app_rocks},
    {"Radar", icon_radar, app_radar},
    {"Source", icon_source, app_source},
    {"Hackaday", icon_feed, app_hackaday},
    {"News", icon_news, app_news},
    {"Weather", icon_weather, app_weather},
    {"Settings", icon_settings, app_settings},
    {"Scanner", icon_scanner, app_scanner},
};
void app_scanner(void);
#define NAPPS ((int)(sizeof apps / sizeof *apps))

static void draw_home(int sel)
{
    char ver[48];
    snprintf(ver, sizeof ver, "badge os v%s", esp_app_get_description()->version);
    memset(fb, 0, sizeof fb);
    int page = sel / 8, pages = (NAPPS + 7) / 8; // 8 icons a page; moving past the last one turns the page
    for (int i = page * 8; i < MIN(NAPPS, page * 8 + 8); i++) {
        int x = X0 + i % 4 * PITCH, y = Y0 + i % 8 / 4 * ROW;
        apps[i].icon(x, y);
        text(x + ICON / 2 - (int)strlen(apps[i].name) * 12, y + ICON + 24, 3, apps[i].name, true);
        if (i == sel) outline(x - 16, y - 16, ICON + 32, ICON + 32, 8);
    }
    text(W - (int)strlen(ver) * 24 - 24, H - 48, 3, ver, true);
    for (int p = 0; pages > 1 && p < pages; p++) { // page dots, bottom centre: the current one filled
        int x = W / 2 - pages * 16 + p * 32;
        if (p == page) box(x, H - 44, 16, 16, true);
        else outline(x, H - 44, 16, 16, 3);
    }
}

static void home(void)
{
    ui_keys("< Left", "Right >", "Open", "Open");
    btn_flush();
    for (int sel = 0, first = 1;;) {
        draw_home(sel);
        ui_frame(1);
        if (first) esp_ota_mark_app_valid_cancel_rollback(), first = 0; // this version boots: keep it (rollback)
        int b = btn_get(10000), open = 0;
        if (b < 0) { // the screen sits still: end the panel session, then wait as long as it takes
            ui_close();
            b = btn_get(-1);
        }
        // Every press counts at once, also those made during the last refresh; the next frame shows where they led.
        for (; b >= 0 && !open; b = btn_get(0)) {
            ESP_LOGI(TAG, "home: button %d on %s", b, apps[sel].name);
            if (b == BTN_PLUS) sel = (sel + NAPPS - 1) % NAPPS; // MENU + is the left one: move left
            else if (b == BTN_MINUS) sel = (sel + 1) % NAPPS; // MENU - is the right one: move right
            else open = 1;
        }
        if (!open) continue;
        ESP_LOGI(TAG, "opening %s", apps[sel].name);
        ui_wipe(); // the home screen would ghost through the app's first frame
        ui_close();
        btn_flush();
        bar_app = apps[sel].name;
        ui_keys(NULL, NULL, NULL, NULL);
        in_app = true;
        apps[sel].run();
        in_app = false;
        bar_app = "Home";
        ui_keys("< Left", "Right >", "Open", "Open");
        usb_host_lib_info_t usb;
        if (usb_host_lib_info(&usb) == ESP_OK) esp_restart(); // the app made USB a host: home must stay flashable
        if (!rng_run) wifi_off(), ranging_start(); // the app took the radio: ranging gets it back
        ui_wipe(); // and the app through the home screen
        btn_flush();
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init(); // Wi-Fi (ESP-NOW) keeps its calibration here
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    epd_init();
    btnq = xQueueCreate(8, sizeof(int));
    sao_init();
    int up, down;
    ESP_LOGI(TAG, "knob: %d (analog on SAO1 GPIO1: %d %d)", knob_angle(), (knob_analog(&up, &down), up), down);
    panel_lock = xSemaphoreCreateMutex();
    xTaskCreate(buttons, "buttons", 3072, NULL, 3, NULL);

    ui_wipe(); // the film still shows whatever ran before the reset; partial updates need a known start
    battery_pct(); // one reading before Wi-Fi starts sharing ADC2
    ranging_start(); // on every screen, so other badges' Radar sees this one; apps that take the radio stop it
    home();
}
