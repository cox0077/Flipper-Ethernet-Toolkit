/*
 * Flipper Ethernet Toolkit v0.1
 * W5500 over Flipper Zero external SPI (3.3V only)
 * SPDX-License-Identifier: MIT
 *
 * Read-only diagnostics: no DHCP, ping, scanning or packet transmission.
 */

#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_spi.h>
#include <furi_hal_spi_config.h>
#include <furi_hal_resources.h>
#include <gui/gui.h>
#include <input/input.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define W5500_REG_GAR      0x0001U
#define W5500_REG_SUBR     0x0005U
#define W5500_REG_SHAR     0x0009U
#define W5500_REG_SIPR     0x000FU
#define W5500_REG_PHYCFGR  0x002EU
#define W5500_REG_VERSIONR 0x0039U

#define W5500_SPI_TIMEOUT_MS 100U
#define PAGE_COUNT 3U

typedef struct {
    bool spi_ok;
    bool detected;
    uint8_t version;
    uint8_t phy;
    uint8_t mac[6];
    uint8_t ip[4];
    uint8_t mask[4];
    uint8_t gateway[4];
} W5500Snapshot;

typedef struct {
    FuriMutex* lock;
    W5500Snapshot snapshot;
    uint8_t page;
} EthernetApp;

/*
 * W5500 SPI variable data mode, common-register block:
 * [address high][address low][control=0x00 read][dummy]
 * The external SPI handle asserts CS during acquire and deasserts on release.
 */
static bool w5500_read(uint16_t addr, uint8_t* value) {
    const FuriHalSpiBusHandle* handle = &furi_hal_spi_bus_handle_external;
    const uint8_t tx[4] = {(uint8_t)(addr >> 8), (uint8_t)addr, 0x00, 0x00};
    uint8_t rx[4] = {0};

    furi_hal_spi_acquire(handle);
    const bool ok = furi_hal_spi_bus_trx(handle, tx, rx, sizeof(tx), W5500_SPI_TIMEOUT_MS);
    furi_hal_spi_release(handle);

    if(ok) *value = rx[3];
    return ok;
}

static bool w5500_read_bytes(uint16_t start, uint8_t* out, size_t size) {
    for(size_t i = 0; i < size; ++i) {
        if(!w5500_read(start + (uint16_t)i, &out[i])) return false;
    }
    return true;
}

static W5500Snapshot w5500_probe(void) {
    W5500Snapshot result = {0};
    if(!w5500_read(W5500_REG_VERSIONR, &result.version)) return result;
    result.spi_ok = true;
    if(result.version != 0x04) return result;
    result.detected = true;

    bool ok = w5500_read(W5500_REG_PHYCFGR, &result.phy);
    ok = ok && w5500_read_bytes(W5500_REG_SHAR, result.mac, sizeof(result.mac));
    ok = ok && w5500_read_bytes(W5500_REG_SIPR, result.ip, sizeof(result.ip));
    ok = ok && w5500_read_bytes(W5500_REG_SUBR, result.mask, sizeof(result.mask));
    ok = ok && w5500_read_bytes(W5500_REG_GAR, result.gateway, sizeof(result.gateway));
    result.spi_ok = ok;
    if(!ok) result.detected = false;
    return result;
}

static void app_refresh(EthernetApp* app) {
    W5500Snapshot new_snapshot = w5500_probe();
    furi_check(furi_mutex_acquire(app->lock, FuriWaitForever) == FuriStatusOk);
    app->snapshot = new_snapshot;
    furi_mutex_release(app->lock);
}

static void app_draw(Canvas* canvas, void* context) {
    EthernetApp* app = context;
    W5500Snapshot s;
    uint8_t page;
    char line[50];

    furi_check(furi_mutex_acquire(app->lock, FuriWaitForever) == FuriStatusOk);
    s = app->snapshot;
    page = app->page;
    furi_mutex_release(app->lock);

    canvas_clear(canvas);
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 2, 11, "Ethernet Toolkit");
    canvas_draw_line(canvas, 0, 14, 127, 14);
    canvas_set_font(canvas, FontSecondary);

    if(page == 0) {
        if(s.detected) {
            canvas_draw_str(canvas, 2, 26, "W5500: found (v04)");
            snprintf(line, sizeof(line), "Cable: %s", (s.phy & 0x01) ? "CONNECTED" : "DISCONNECTED");
            canvas_draw_str(canvas, 2, 37, line);
            if(s.phy & 0x01) {
                snprintf(line, sizeof(line), "PHY: %u Mbps, %s", (s.phy & 0x02) ? 100 : 10,
                         (s.phy & 0x04) ? "Full" : "Half");
                canvas_draw_str(canvas, 2, 48, line);
            } else {
                canvas_draw_str(canvas, 2, 48, "Connect an RJ45 cable");
            }
        } else {
            canvas_draw_str(canvas, 2, 26, "W5500: NOT FOUND");
            snprintf(line, sizeof(line), "Version read: 0x%02X", s.version);
            canvas_draw_str(canvas, 2, 37, line);
            canvas_draw_str(canvas, 2, 48, s.spi_ok ? "Check wiring / power" : "SPI transfer error");
        }
    } else if(page == 1) {
        if(!s.detected) {
            canvas_draw_str(canvas, 2, 28, "No W5500 detected");
            canvas_draw_str(canvas, 2, 43, "See wiring on page 3");
        } else {
            snprintf(line, sizeof(line), "MAC %02X:%02X:%02X:%02X:%02X:%02X",
                     s.mac[0], s.mac[1], s.mac[2], s.mac[3], s.mac[4], s.mac[5]);
            canvas_draw_str(canvas, 2, 22, line);
            snprintf(line, sizeof(line), "IP %u.%u.%u.%u", s.ip[0], s.ip[1], s.ip[2], s.ip[3]);
            canvas_draw_str(canvas, 2, 32, line);
            snprintf(line, sizeof(line), "SN %u.%u.%u.%u", s.mask[0], s.mask[1], s.mask[2], s.mask[3]);
            canvas_draw_str(canvas, 2, 42, line);
            snprintf(line, sizeof(line), "GW %u.%u.%u.%u", s.gateway[0], s.gateway[1], s.gateway[2], s.gateway[3]);
            canvas_draw_str(canvas, 2, 52, line);
        }
    } else {
        canvas_draw_str(canvas, 2, 24, "MOSI PA7-2   MISO PA6-3");
        canvas_draw_str(canvas, 2, 35, "SCK PB3-5    CS PA4-4");
        canvas_draw_str(canvas, 2, 46, "RST PC3-7    VCC 3V3-9");
        canvas_draw_str(canvas, 2, 54, "GND: pin 8 or 11");
    }

    if(page == 0) {
        canvas_draw_str(canvas, 2, 63, "</> page  OK refresh");
    } else {
        snprintf(line, sizeof(line), "< / > page   %u/%u", (unsigned)(page + 1), PAGE_COUNT);
        canvas_draw_str(canvas, 2, 63, line);
    }
}

static void app_input(InputEvent* input, void* context) {
    FuriMessageQueue* queue = context;
    (void)furi_message_queue_put(queue, input, 0);
}

int32_t flipper_eth_toolkit_app(void* p) {
    UNUSED(p);
    EthernetApp app = {0};
    app.lock = furi_mutex_alloc(FuriMutexTypeNormal);
    furi_check(app.lock);

    FuriMessageQueue* queue = furi_message_queue_alloc(8, sizeof(InputEvent));
    furi_check(queue);

    // Keep the W5500 reset input high; no reset or IP settings are changed.
    furi_hal_gpio_write(&gpio_ext_pc3, true);
    furi_hal_gpio_init(&gpio_ext_pc3, GpioModeOutputPushPull, GpioPullNo, GpioSpeedLow);
    furi_hal_spi_bus_handle_init(&furi_hal_spi_bus_handle_external);
    furi_delay_ms(100);

    app_refresh(&app);

    ViewPort* view_port = view_port_alloc();
    view_port_draw_callback_set(view_port, app_draw, &app);
    view_port_input_callback_set(view_port, app_input, queue);

    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, view_port, GuiLayerFullscreen);

    bool running = true;
    InputEvent input;
    while(running) {
        const FuriStatus status = furi_message_queue_get(queue, &input, 1500);
        bool refresh = (status != FuriStatusOk);
        if(status == FuriStatusOk && input.type == InputTypeShort) {
            if(input.key == InputKeyBack) {
                running = false;
            } else if(input.key == InputKeyRight || input.key == InputKeyDown) {
                furi_mutex_acquire(app.lock, FuriWaitForever);
                app.page = (app.page + 1U) % PAGE_COUNT;
                furi_mutex_release(app.lock);
            } else if(input.key == InputKeyLeft || input.key == InputKeyUp) {
                furi_mutex_acquire(app.lock, FuriWaitForever);
                app.page = (app.page + PAGE_COUNT - 1U) % PAGE_COUNT;
                furi_mutex_release(app.lock);
            } else if(input.key == InputKeyOk) {
                refresh = true;
            }
        }
        if(running) {
            if(refresh) app_refresh(&app);
            view_port_update(view_port);
        }
    }

    gui_remove_view_port(gui, view_port);
    view_port_free(view_port);
    furi_record_close(RECORD_GUI);
    furi_hal_spi_bus_handle_deinit(&furi_hal_spi_bus_handle_external);
    furi_hal_gpio_write(&gpio_ext_pc3, true);
    furi_hal_gpio_init(&gpio_ext_pc3, GpioModeAnalog, GpioPullNo, GpioSpeedLow);
    furi_message_queue_free(queue);
    furi_mutex_free(app.lock);
    return 0;
}
