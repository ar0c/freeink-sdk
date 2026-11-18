/* SPDX-License-Identifier: LGPL-3.0-or-later */

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32S31

#include "lcd_driver.h"
#include "../output_common/rmt_compat.h"

#include <esp_cache.h>
#include <esp_check.h>
#include <esp_clk_tree.h>
#include <esp_heap_caps.h>
#include <esp_intr_alloc.h>
#include <esp_ipc.h>
#include <esp_private/gdma.h>
#include <esp_private/gpio.h>
#include <esp_private/periph_ctrl.h>
#include <esp_rom_gpio.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hal/axi_dma_ll.h>
#include <hal/dma_types.h>
#include <hal/lcd_hal.h>
#include <hal/lcd_ll.h>
#include <hal/lcd_periph.h>
#include <hal/rmt_ll.h>
#include <soc/rmt_struct.h>
#include <string.h>

enum {
    DMA_ALIGNMENT = 64,
    BOUNCE_LINES = 8,
    MAX_SCAN_LINES = 1000,
    CKV_TICK_HZ = 10000000,
    CKV_MIN_TICKS = CKV_TICK_HZ / 1000000,
    RMT_CKV = 0,
};

#define FRAME_EVENTS (LCD_LL_EVENT_VSYNC_END | LCD_LL_EVENT_TRANS_DONE)
static const char* const TAG = "epd_lcd_s31";

static struct {
    LcdEpdConfig_t cfg;
    lcd_hal_context_t hal;
    gdma_channel_handle_t dma;
    axi_dma_dev_t* dma_dev;
    int dma_channel;
    intr_handle_t lcd_intr;
    int irq_core;
    dma_descriptor_align8_t* descriptors;
    uint8_t* bounce[2];
    int line_bytes;
    int scan_lines;
    int lines_fed;
    uint16_t ckv_high_ticks;
    uint16_t ckv_period_ticks;
    int64_t last_phase_start_us;
    bool dma_connected;
    bool lcd_clock;
    bool rmt_clock;
    bool bus_claimed;
    bool scanning;
    bool frame_complete;
    bool frame_config_valid;
    line_cb_func_t get_line;
    frame_done_func_t frame_done;
    void* line_context;
    void* frame_context;
} scan;
static portMUX_TYPE start_lock = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR set_lcd_irq(bool enabled) {
    PERIPH_RCC_ATOMIC() {
        lcd_ll_enable_interrupt(scan.hal.dev, FRAME_EVENTS, enabled);
    }
}

static void IRAM_ATTR stop_scan(void) {
    axi_dma_ll_tx_enable_interrupt(scan.dma_dev, scan.dma_channel, GDMA_LL_EVENT_TX_EOF, false);
    set_lcd_irq(false);
    lcd_ll_stop(scan.hal.dev);
    if (scan.rmt_clock) rmt_ll_tx_stop(&RMT, RMT_CKV);
    gdma_stop(scan.dma);
    scan.scanning = false;
}

static void abort_on_irq_core(void* unused) {
    (void)unused;
    stop_scan();
    gdma_reset(scan.dma);
    axi_dma_ll_tx_clear_interrupt_status(scan.dma_dev, scan.dma_channel, UINT32_MAX);
}

void epd_lcd_abort_frame(void) {
    if (scan.dma_dev != NULL) {
        esp_ipc_call_blocking(scan.irq_core, abort_on_irq_core, NULL);
    }
}

static void IRAM_ATTR finish_scan(bool complete) {
    stop_scan();
    scan.frame_complete = complete;
    if (scan.frame_done != NULL) scan.frame_done(scan.frame_context);
}

static void IRAM_ATTR lcd_isr(void* unused) {
    (void)unused;
    uint32_t events = lcd_ll_get_interrupt_status(scan.hal.dev);
    lcd_ll_clear_interrupt_status(scan.hal.dev, events);
    if (scan.scanning && (events & LCD_LL_EVENT_VSYNC_END)) {
        finish_scan(scan.lines_fed == scan.scan_lines);
    }
}

static esp_err_t IRAM_ATTR fill_bounce(int index, bool* awoken) {
    for (int row = 0; row < BOUNCE_LINES; row++) {
        uint8_t* dst = scan.bounce[index] + row * scan.line_bytes;
        if (scan.lines_fed < scan.scan_lines && scan.get_line != NULL) {
            *awoken |= scan.get_line(scan.line_context, dst);
            scan.lines_fed++;
        } else {
            memset(dst, 0, scan.line_bytes);
        }
    }
    return esp_cache_msync(scan.bounce[index], scan.line_bytes * BOUNCE_LINES, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
}

static bool IRAM_ATTR dma_eof(
    gdma_channel_handle_t channel, gdma_event_data_t* event, void* unused
) {
    (void)channel;
    (void)unused;
    if (!scan.scanning) return false;
    const int index = event->tx_eof_desc_addr == (intptr_t)&scan.descriptors[0] ? 0 : 1;
    bool awoken = false;
    if (event->tx_eof_desc_addr != (intptr_t)&scan.descriptors[index] || fill_bounce(index, &awoken) != ESP_OK) {
        finish_scan(false);
    }
    return awoken;
}

static void register_dma_callback(void* result) {
    gdma_tx_event_callbacks_t callbacks = { .on_trans_eof = dma_eof };
    *(esp_err_t*)result = gdma_register_tx_event_callbacks(scan.dma, &callbacks, NULL);
}

static esp_err_t init_dma(void) {
    size_t size = scan.line_bytes * BOUNCE_LINES;
    uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA;
    scan.descriptors = heap_caps_aligned_calloc(DMA_ALIGNMENT, 2, sizeof(*scan.descriptors), caps);
    if (scan.descriptors == NULL) return ESP_ERR_NO_MEM;
    for (int i = 0; i < 2; i++) {
        scan.bounce[i] = heap_caps_aligned_alloc(DMA_ALIGNMENT, size, caps);
        if (scan.bounce[i] == NULL) return ESP_ERR_NO_MEM;
        scan.descriptors[i].dw0.suc_eof = 1;
        scan.descriptors[i].dw0.size = size;
        scan.descriptors[i].dw0.length = size;
        scan.descriptors[i].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
        scan.descriptors[i].buffer = scan.bounce[i];
        scan.descriptors[i].next = &scan.descriptors[1 - i];
    }

    gdma_channel_alloc_config_t channel = {
        .intr_priority = 3,
        .flags.isr_cache_safe = true,
    };
    ESP_RETURN_ON_ERROR(gdma_new_axi_channel(&channel, &scan.dma, NULL), TAG, "allocate LCD DMA");
    gdma_trigger_t trigger = GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0);
    ESP_RETURN_ON_ERROR(gdma_connect(scan.dma, trigger), TAG, "connect LCD DMA");
    scan.dma_connected = true;
    gdma_strategy_config_t strategy = {0};
    ESP_RETURN_ON_ERROR(gdma_apply_strategy(scan.dma, &strategy), TAG, "set LCD DMA strategy");
    gdma_transfer_config_t transfer = {
        .max_data_burst_size = DMA_ALIGNMENT,
        .access_ext_mem = false,
    };
    ESP_RETURN_ON_ERROR(gdma_config_transfer(scan.dma, &transfer), TAG, "configure LCD DMA");

    int group;
    ESP_RETURN_ON_ERROR(gdma_get_group_channel_id(scan.dma, &group, &scan.dma_channel), TAG, "identify LCD DMA");
    esp_err_t result = ESP_FAIL;
    ESP_RETURN_ON_ERROR(esp_ipc_call_blocking(scan.irq_core, register_dma_callback, &result), TAG, "bind DMA IRQ core");
    if (result != ESP_OK) return result;
    scan.dma_dev = AXI_DMA_LL_GET_HW(group - GDMA_LL_AXI_GROUP_START_ID);
    return ESP_OK;
}

static esp_err_t init_lcd(void) {
    const int module = soc_lcd_rgb_signals[0].module;
    PERIPH_RCC_ACQUIRE_ATOMIC(module, ref_count) {
        if (ref_count == 0) {
            lcd_ll_enable_bus_clock(0, true);
            lcd_ll_reset_register(0);
        }
    }
    lcd_hal_init(&scan.hal, 0);
    scan.lcd_clock = true;
    PERIPH_RCC_ATOMIC() {
        lcd_ll_enable_clock(scan.hal.dev, true);
        lcd_ll_select_clk_src(0, LCD_CLK_SRC_PLL160M);
    }
    lcd_ll_mem_set_low_power_mode(scan.hal.dev, LCD_LL_MEM_LP_MODE_SHUT_DOWN);
    lcd_ll_mem_power_by_pmu(scan.hal.dev);
    lcd_ll_reset(scan.hal.dev);
    lcd_ll_enable_trans_buffer(scan.hal.dev, true);
    lcd_ll_enable_rgb_mode(scan.hal.dev, true);
    lcd_ll_set_dma_read_stride(scan.hal.dev, 8);
    lcd_ll_set_data_wire_width(scan.hal.dev, 8);
    lcd_ll_set_phase_cycles(scan.hal.dev, 0, 0, 1);
    lcd_ll_enable_output_always_on(scan.hal.dev, true);
    lcd_ll_set_idle_level(scan.hal.dev, false, true, true);
    lcd_ll_set_clock_idle_level(scan.hal.dev, false);
    lcd_ll_set_pixel_clock_edge(scan.hal.dev, false);
    lcd_ll_reverse_wire_bit_order(scan.hal.dev, false);
    lcd_ll_set_blank_cycles(scan.hal.dev, 1, 1);
    lcd_ll_enable_output_hsync_in_porch_region(scan.hal.dev, false);
    lcd_ll_enable_auto_next_frame(scan.hal.dev, false);
    set_lcd_irq(false);
    lcd_ll_clear_interrupt_status(scan.hal.dev, FRAME_EVENTS);

    esp_err_t result = esp_intr_alloc_intrstatus(
        soc_lcd_rgb_signals[0].irq_id, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL2,
        (uint32_t)lcd_ll_get_interrupt_status_reg(scan.hal.dev), FRAME_EVENTS,
        lcd_isr, NULL, &scan.lcd_intr
    );
    if (result == ESP_OK) scan.irq_core = esp_intr_get_cpu(scan.lcd_intr);
    return result;
}

static esp_err_t output_signal(gpio_num_t pin, int signal) {
    ESP_RETURN_ON_ERROR(gpio_reset_pin(pin), TAG, "reset LCD GPIO");
    ESP_RETURN_ON_ERROR(gpio_set_direction(pin, GPIO_MODE_OUTPUT), TAG, "set LCD GPIO");
    esp_rom_gpio_connect_out_signal(pin, signal, false, false);
    return ESP_OK;
}

static esp_err_t init_bus(void) {
    scan.bus_claimed = true;
    const lcd_bus_config_t* bus = &scan.cfg.bus;
    for (int i = 0; i < 8; i++) {
        ESP_RETURN_ON_ERROR(
            gpio_iomux_output(bus->data[i], soc_lcd_rgb_iomux_descs[0].data_pins[i].func),
            TAG, "configure LCD data IOMUX"
        );
    }
    ESP_RETURN_ON_ERROR(output_signal(bus->leh, soc_lcd_rgb_signals[0].hsync_sig), TAG, "configure XLE");
    ESP_RETURN_ON_ERROR(output_signal(bus->clock, soc_lcd_rgb_signals[0].pclk_sig), TAG, "configure XCL");
    ESP_RETURN_ON_ERROR(output_signal(bus->start_pulse, soc_lcd_rgb_signals[0].de_sig), TAG, "configure XSTL");
    ESP_RETURN_ON_ERROR(gpio_set_level(bus->stv, 1), TAG, "park SPV high");
    return gpio_set_direction(bus->stv, GPIO_MODE_OUTPUT);
}

static esp_err_t init_ckv(void) {
    uint32_t source_hz;
    ESP_RETURN_ON_ERROR(esp_clk_tree_src_get_freq_hz(
        (soc_module_clk_t)RMT_CLK_SRC_DEFAULT, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &source_hz
    ), TAG, "read CKV source clock");
    if (source_hz % CKV_TICK_HZ != 0 || source_hz / CKV_TICK_HZ == 0 || source_hz / CKV_TICK_HZ > UINT8_MAX) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    rmt_compat_enable_clock(RMT_CKV);
    scan.rmt_clock = true;
    rmt_ll_mem_force_power_on(&RMT);
    rmt_compat_enable_mem_access_nonfifo(true);
    rmt_compat_set_group_clock_src(RMT_CKV);
    rmt_compat_set_clock_div(RMT_CKV, source_hz / CKV_TICK_HZ);
    rmt_compat_set_mem_blocks(RMT_CKV, 1);
    rmt_compat_tx_set_idle_level(RMT_CKV, 0, true);
    rmt_compat_tx_enable_carrier(RMT_CKV, false);
    rmt_compat_tx_enable_loop(RMT_CKV, true);
    rmt_compat_connect_gpio(RMT_CKV, scan.cfg.bus.ckv);
    return ESP_OK;
}

void epd_lcd_set_pixel_clock_MHz(int frequency) {
    scan.frame_config_valid = false;
    if (frequency <= 0 || frequency > 80 || frequency % 10 != 0) return;
    const uint32_t pixel_clock = (uint32_t)frequency * 1000000;
    const LcdLineTiming_t* line = &scan.cfg.line;
    if (line->le_high_time <= 0 || line->line_front_porch < 0 || line->line_end <= 0
        || line->le_high_time > 64 || line->line_front_porch > 64 || line->line_end > 64) return;
    const int clocks_per_tick = pixel_clock / CKV_TICK_HZ;
    const int data_clocks = line->le_high_time + line->line_front_porch + scan.line_bytes;
    const int high_ticks = (data_clocks + clocks_per_tick - 1) / clocks_per_tick;
    const int end_ticks = (data_clocks + line->line_end + clocks_per_tick - 1) / clocks_per_tick;
    scan.ckv_high_ticks = high_ticks;
    scan.ckv_period_ticks = end_ticks > high_ticks + CKV_MIN_TICKS ? end_ticks : high_ticks + CKV_MIN_TICKS;
    const int tail_clocks = scan.ckv_period_ticks * clocks_per_tick - data_clocks;

    hal_utils_clk_div_t divider = {0};
    if (lcd_hal_cal_pclk_freq(&scan.hal, 160000000, pixel_clock, &divider) != pixel_clock) return;
    PERIPH_RCC_ATOMIC() {
        lcd_ll_set_group_clock_coeff(0, divider.integer, divider.denominator, divider.numerator);
    }
    lcd_ll_set_horizontal_timing(
        scan.hal.dev, line->le_high_time, line->line_front_porch, scan.line_bytes, tail_clocks
    );
    lcd_ll_set_vertical_timing(scan.hal.dev, 1, 0, scan.scan_lines, 1);
    lcd_ll_set_hsync_position(scan.hal.dev, 1);
    rmt_compat_write_single_item(RMT_CKV, high_ticks, true, scan.ckv_period_ticks - high_ticks, false, true);
    scan.frame_config_valid = true;
}

void epd_lcd_set_line_timing(const LcdLineTiming_t* timing) {
    scan.cfg.line = *timing;
}

esp_err_t epd_lcd_init(const LcdEpdConfig_t* config, int width, int height) {
    if (scan.lcd_clock) return ESP_ERR_INVALID_STATE;
    if (config == NULL || config->bus_width != 8 || width <= 0 || width % 16 != 0
        || width / 4 * BOUNCE_LINES > DMA_DESCRIPTOR_BUFFER_MAX_SIZE
        || width / 4 * BOUNCE_LINES % DMA_ALIGNMENT != 0 || height <= 0 || height > MAX_SCAN_LINES
        || config->pixel_clock == 0 || config->pixel_clock % 1000000 != 0) return ESP_ERR_INVALID_ARG;
    for (int i = 0; i < 8; i++) {
        if (config->bus.data[i] != soc_lcd_rgb_iomux_descs[0].data_pins[i].gpio_num) return ESP_ERR_INVALID_ARG;
    }
    const gpio_num_t controls[] = {
        config->bus.clock, config->bus.ckv, config->bus.start_pulse, config->bus.leh, config->bus.stv,
    };
    uint64_t used = 0;
    for (int i = 0; i < 8; i++) used |= UINT64_C(1) << config->bus.data[i];
    for (unsigned i = 0; i < sizeof(controls) / sizeof(controls[0]); i++) {
        if (!GPIO_IS_VALID_OUTPUT_GPIO(controls[i]) || (used & (UINT64_C(1) << controls[i]))) {
            return ESP_ERR_INVALID_ARG;
        }
        used |= UINT64_C(1) << controls[i];
    }

    scan.cfg = *config;
    scan.line_bytes = width / 4;
    scan.scan_lines = height;
    esp_err_t ret;
    ESP_GOTO_ON_ERROR(init_lcd(), fail, TAG, "init RGB engine");
    ESP_GOTO_ON_ERROR(init_dma(), fail, TAG, "init DMA");
    ESP_GOTO_ON_ERROR(init_bus(), fail, TAG, "init pins");
    ESP_GOTO_ON_ERROR(init_ckv(), fail, TAG, "init gate clock");
    epd_lcd_set_pixel_clock_MHz(config->pixel_clock / 1000000);
    if (scan.frame_config_valid) return ESP_OK;
    ret = ESP_ERR_INVALID_ARG;
fail:
    epd_lcd_deinit();
    return ret;
}

void epd_lcd_deinit(void) {
    epd_lcd_abort_frame();
    if (scan.lcd_intr != NULL) esp_intr_free(scan.lcd_intr);
    if (scan.dma_connected) gdma_disconnect(scan.dma);
    if (scan.dma != NULL) gdma_del_channel(scan.dma);
    if (scan.lcd_clock) {
        lcd_ll_enable_trans_buffer(scan.hal.dev, false);
        PERIPH_RCC_ATOMIC() {
            lcd_ll_enable_clock(scan.hal.dev, false);
        }
        const int module = soc_lcd_rgb_signals[0].module;
        PERIPH_RCC_RELEASE_ATOMIC(module, ref_count) {
            if (ref_count == 0) lcd_ll_enable_bus_clock(0, false);
        }
    }
    if (scan.rmt_clock) {
        rmt_compat_disable_clock(RMT_CKV);
        gpio_reset_pin(scan.cfg.bus.ckv);
    }
    if (scan.bus_claimed) {
        for (int i = 0; i < 8; i++) gpio_reset_pin(scan.cfg.bus.data[i]);
        gpio_reset_pin(scan.cfg.bus.stv);
        gpio_reset_pin(scan.cfg.bus.clock);
        gpio_reset_pin(scan.cfg.bus.leh);
        gpio_reset_pin(scan.cfg.bus.start_pulse);
    }
    heap_caps_free(scan.descriptors);
    heap_caps_free(scan.bounce[0]);
    heap_caps_free(scan.bounce[1]);
    memset(&scan, 0, sizeof(scan));
}

void IRAM_ATTR epd_lcd_line_source_cb(line_cb_func_t get_line, void* payload) {
    scan.get_line = get_line;
    scan.line_context = payload;
}

void IRAM_ATTR epd_lcd_frame_done_cb(frame_done_func_t done, void* payload) {
    scan.frame_done = done;
    scan.frame_context = payload;
}

static void start_on_irq_core(void* unused) {
    (void)unused;
    abort_on_irq_core(NULL);
    lcd_ll_fifo_reset(scan.hal.dev);
    scan.lines_fed = 0;
    bool awoken = false;
    if (fill_bounce(0, &awoken) != ESP_OK || fill_bounce(1, &awoken) != ESP_OK) return;
    if (esp_cache_msync(scan.descriptors, 2 * sizeof(*scan.descriptors),
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED) != ESP_OK) return;
    scan.scanning = true;
    axi_dma_ll_tx_enable_interrupt(scan.dma_dev, scan.dma_channel, GDMA_LL_EVENT_TX_EOF, true);
    if (gdma_start(scan.dma, (intptr_t)scan.descriptors) != ESP_OK) {
        stop_scan();
        return;
    }

    taskENTER_CRITICAL(&start_lock);
    scan.last_phase_start_us = esp_timer_get_time();
    gpio_set_level(scan.cfg.bus.stv, 0);
    rmt_compat_tx_configure_finite_loop(RMT_CKV, scan.scan_lines + 5);
    rmt_compat_tx_start(RMT_CKV);
    esp_rom_delay_us((scan.ckv_period_ticks + CKV_MIN_TICKS - 1) / CKV_MIN_TICKS);
    gpio_set_level(scan.cfg.bus.stv, 1);
    esp_rom_delay_us(scan.ckv_high_ticks / CKV_MIN_TICKS);
    lcd_ll_clear_interrupt_status(scan.hal.dev, FRAME_EVENTS);
    set_lcd_irq(true);
    lcd_ll_start(scan.hal.dev);
    taskEXIT_CRITICAL(&start_lock);
}

void epd_lcd_start_frame(void) {
    scan.frame_complete = false;
    if (!scan.frame_config_valid) return;

    const int64_t remaining_us = scan.last_phase_start_us + scan.cfg.min_phase_interval_us - esp_timer_get_time();
    if (scan.last_phase_start_us != 0 && remaining_us > 0) esp_rom_delay_us((uint32_t)remaining_us);
    esp_ipc_call_blocking(scan.irq_core, start_on_irq_core, NULL);
}

bool epd_lcd_frame_complete(void) {
    return scan.frame_complete;
}

#endif  // CONFIG_IDF_TARGET_ESP32S31
