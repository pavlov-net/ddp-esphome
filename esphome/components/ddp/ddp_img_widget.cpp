// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// LVGL DDP Image widget implementation

// Only compile if LVGL component is available
#ifdef USE_LVGL

#include "ddp_img_widget.h"
#include "ddp_img_widget_impl.h"
#include "ddp.h"
#include "ddp_pixel_convert.h"
#include "esphome/core/log.h"
#include "esp_timer.h"
#include <cstring>
#include <algorithm>

namespace esphome {
namespace ddp {

static const char* TAG = "ddp.img";

// LVGL assertions
static_assert(LV_COLOR_DEPTH == 16 || LV_COLOR_DEPTH == 32, "LV_COLOR_DEPTH must be 16 or 32");
static constexpr size_t BYTES_PER_PIXEL = LV_COLOR_DEPTH / 8;

}  // namespace ddp
}  // namespace esphome

// ========================================================================
// C LINKAGE SECTION (LVGL interface)
// ========================================================================

extern "C" {

/**********************
 * STATIC PROTOTYPES
 **********************/
static void lv_ddpimg_constructor(const lv_obj_class_t* class_p, lv_obj_t* obj);
static void lv_ddpimg_destructor(const lv_obj_class_t* class_p, lv_obj_t* obj);
static void lv_ddpimg_event(const lv_obj_class_t* class_p, lv_event_t* e);

/**********************
 * CLASS DESCRIPTOR
 **********************/

/**
 * DDP Image widget class descriptor (LVGL 8.4 pattern)
 * This MUST be a const global with C linkage
 */
const lv_obj_class_t lv_ddpimg_class = {
    .base_class = &lv_obj_class,
    .constructor_cb = lv_ddpimg_constructor,
    .destructor_cb = lv_ddpimg_destructor,
#if LV_USE_USER_DATA
    .user_data = nullptr,
#endif
    .event_cb = lv_ddpimg_event,
    .width_def = LV_SIZE_CONTENT,
    .height_def = LV_SIZE_CONTENT,
    .editable = LV_OBJ_CLASS_EDITABLE_FALSE,
    .group_def = LV_OBJ_CLASS_GROUP_DEF_FALSE,
    .instance_size = sizeof(lv_ddpimg_t),
};

/**********************
 * WIDGET LIFECYCLE
 **********************/

/**
 * Constructor (called when widget created)
 */
static void lv_ddpimg_constructor(const lv_obj_class_t* class_p, lv_obj_t* obj) {
    LV_UNUSED(class_p);
    LV_TRACE_OBJ_CREATE("begin");

    lv_ddpimg_t* ddpimg = (lv_ddpimg_t*)obj;

    // Initialize C struct members
    ddpimg->ddp_parent = nullptr;
    ddpimg->stream_id = 0;
    ddpimg->img_w = -1;
    ddpimg->img_h = -1;
    ddpimg->back_buffers = 2;  // Default: triple-buffer

    // Clear image descriptor
    lv_memset_00(&ddpimg->img_dsc, sizeof(lv_img_dsc_t));

    // Create C++ implementation object
    ddpimg->impl_ptr = new esphome::ddp::DdpImgWidgetImpl(ddpimg);

    // Widget flags (like lv_img)
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(obj, LV_OBJ_FLAG_ADV_HITTEST);

    LV_TRACE_OBJ_CREATE("finished");
}

/**
 * Destructor (called when widget deleted)
 */
static void lv_ddpimg_destructor(const lv_obj_class_t* class_p, lv_obj_t* obj) {
    LV_UNUSED(class_p);

    lv_ddpimg_t* ddpimg = (lv_ddpimg_t*)obj;

    // Delete C++ implementation object
    if (ddpimg->impl_ptr) {
        auto* impl = static_cast<esphome::ddp::DdpImgWidgetImpl*>(ddpimg->impl_ptr);
        delete impl;  // Destructor calls free_buffers()
        ddpimg->impl_ptr = nullptr;
    }
}

/**
 * Event handler (LVGL events)
 */
static void lv_ddpimg_event(const lv_obj_class_t* class_p, lv_event_t* e) {
    LV_UNUSED(class_p);

    lv_event_code_t code = lv_event_get_code(e);

    // Call base class for non-draw events
    if (code != LV_EVENT_DRAW_MAIN && code != LV_EVENT_DRAW_POST) {
        lv_res_t res = lv_obj_event_base(&lv_ddpimg_class, e);
        if (res != LV_RES_OK) return;
    }

    lv_obj_t* obj = lv_event_get_target(e);
    lv_ddpimg_t* ddpimg = (lv_ddpimg_t*)obj;

    if (!ddpimg->impl_ptr) return;
    auto* impl = static_cast<esphome::ddp::DdpImgWidgetImpl*>(ddpimg->impl_ptr);

    switch (code) {
        case LV_EVENT_DRAW_MAIN:
            // Swap buffers and update image descriptor
            impl->present_if_ready();

            // Draw image from descriptor
            if (ddpimg->img_dsc.data && ddpimg->img_dsc.header.w > 0 && ddpimg->img_dsc.header.h > 0) {
                lv_draw_ctx_t* draw_ctx = lv_event_get_draw_ctx(e);
                lv_draw_img_dsc_t img_draw_dsc;
                lv_draw_img_dsc_init(&img_draw_dsc);
                lv_obj_init_draw_img_dsc(obj, LV_PART_MAIN, &img_draw_dsc);

                lv_draw_img(draw_ctx, &img_draw_dsc, &obj->coords, &ddpimg->img_dsc);
            }
            break;

        case LV_EVENT_SIZE_CHANGED:
            // Handle dynamic size changes (if dimensions were auto-detected)
            impl->handle_size_change();
            break;

        case LV_EVENT_GET_SELF_SIZE: {
            // Report widget's natural size (for SIZE_CONTENT)
            lv_point_t* p = (lv_point_t*)lv_event_get_param(e);
            if (ddpimg->img_w > 0 && ddpimg->img_h > 0) {
                p->x = ddpimg->img_w;
                p->y = ddpimg->img_h;
            } else {
                p->x = ddpimg->img_dsc.header.w;
                p->y = ddpimg->img_dsc.header.h;
            }
            break;
        }

        default:
            break;
    }
}

/**********************
 * PUBLIC API
 **********************/

/**
 * Create widget (standard LVGL pattern)
 */
lv_obj_t* lv_ddpimg_create(lv_obj_t* parent) {
    LV_LOG_INFO("begin");
    lv_obj_t* obj = lv_obj_class_create_obj(&lv_ddpimg_class, parent);
    lv_obj_class_init_obj(obj);
    return obj;
}

/**
 * Configuration functions (called from Python codegen)
 */
void lv_ddpimg_set_ddp_parent(lv_obj_t* obj, DdpComponent* parent) {
    LV_ASSERT_OBJ(obj, &lv_ddpimg_class);
    lv_ddpimg_t* ddpimg = (lv_ddpimg_t*)obj;
    ddpimg->ddp_parent = parent;
}

void lv_ddpimg_set_stream_id(lv_obj_t* obj, uint8_t stream_id) {
    LV_ASSERT_OBJ(obj, &lv_ddpimg_class);
    lv_ddpimg_t* ddpimg = (lv_ddpimg_t*)obj;
    ddpimg->stream_id = stream_id;
}

void lv_ddpimg_set_size(lv_obj_t* obj, int16_t w, int16_t h) {
    LV_ASSERT_OBJ(obj, &lv_ddpimg_class);
    lv_ddpimg_t* ddpimg = (lv_ddpimg_t*)obj;
    ddpimg->img_w = w;
    ddpimg->img_h = h;
}

void lv_ddpimg_set_back_buffers(lv_obj_t* obj, uint8_t count) {
    LV_ASSERT_OBJ(obj, &lv_ddpimg_class);
    lv_ddpimg_t* ddpimg = (lv_ddpimg_t*)obj;
    ddpimg->back_buffers = (count > 2) ? 2 : count;
}

void lv_ddpimg_init(lv_obj_t* obj) {
    LV_ASSERT_OBJ(obj, &lv_ddpimg_class);
    lv_ddpimg_t* ddpimg = (lv_ddpimg_t*)obj;

    auto* impl = static_cast<esphome::ddp::DdpImgWidgetImpl*>(ddpimg->impl_ptr);
    if (!impl) return;

    // Allocate buffers
    impl->allocate_buffers();

    // Register with DDP component
    if (ddpimg->ddp_parent) {
        ddpimg->ddp_parent->register_renderer(impl);
    }
}

}  // extern "C"

// ========================================================================
// C++ IMPLEMENTATION SECTION
// ========================================================================

namespace esphome {
namespace ddp {

// -------- DdpRenderer interface (UDP TASK CONTEXT) --------

void DdpImgWidgetImpl::on_data(size_t offset_px, const uint8_t* pixels,
                                PixelFormat format, size_t pixel_count) {
    // UDP TASK CONTEXT - no LVGL or ESPHome APIs allowed!

    // Check if buffers are allocated
    if (buf_px_ == 0) {
        return;
    }

    // Bounds check
    if (offset_px + pixel_count > buf_px_) {
        return;  // Silently drop out-of-bounds data
    }

#ifdef DDP_METRICS
    // Track frame assembly (for coverage calculation)
    if (frame_bytes_accum_.load(std::memory_order_relaxed) == 0) {
        // First packet in frame
        frame_first_pkt_us_.store(esp_timer_get_time(), std::memory_order_relaxed);
    }

    // Accumulate bytes received (depends on format)
    size_t bytes = 0;
    if (format == PixelFormat::RGB888) {
        bytes = pixel_count * 3;
    } else if (format == PixelFormat::RGB565_BE || format == PixelFormat::RGB565_LE) {
        bytes = pixel_count * 2;
    } else if (format == PixelFormat::RGBW) {
        bytes = pixel_count * 4;
    }
    frame_bytes_accum_.fetch_add(bytes, std::memory_order_relaxed);
#endif

    // Select destination buffer based on back buffer mode
    uint16_t* dst_buf = nullptr;
    if (widget_->back_buffers == 0) {
        // No buffering: write directly to front
        dst_buf = front_buf_;
    } else {
        // Buffered: write to accum
        dst_buf = accum_buf_;
    }

    if (!dst_buf) return;

    // Convert and write pixels to buffer using shared helpers
#if LV_COLOR_DEPTH == 16
    uint16_t* dst = dst_buf + offset_px;

    // Determine byte swap requirement once (used by RGB888 and RGBW conversions)
    constexpr bool swap_bytes =
    #if defined(LV_COLOR_16_SWAP) && LV_COLOR_16_SWAP
        true;
    #else
        false;
    #endif

    if (format == PixelFormat::RGB888) {
        // RGB888 → RGB565 (with optional byte swap)
        convert_rgb888_to_rgb565(dst, pixels, pixel_count, swap_bytes);

    } else if (format == PixelFormat::RGB565_BE || format == PixelFormat::RGB565_LE) {
        // RGB565 → RGB565 (byte swap if endianness mismatch)
        const bool src_be = (format == PixelFormat::RGB565_BE);

        if (src_be == swap_bytes) {
            // Direct copy
            std::memcpy(dst, pixels, pixel_count * 2);
        } else {
            // Copy + byte swap
            std::memcpy(dst, pixels, pixel_count * 2);
            swap_rgb565_bytes(dst, pixel_count);
        }

    } else if (format == PixelFormat::RGBW) {
        // RGBW → RGB565 (drop W channel, with optional byte swap)
        convert_rgbw_to_rgb565(dst, pixels, pixel_count, swap_bytes);
    }

#elif LV_COLOR_DEPTH == 32
    uint32_t* dst32 = reinterpret_cast<uint32_t*>(dst_buf) + offset_px;

    if (format == PixelFormat::RGB888) {
        // RGB888 → RGB32 (RGBA8888)
        convert_rgb888_to_rgb32(dst32, pixels, pixel_count);

    } else if (format == PixelFormat::RGB565_BE || format == PixelFormat::RGB565_LE) {
        // RGB565 → RGB32 (manual expansion)
        const bool src_be = (format == PixelFormat::RGB565_BE);
        const uint8_t* sp = pixels;

        for (size_t i = 0; i < pixel_count; ++i) {
            uint16_t v = src_be ? (uint16_t)((sp[0] << 8) | sp[1])
                                : (uint16_t)((sp[1] << 8) | sp[0]);
            uint8_t r5 = (uint8_t)((v >> 11) & 0x1F);
            uint8_t g6 = (uint8_t)((v >> 5)  & 0x3F);
            uint8_t b5 = (uint8_t)( v        & 0x1F);

            uint32_t c = ((r5 << 3) | (r5 >> 2)) |        // R
                         (((g6 << 2) | (g6 >> 4)) << 8) | // G
                         (((b5 << 3) | (b5 >> 2)) << 16) | // B
                         (0xFF << 24);                     // A
            dst32[i] = c;
            sp += 2;
        }

    } else if (format == PixelFormat::RGBW) {
        // RGBW → RGB32 (drop W channel, use 0xFF for alpha)
        convert_rgbw_to_rgb32(dst32, pixels, pixel_count);
    }
#endif
}

void DdpImgWidgetImpl::on_push() {
    // UDP TASK CONTEXT - no LVGL or ESPHome APIs allowed!

#ifdef DDP_METRICS
    // Calculate coverage (pure math, safe in UDP task)
    size_t bytes_received = frame_bytes_accum_.load(std::memory_order_relaxed);
    size_t expected_bytes = buf_px_ * BYTES_PER_PIXEL;
    double coverage = (expected_bytes > 0) ? std::min(1.0, (double)bytes_received / (double)expected_bytes) : 0.0;

    // Update coverage EWMA
    constexpr double ALPHA = 0.2;
    double prev_cov = metrics_.coverage_ewma;
    metrics_.coverage_ewma = (prev_cov == 0.0) ? coverage : (ALPHA * coverage + (1.0 - ALPHA) * prev_cov);

    // Mark timestamp when frame became ready
    ready_set_us_.store(esp_timer_get_time(), std::memory_order_relaxed);

    // Reset frame assembly tracking for next frame
    frame_bytes_accum_.store(0, std::memory_order_relaxed);
#endif

    // Swap buffers and set atomic flags based on buffer mode
    if (widget_->back_buffers == 2) {
        // Triple-buffer: swap accum↔ready
        std::swap(ready_buf_, accum_buf_);
        have_ready_.store(true, std::memory_order_release);
        need_invalidate_.store(true, std::memory_order_relaxed);  // Trigger draw cycle
    } else if (widget_->back_buffers == 1) {
        // Double-buffer: signal copy needed
        need_copy_to_front_.store(true, std::memory_order_relaxed);
        need_invalidate_.store(true, std::memory_order_relaxed);
    } else {
        // No buffer mode (0): just need to invalidate
        need_invalidate_.store(true, std::memory_order_relaxed);
    }

    // NOTE: We cannot call lv_obj_invalidate() from UDP task - causes thread safety warnings
    // Instead, we set need_invalidate_ flag which is checked in loop() on main thread
}

uint8_t DdpImgWidgetImpl::get_stream_id() const {
    return widget_->stream_id;
}

bool DdpImgWidgetImpl::get_dimensions(int* w, int* h) const {
    if (widget_->img_w <= 0 || widget_->img_h <= 0) return false;
    if (w) *w = widget_->img_w;
    if (h) *h = widget_->img_h;
    return true;
}

void DdpImgWidgetImpl::loop() {
    // Called from DdpComponent::loop() on main thread
    // Check if invalidation is needed (safe - we're on main thread)
    if (need_invalidate_.exchange(false, std::memory_order_relaxed)) {
        lv_obj_invalidate(&widget_->obj);
    }
}

#ifdef DDP_METRICS
void DdpImgWidgetImpl::reset_windowed_metrics() {
    metrics_.win_frames_presented = 0;
}
#endif

// -------- Buffer management (MAIN THREAD CONTEXT) --------

void DdpImgWidgetImpl::allocate_buffers() {
    // Auto-detect dimensions from widget size if not specified
    int w = widget_->img_w;
    int h = widget_->img_h;

    if (w <= 0 || h <= 0) {
        w = (int)lv_obj_get_width(&widget_->obj);
        h = (int)lv_obj_get_height(&widget_->obj);

        if (widget_->img_w <= 0) widget_->img_w = w;
        if (widget_->img_h <= 0) widget_->img_h = h;
    }

    if (w <= 0 || h <= 0) {
        ESP_LOGW(TAG, "Cannot allocate buffers: invalid dimensions %dx%d", w, h);
        return;
    }

    size_t px = (size_t)w * (size_t)h;
    size_t bytes = px * BYTES_PER_PIXEL;

    // Free existing buffers if size changed
    if (buf_px_ != px) {
        free_buffers();
        buf_px_ = px;
    }

    // Allocate front buffer
    if (!front_buf_) {
        front_buf_ = static_cast<uint16_t*>(lv_mem_alloc(bytes));
        if (!front_buf_) {
            ESP_LOGE(TAG, "Failed to allocate front buffer (%u bytes)", (unsigned)bytes);
            free_buffers();
            return;
        }
        std::memset(front_buf_, 0, bytes);
    }

    // Allocate ready buffer (triple-buffer mode)
    if (widget_->back_buffers == 2 && !ready_buf_) {
        ready_buf_ = static_cast<uint16_t*>(lv_mem_alloc(bytes));
        if (!ready_buf_) {
            ESP_LOGE(TAG, "Failed to allocate ready buffer (%u bytes)", (unsigned)bytes);
            free_buffers();
            return;
        }
        std::memset(ready_buf_, 0, bytes);
    }

    // Allocate accum buffer (double or triple-buffer mode)
    if (widget_->back_buffers >= 1 && !accum_buf_) {
        accum_buf_ = static_cast<uint16_t*>(lv_mem_alloc(bytes));
        if (!accum_buf_) {
            ESP_LOGE(TAG, "Failed to allocate accum buffer (%u bytes)", (unsigned)bytes);
            free_buffers();
            return;
        }
        std::memset(accum_buf_, 0, bytes);
    }

    // Set up image descriptor to point to front buffer
    widget_->img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    widget_->img_dsc.header.w = w;
    widget_->img_dsc.header.h = h;
    widget_->img_dsc.data = (const uint8_t*)front_buf_;
    widget_->img_dsc.data_size = bytes;

    ESP_LOGI(TAG, "Stream %u: Allocated buffers %dx%d (%u bytes, mode=%u)",
             widget_->stream_id, w, h, (unsigned)bytes, widget_->back_buffers);
}

void DdpImgWidgetImpl::free_buffers() {
    if (front_buf_) {
        lv_mem_free(front_buf_);
        front_buf_ = nullptr;
    }
    if (ready_buf_) {
        lv_mem_free(ready_buf_);
        ready_buf_ = nullptr;
    }
    if (accum_buf_) {
        lv_mem_free(accum_buf_);
        accum_buf_ = nullptr;
    }

    have_ready_.store(false);
    need_copy_to_front_.store(false);
    buf_px_ = 0;

    // Clear image descriptor
    widget_->img_dsc.data = nullptr;
    widget_->img_dsc.data_size = 0;
    widget_->img_dsc.header.w = 0;
    widget_->img_dsc.header.h = 0;
}

// -------- LVGL event callbacks (MAIN THREAD CONTEXT) --------

void DdpImgWidgetImpl::present_if_ready() {
    // Called from LV_EVENT_DRAW_MAIN (main thread)

    // Triple-buffer path: copy ready→front
    if (widget_->back_buffers == 2) {
        if (have_ready_.exchange(false, std::memory_order_acquire)) {
            if (front_buf_ && ready_buf_ && buf_px_ > 0) {
                std::memcpy(front_buf_, ready_buf_, buf_px_ * BYTES_PER_PIXEL);

#ifdef DDP_METRICS
                // Update presentation metrics
                int64_t now_us = esp_timer_get_time();
                metrics_.frames_presented++;
                metrics_.win_frames_presented++;

                // Calculate present latency (first packet → presentation)
                int64_t first_pkt = frame_first_pkt_us_.load(std::memory_order_relaxed);
                if (first_pkt > 0) {
                    double lat_us = (double)(now_us - first_pkt);
                    constexpr double ALPHA = 0.2;
                    double prev = metrics_.present_lat_us_ewma;
                    metrics_.present_lat_us_ewma = (prev == 0.0) ? lat_us : (ALPHA * lat_us + (1.0 - ALPHA) * prev);
                }

                // Calculate queue wait time (ready set → presentation)
                int64_t ready_time = ready_set_us_.load(std::memory_order_relaxed);
                if (ready_time > 0) {
                    double wait_ms = (double)(now_us - ready_time) / 1000.0;
                    constexpr double ALPHA = 0.2;
                    double prev = metrics_.queue_wait_ms_ewma;
                    metrics_.queue_wait_ms_ewma = (prev == 0.0) ? wait_ms : (ALPHA * wait_ms + (1.0 - ALPHA) * prev);
                }
#endif
            }
        }
    }

    // Double-buffer path: copy accum→front
    if (widget_->back_buffers == 1 && need_copy_to_front_.exchange(false)) {
        if (front_buf_ && accum_buf_ && buf_px_) {
            std::memcpy(front_buf_, accum_buf_, buf_px_ * BYTES_PER_PIXEL);

#ifdef DDP_METRICS
            metrics_.frames_presented++;
            metrics_.win_frames_presented++;
#endif
        }
    }

    // No buffer mode (0): data already in front_buf_
    // NOTE: Invalidation is now handled by the timer callback checking need_invalidate_ flag
}

void DdpImgWidgetImpl::handle_size_change() {
    // Only handle if dimensions were auto-detected
    bool auto_any = (widget_->img_w <= 0 || widget_->img_h <= 0);
    if (!auto_any) return;

    int W = (int)lv_obj_get_width(&widget_->obj);
    int H = (int)lv_obj_get_height(&widget_->obj);

    bool changed = false;
    if (widget_->img_w <= 0 && W != widget_->img_dsc.header.w) {
        widget_->img_w = W;
        changed = true;
    }
    if (widget_->img_h <= 0 && H != widget_->img_dsc.header.h) {
        widget_->img_h = H;
        changed = true;
    }

    if (changed) {
        // Reallocate buffers for new size
        allocate_buffers();
    }
}

}  // namespace ddp
}  // namespace esphome

#endif  // USE_LVGL
