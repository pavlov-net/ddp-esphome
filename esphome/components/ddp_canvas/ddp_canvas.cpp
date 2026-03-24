// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// LVGL canvas renderer implementation

#include "ddp_canvas.h"
#include "esphome/components/ddp/ddp_pixel_convert.h"
#include "esphome/core/log.h"
#include "esp_timer.h"
#include <cstring>
#include <algorithm>

namespace esphome {
namespace ddp {

static const char* TAG = "ddp.canvas";

// LVGL assertions
static_assert(LV_COLOR_DEPTH == 16 || LV_COLOR_DEPTH == 24 || LV_COLOR_DEPTH == 32,
              "LV_COLOR_DEPTH must be 16, 24, or 32");
static constexpr size_t BYTES_PER_PIXEL = LV_COLOR_DEPTH / 8;

static const char* color_format_name(lv_color_format_t cf) {
  switch (cf) {
    case LV_COLOR_FORMAT_RGB565:                return "RGB565";
    case LV_COLOR_FORMAT_RGB888:                return "RGB888";
    case LV_COLOR_FORMAT_ARGB8888:              return "ARGB8888";
    case LV_COLOR_FORMAT_XRGB8888:              return "XRGB8888";
    case LV_COLOR_FORMAT_ARGB8565:              return "ARGB8565";
    case LV_COLOR_FORMAT_I1:                    return "I1";
    case LV_COLOR_FORMAT_I2:                    return "I2";
    case LV_COLOR_FORMAT_I4:                    return "I4";
    case LV_COLOR_FORMAT_I8:                    return "I8";
    case LV_COLOR_FORMAT_A1:                    return "A1";
    case LV_COLOR_FORMAT_A2:                    return "A2";
    case LV_COLOR_FORMAT_A4:                    return "A4";
    case LV_COLOR_FORMAT_A8:                    return "A8";
    case LV_COLOR_FORMAT_L8:                    return "L8";
    default:                                    return "UNKNOWN";
  }
}

// -------- DdpRenderer interface (UDP TASK CONTEXT) --------

void DdpCanvas::on_data(size_t offset_px, const uint8_t* pixels,
                        PixelFormat format, size_t pixel_count) {
  // UDP TASK CONTEXT - no LVGL or ESPHome APIs allowed!

  // Check if canvas is bound
  if (!bound_ || buf_px_ == 0) {
    return;
  }

  // Bounds check
  if (offset_px + pixel_count > buf_px_) {
    return;  // Silently drop out-of-bounds data
  }

  // Update receiving sensor timestamp (deferred publish to main loop)
  if (receiving_sensor_) {
    last_frame_us_.store(esp_timer_get_time(), std::memory_order_relaxed);
    need_sensor_update_.store(true, std::memory_order_relaxed);
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

  // Select destination buffer
  uint16_t* dst_buf = nullptr;
  if (back_buffers_ == 0) {
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

  if (format == PixelFormat::RGB888) {
    // RGB888 → native RGB565 (no byte swap: LVGL 9 canvas uses native byte order)
    convert_rgb888_to_rgb565(dst, pixels, pixel_count, false);

  } else if (format == PixelFormat::RGB565_BE || format == PixelFormat::RGB565_LE) {
    // RGB565 → native RGB565
    if (format == PixelFormat::RGB565_LE) {
      // Source is LE (native on ESP32) — direct copy
      std::memcpy(dst, pixels, pixel_count * 2);
    } else {
      // Source is BE — copy + byte swap to native LE
      std::memcpy(dst, pixels, pixel_count * 2);
      swap_rgb565_bytes(dst, pixel_count);
    }

  } else if (format == PixelFormat::RGBW) {
    // RGBW → native RGB565 (drop W channel)
    convert_rgbw_to_rgb565(dst, pixels, pixel_count, false);
  }

#elif LV_COLOR_DEPTH == 24
  // LVGL 9 RGB888: lv_color_t is {blue, green, red} — BGR byte order
  uint8_t* dst24 = reinterpret_cast<uint8_t*>(dst_buf) + offset_px * 3;

  if (format == PixelFormat::RGB888) {
    // RGB888 (R,G,B) → BGR888 (swap R↔B)
    convert_rgb888_to_bgr888(dst24, pixels, pixel_count);

  } else if (format == PixelFormat::RGB565_BE || format == PixelFormat::RGB565_LE) {
    // RGB565 → BGR888 (expand + reorder)
    const bool src_be = (format == PixelFormat::RGB565_BE);
    convert_rgb565_to_bgr888(dst24, pixels, pixel_count, src_be);

  } else if (format == PixelFormat::RGBW) {
    // RGBW → BGR888 (drop W, swap R↔B)
    convert_rgbw_to_bgr888(dst24, pixels, pixel_count);
  }

#elif LV_COLOR_DEPTH == 32
  uint32_t* dst32 = reinterpret_cast<uint32_t*>(dst_buf) + offset_px;

  if (format == PixelFormat::RGB888) {
    // RGB888 → RGB32 (lv_color32_t: BGRA byte order)
    convert_rgb888_to_rgb32(dst32, pixels, pixel_count);

  } else if (format == PixelFormat::RGB565_BE || format == PixelFormat::RGB565_LE) {
    // RGB565 → RGB32 (lv_color32_t: BGRA byte order)
    const bool src_be = (format == PixelFormat::RGB565_BE);
    const uint8_t* sp = pixels;

    for (size_t i = 0; i < pixel_count; ++i) {
      uint16_t v = src_be ? (uint16_t)((sp[0] << 8) | sp[1])
                          : (uint16_t)((sp[1] << 8) | sp[0]);
      uint8_t r5 = (uint8_t)((v >> 11) & 0x1F);
      uint8_t g6 = (uint8_t)((v >> 5)  & 0x3F);
      uint8_t b5 = (uint8_t)( v        & 0x1F);

      // Expand to 8-bit: lv_color32_t is {blue, green, red, alpha} in memory
      uint32_t c = ((b5 << 3) | (b5 >> 2)) |         // B (byte 0)
                   (((g6 << 2) | (g6 >> 4)) << 8) |  // G (byte 1)
                   (((r5 << 3) | (r5 >> 2)) << 16) | // R (byte 2)
                   (0xFF << 24);                     // A (byte 3)
      dst32[i] = c;
      sp += 2;
    }

  } else if (format == PixelFormat::RGBW) {
    // RGBW → RGB32 (lv_color32_t: BGRA byte order, drop W channel)
    convert_rgbw_to_rgb32(dst32, pixels, pixel_count);
  }
#endif
}

void DdpCanvas::on_push() {
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
  if (back_buffers_ == 2) {
    // Triple-buffer: swap accum↔ready
    std::swap(ready_buf_, accum_buf_);
    have_ready_.store(true, std::memory_order_release);
  } else if (back_buffers_ == 1) {
    // Double-buffer: signal copy needed
    need_copy_to_front_.store(true, std::memory_order_relaxed);
    need_invalidate_.store(true, std::memory_order_relaxed);
  } else {
    // No buffering: just invalidate
    need_invalidate_.store(true, std::memory_order_relaxed);
  }
}

bool DdpCanvas::get_dimensions(int* w, int* h) const {
  if (w_ <= 0 || h_ <= 0) return false;
  if (w) *w = w_;
  if (h) *h = h_;
  return true;
}

// -------- ESPHome lifecycle --------

void DdpCanvas::setup() {
  // Note: Renderer registration happens at codegen time (in Python __init__.py)
  // so that DdpComponent::setup() has complete renderer info for mDNS

  // Initialize receiving sensor to false
  if (receiving_sensor_) {
    receiving_sensor_->publish_state(false);
    last_receiving_state_ = false;
  }

  // Set up periodic binding check (canvas may not be ready yet)
  this->set_interval("bind", 250, [this]() {
    bool before = bound_;
    bind_if_possible_();
    bool after = bound_;

    if (!before && after) {
      ESP_LOGI(TAG, "Stream %u bound to canvas=%p (%dx%d)",
               stream_id_, (void*)canvas_, w_, h_);
      lv_canvas_fill_bg(canvas_, lv_color_black(), LV_OPA_COVER);

      // Stop checking once bound
      this->cancel_interval("bind");
    }
  });
}

void DdpCanvas::loop() {
  // MAIN THREAD CONTEXT - safe to call LVGL and ESPHome APIs

  // Check receiving sensor timeout first (before processing frames)
  if (receiving_sensor_ && last_receiving_state_) {
    int64_t last_frame = last_frame_us_.load(std::memory_order_relaxed);
    int64_t now_us = esp_timer_get_time();
    if (last_frame > 0 && (now_us - last_frame) > 1'000'000) {  // 1 second timeout
      receiving_sensor_->publish_state(false);
      last_receiving_state_ = false;
    }
  }

  // Deferred sensor update from UDP task (turn ON when data arrives)
  if (need_sensor_update_.exchange(false, std::memory_order_relaxed)) {
    if (receiving_sensor_ && !last_receiving_state_) {
      receiving_sensor_->publish_state(true);
      last_receiving_state_ = true;
    }
  }

  // Triple-buffer path: copy ready→front (main thread, LVGL-safe)
  if (back_buffers_ == 2) {
    if (have_ready_.exchange(false, std::memory_order_acquire)) {
      if (canvas_ && buf_px_ > 0 && front_buf_ && ready_buf_) {
        std::memcpy(front_buf_, ready_buf_, buf_px_ * BYTES_PER_PIXEL);
      }
      need_invalidate_.store(true, std::memory_order_relaxed);
    }
  }

  // Double-buffer path: copy accum→front (main thread)
  if (back_buffers_ == 1 && need_copy_to_front_.exchange(false)) {
    if (front_buf_ && accum_buf_ && buf_px_) {
      std::memcpy(front_buf_, accum_buf_, buf_px_ * BYTES_PER_PIXEL);
    }
    need_invalidate_.store(true, std::memory_order_relaxed);
  }

  // Invalidate canvas if needed (LVGL API - must be on main thread)
  // This is where actual presentation happens, so track metrics here
  if (need_invalidate_.exchange(false)) {
    if (canvas_) {
      lv_obj_invalidate(canvas_);
    }

#ifdef DDP_METRICS
    // Update presentation metrics (all buffer modes)
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

#ifdef DDP_METRICS
void DdpCanvas::reset_windowed_metrics() {
  metrics_.win_frames_presented = 0;
}
#endif

void DdpCanvas::dump_config() {
  // Note: DDP component already logs stream ID and dimensions
  // Only log canvas-specific details here
  ESP_LOGCONFIG(TAG, "DdpCanvas (stream %u):", stream_id_);
  ESP_LOGCONFIG(TAG, "  Back buffers: %u", back_buffers_);
  if (receiving_sensor_) {
    LOG_BINARY_SENSOR("  ", "Receiving Sensor", receiving_sensor_);
  }
}

// -------- Canvas binding --------

void DdpCanvas::bind_if_possible_() {
  if (bound_) return;

  // Try to get canvas from getter
  if (!canvas_ && getter_) {
    canvas_ = getter_();
  }
  if (!canvas_) return;

  // Auto-detect dimensions if not specified
  if (w_ <= 0 || h_ <= 0) {
    int W = (int)lv_obj_get_width(canvas_);
    int H = (int)lv_obj_get_height(canvas_);
    if (w_ <= 0) w_ = W;
    if (h_ <= 0) h_ = H;
  }
  if (w_ <= 0 || h_ <= 0) return;

  // Allocate buffers
  ensure_buffers_();

  // Check if all required buffers are allocated
  if (!front_buf_) return;
  if (back_buffers_ == 2 && (!ready_buf_ || !accum_buf_)) return;
  if (back_buffers_ == 1 && !accum_buf_) return;

  // Clear canvas to black
  lv_canvas_fill_bg(canvas_, lv_color_black(), LV_OPA_COVER);

  // Register for size change events
  lv_obj_add_event_cb(canvas_, &DdpCanvas::on_canvas_size_changed_, LV_EVENT_SIZE_CHANGED, this);

  bound_ = true;
}

void DdpCanvas::ensure_buffers_() {
  if (!canvas_) return;

  // Adopt the canvas' existing buffer as our front (owned by LVGL canvas)
  auto* draw_buf = lv_canvas_get_draw_buf(canvas_);
  if (!draw_buf || !draw_buf->data || draw_buf->header.w <= 0 || draw_buf->header.h <= 0) {
    // Canvas not fully initialized yet
    return;
  }

  const size_t px = (size_t)draw_buf->header.w * (size_t)draw_buf->header.h;

  // Log whenever the canvas buffer pointer changes or size changes
  if (front_buf_ != (uint16_t*)draw_buf->data || buf_px_ != px) {
    ESP_LOGI(TAG,
      "Using canvas draw_buf: cv=%p buf=%p data=%p w=%d h=%d cf=%u(%s) LV_COLOR_DEPTH=%d buf_px(old=%u -> new=%u)",
      (void*)canvas_, (void*)draw_buf, (void*)draw_buf->data,
      (int)draw_buf->header.w, (int)draw_buf->header.h,
      (unsigned)draw_buf->header.cf, color_format_name((lv_color_format_t)draw_buf->header.cf),
      (int)LV_COLOR_DEPTH,
      (unsigned)buf_px_, (unsigned)px);
  }

  front_buf_ = (uint16_t*)draw_buf->data;

  // (Re)allocate only ready/accum when size changes
  if (buf_px_ != px) {
    free_ready_accum_();
    buf_px_ = px;
  }

  size_t bytes = px * BYTES_PER_PIXEL;

  // Allocate ready buffer (triple-buffer mode)
  if (back_buffers_ == 2 && !ready_buf_) {
    ready_buf_ = static_cast<uint16_t*>(lv_malloc(bytes));
    if (!ready_buf_) {
      free_ready_accum_();
      buf_px_ = 0;
      return;
    }
    std::memset(ready_buf_, 0, bytes);
  }

  // Allocate accum buffer (double or triple-buffer mode)
  if ((back_buffers_ >= 1) && !accum_buf_) {
    accum_buf_ = static_cast<uint16_t*>(lv_malloc(bytes));
    if (!accum_buf_) {
      free_ready_accum_();
      buf_px_ = 0;
      return;
    }
    std::memset(accum_buf_, 0, bytes);
  }

  // Free extra buffers if mode changed
  if (back_buffers_ < 2 && ready_buf_) {
    lv_free(ready_buf_);
    ready_buf_ = nullptr;
  }
  if (back_buffers_ < 1 && accum_buf_) {
    lv_free(accum_buf_);
    accum_buf_ = nullptr;
  }
}

void DdpCanvas::free_ready_accum_() {
  if (ready_buf_) {
    lv_free(ready_buf_);
    ready_buf_ = nullptr;
  }
  if (accum_buf_) {
    lv_free(accum_buf_);
    accum_buf_ = nullptr;
  }
  have_ready_.store(false);
}

void DdpCanvas::on_canvas_size_changed_(lv_event_t* e) {
  auto* canvas = lv_event_get_target_obj(e);
  auto* self = static_cast<DdpCanvas*>(lv_event_get_user_data(e));
  if (!self || !canvas || canvas != self->canvas_) return;

  // Only handle if dimensions were auto-detected
  bool auto_any = (self->w_ <= 0 || self->h_ <= 0);
  if (!auto_any) return;

  int W = (int)lv_obj_get_width(canvas);
  int H = (int)lv_obj_get_height(canvas);
  if (self->w_ <= 0) self->w_ = W;
  if (self->h_ <= 0) self->h_ = H;

  // Re-adopt the canvas buffer and resize ready/accum if needed
  self->ensure_buffers_();
}


}  // namespace ddp
}  // namespace esphome
