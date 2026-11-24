// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once

// Only compile if LVGL component is available
#ifdef USE_LVGL

#include "ddp_renderer.h"
#include "ddp_img_widget.h"  // Need full definition of lv_ddpimg_t
#include <atomic>
#include <cstdint>
#include <cstddef>

namespace esphome {
namespace ddp {

/**
 * C++ implementation class for DDP image widget
 * Implements DdpRenderer interface for UDP task callbacks
 *
 * This class handles all the DDP streaming logic (buffer management,
 * pixel conversion, etc.) while the C widget struct handles LVGL integration.
 */
class DdpImgWidgetImpl : public DdpRenderer {
 public:
  explicit DdpImgWidgetImpl(lv_ddpimg_t* widget) : widget_(widget) {}
  ~DdpImgWidgetImpl() override { free_buffers(); }

  // DdpRenderer interface (UDP TASK CONTEXT)
  void on_data(size_t offset_px, const uint8_t* pixels,
               PixelFormat format, size_t pixel_count) override;
  void on_push() override;
  uint8_t get_stream_id() const override;
  bool get_dimensions(int* w, int* h) const override;
  const char* get_name() const override { return "DdpImgWidget"; }
  void loop() override;  // Check invalidation flag and invalidate widget if needed

#ifdef DDP_METRICS
  const RendererMetrics* get_metrics() const override { return &metrics_; }
  void reset_windowed_metrics() override;
#endif

  // Buffer management (MAIN THREAD CONTEXT)
  void allocate_buffers();
  void free_buffers();

  // Called from LVGL event callbacks (MAIN THREAD CONTEXT)
  void present_if_ready();  // Swap buffers, update front
  void handle_size_change();

 private:
  lv_ddpimg_t* widget_;      // Back-reference to C widget struct (NOT OWNED)

  // Triple-buffering (same logic as ddp_canvas)
  // All buffers use LVGL's native color format (LV_COLOR_DEPTH=16 or 32)
  uint16_t* front_buf_{nullptr};   // Currently displayed (pointed to by img_dsc)
  uint16_t* ready_buf_{nullptr};   // Next to present (filled on PUSH)
  uint16_t* accum_buf_{nullptr};   // RX writes here
  size_t buf_px_{0};               // Buffer size in pixels

  std::atomic<bool> have_ready_{false};
  std::atomic<bool> need_copy_to_front_{false};
  std::atomic<bool> need_invalidate_{false};

#ifdef DDP_METRICS
  // Performance metrics
  RendererMetrics metrics_;

  // Frame assembly tracking (for coverage calculation)
  std::atomic<size_t> frame_bytes_accum_{0};
  std::atomic<int64_t> frame_first_pkt_us_{0};
  std::atomic<int64_t> ready_set_us_{0};  // When frame became ready (for queue wait)
#endif
};

}  // namespace ddp
}  // namespace esphome

#endif  // USE_LVGL
