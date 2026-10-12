// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once

// Only compile if light component is available
#ifdef USE_LIGHT

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/light/addressable_light_effect.h"
#include "ddp.h"
#include "ddp_renderer.h"

#include <vector>
#include <atomic>

namespace esphome {
namespace ddp {

// DDP renderer that outputs to AddressableLight as an effect (like E1.31)
class DdpLightEffect : public light::AddressableLightEffect, public DdpRenderer {
 public:
  explicit DdpLightEffect(const char* name) : AddressableLightEffect(name) {}

  // Configuration
  void set_parent(DdpComponent* parent) { parent_ = parent; }
  void set_stream_id(uint8_t id) { stream_id_ = id; }

  // DdpRenderer interface (UDP TASK CONTEXT)
  void on_data(size_t offset_px, const uint8_t* pixels,
               PixelFormat format, size_t pixel_count) override;
  void on_push() override;
  uint8_t get_stream_id() const override { return stream_id_; }
  bool get_dimensions(int* w, int* h) const override;
  const char* get_name() const override;

  // AddressableLightEffect interface (MAIN THREAD)
  void start() override;
  void stop() override;
  void apply(light::AddressableLight& it, const Color& current_color) override;

 protected:
  DdpComponent* parent_{nullptr};
  uint8_t stream_id_{0};

  // Frame buffer - RGBW format (owned by effect, heap-allocated)
  // Large allocation (e.g., 256x4 = 1KB typical), so allocated lazily for PSRAM if available
  uint8_t* frame_buffer_{nullptr};  // RGBW frame buffer
  size_t frame_pixels_{0};          // Expected pixel count (num_leds)
  std::atomic<bool> frame_ready_{false};  // True when frame is ready to apply()

  // White channel support detection
  bool supports_white_{false};  // True if strip supports ColorMode::RGB_WHITE

  // Allocator for frame buffer (tries PSRAM first, falls back to internal RAM)
  RAMAllocator<uint8_t> allocator_;
};

}  // namespace ddp
}  // namespace esphome

#endif  // USE_LIGHT
