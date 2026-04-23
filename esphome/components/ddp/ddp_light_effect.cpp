// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// Light effect renderer - renders DDP to AddressableLight strips

#include "ddp_light_effect.h"

// Only compile if light component is available
#ifdef USE_LIGHT

#include "esphome/components/ddp/ddp_pixel_convert.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cstring>
#include <algorithm>
#include <type_traits>

namespace esphome {
namespace ddp {

static const char* TAG = "ddp.light_effect";

// Helper to extract const char* from either std::string or const char*
template<typename T>
static inline const char* get_cstr(const T& val) {
  if constexpr (std::is_pointer_v<T>) {
    return val;  // Already const char*
  } else {
    return val.c_str();  // std::string, call .c_str()
  }
}

// -------- DdpRenderer interface (UDP TASK CONTEXT) --------

void DdpLightEffect::on_data(size_t offset_px, const uint8_t* pixels,
                              PixelFormat format, size_t pixel_count) {
  // UDP TASK CONTEXT - no ESPHome APIs allowed!

  // Allocate buffer if not yet initialized
  if (!frame_buffer_ && frame_pixels_ > 0) {
    frame_buffer_ = allocator_.allocate(frame_pixels_ * 4);  // RGBW
    if (!frame_buffer_) {
      ESP_LOGE(TAG, "Failed to allocate frame buffer (%zu bytes)", frame_pixels_ * 4);
      return;
    }
    std::memset(frame_buffer_, 0, frame_pixels_ * 4);
  }

  if (!frame_buffer_ || frame_pixels_ == 0) return;

  // Bounds check
  if (offset_px + pixel_count > frame_pixels_) {
    return;  // Silently drop out-of-bounds data
  }

  uint8_t* dst_rgbw = frame_buffer_ + (offset_px * 4);

  // Determine RGBW conversion mode based on strip capability
  RGBWMode mode = supports_white_ ? RGBWMode::ACCURATE : RGBWMode::NONE;

  // Convert all formats to RGBW on arrival (single conversion)
  if (format == PixelFormat::RGBW) {
    // Direct copy - already RGBW
    std::memcpy(dst_rgbw, pixels, pixel_count * 4);

  } else if (format == PixelFormat::RGB888) {
    // RGB888 → RGBW (mode-based: ACCURATE for RGBW strips, NONE for RGB strips)
    convert_rgb888_to_rgbw(dst_rgbw, pixels, pixel_count, mode);

  } else if (format == PixelFormat::RGB565_BE || format == PixelFormat::RGB565_LE) {
    // RGB565 → RGBW (mode-based: ACCURATE for RGBW strips, NONE for RGB strips)
    const bool src_big_endian = (format == PixelFormat::RGB565_BE);
    convert_rgb565_to_rgbw(dst_rgbw, pixels, pixel_count, src_big_endian, mode);
  }
}

void DdpLightEffect::on_push() {
  // UDP TASK CONTEXT - just set atomic flag
  frame_ready_.store(true, std::memory_order_release);
}

bool DdpLightEffect::get_dimensions(int* w, int* h) const {
  // LED strips are 1D: report as num_leds × 1
  auto* it = get_addressable_();
  if (it) {
    if (w) *w = it->size();
    if (h) *h = 1;
    return true;
  }

  return false;
}

const char* DdpLightEffect::get_name() const {
  // Access base class name_ member directly (protected)
  // Old ESPHome: name_ is std::string, call .c_str()
  // New ESPHome (PR #11487): name_ is const char*, return directly
  return get_cstr(name_);
}

// -------- AddressableLightEffect interface --------

void DdpLightEffect::start() {
  AddressableLightEffect::start();

  // Note: Renderer registration happens at codegen time (in Python __init__.py)
  // so that DdpComponent::setup() has complete renderer info for mDNS

  // Calculate frame buffer size from LED count
  int num_leds = 0;
  if (get_dimensions(&num_leds, nullptr)) {
    frame_pixels_ = (size_t)num_leds;
  }

  // Detect strip capability for white channel
  auto *it = get_addressable_();
  if (it) {
    auto traits = it->get_traits();
    supports_white_ = traits.supports_color_mode(light::ColorMode::RGB_WHITE);
    ESP_LOGI(TAG, "Started DDP light effect '%s' for stream %u (%d LEDs, %s strip)",
             get_name(), stream_id_, num_leds, supports_white_ ? "RGBW" : "RGB");
  } else {
    ESP_LOGI(TAG, "Started DDP light effect '%s' for stream %u (%d LEDs)",
             get_name(), stream_id_, num_leds);
  }
}

void DdpLightEffect::stop() {
  // Note: We don't unregister since registration is permanent (happens at codegen time)
  ESP_LOGI(TAG, "Stopped DDP light effect '%s' for stream %u", get_name(), stream_id_);

  // Free frame buffer (RGBW = 4 bytes per pixel)
  if (frame_buffer_) {
    allocator_.deallocate(frame_buffer_, frame_pixels_ * 4);
    frame_buffer_ = nullptr;
  }

  AddressableLightEffect::stop();
}

void DdpLightEffect::apply(light::AddressableLight& it, const Color& current_color) {
  // MAIN THREAD CONTEXT - safe to call ESPHome APIs

  // Check if we have a frame ready
  if (!frame_ready_.exchange(false, std::memory_order_acquire)) {
    return;
  }

  if (!frame_buffer_ || frame_pixels_ == 0) return;

  // Get LED count
  int num_leds = it.size();

  // Limit to available LEDs
  size_t leds_to_set = std::min((size_t)num_leds, frame_pixels_);

  // Apply RGBW pixels directly (buffer is always RGBW after on_data conversion)
  const uint8_t* data = frame_buffer_;
  for (size_t i = 0; i < leds_to_set; i++) {
    it[i] = Color(data[i * 4 + 0],  // R
                  data[i * 4 + 1],  // G
                  data[i * 4 + 2],  // B
                  data[i * 4 + 3]); // W
  }

  // Schedule LED update
  it.schedule_show();
}

}  // namespace ddp
}  // namespace esphome

#endif  // USE_LIGHT
