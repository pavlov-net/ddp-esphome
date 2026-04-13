// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// Shared pixel format conversion utilities for DDP renderers
// These functions are designed to be called from the UDP receive task
// (pure math operations, no LVGL/ESPHome API calls, no allocations)

#pragma once

#include <array>
#include <cstdint>
#include <cstddef>

namespace esphome {
namespace ddp {

// RGB to RGBW white channel conversion modes (internal use only)
enum class RGBWMode : uint8_t {
  NONE = 0,      // RGB only, W=0 (for RGB strips)
  ACCURATE = 1,  // Extract white, reduce RGB (for RGBW strips)
};

// RGB565 conversion lookup tables (compile-time generated, C++20 constexpr)
// Used for fast RGB888 → RGB565 conversion
//
// Implementation pattern based on hub75 color_lut.h optimization:
// - Constexpr generator functions return std::array (better constexpr guarantees)
// - Static constexpr storage ensures flash placement via .rodata section
// - Compile-time validation with static_assert
// - Zero runtime overhead, all computation at compile time
//
// Memory: 3 × 256 × 2 bytes = 1.5 KB in flash (not RAM)

namespace detail {
// Generate R5 lookup table: 8-bit red → 5-bit red in RGB565 format
// Maps [0..255] → [0x0000..0xF800] (bits 15-11)
// consteval forces compile-time evaluation only (C++20)
consteval std::array<uint16_t, 256> generate_r5_lut() {
  std::array<uint16_t, 256> lut{};
  for (int i = 0; i < 256; ++i) {
    lut[i] = static_cast<uint16_t>((i & 0xF8) << 8);  // 5 bits red, shifted to MSB
  }
  return lut;
}

// Generate G6 lookup table: 8-bit green → 6-bit green in RGB565 format
// Maps [0..255] → [0x0000..0x07E0] (bits 10-5)
// consteval forces compile-time evaluation only (C++20)
consteval std::array<uint16_t, 256> generate_g6_lut() {
  std::array<uint16_t, 256> lut{};
  for (int i = 0; i < 256; ++i) {
    lut[i] = static_cast<uint16_t>((i & 0xFC) << 3);  // 6 bits green, middle
  }
  return lut;
}

// Generate B5 lookup table: 8-bit blue → 5-bit blue in RGB565 format
// Maps [0..255] → [0x0000..0x001F] (bits 4-0)
// consteval forces compile-time evaluation only (C++20)
consteval std::array<uint16_t, 256> generate_b5_lut() {
  std::array<uint16_t, 256> lut{};
  for (int i = 0; i < 256; ++i) {
    lut[i] = static_cast<uint16_t>(i >> 3);  // 5 bits blue, LSB
  }
  return lut;
}

// Static constexpr storage - guaranteed compile-time evaluation and flash placement
static constexpr auto LUT_R5_ARRAY = generate_r5_lut();
static constexpr auto LUT_G6_ARRAY = generate_g6_lut();
static constexpr auto LUT_B5_ARRAY = generate_b5_lut();
}  // namespace detail

// Public LUT pointers (fully const-correct: const pointers to const data)
inline constexpr const uint16_t *const LUT_R5 = detail::LUT_R5_ARRAY.data();
inline constexpr const uint16_t *const LUT_G6 = detail::LUT_G6_ARRAY.data();
inline constexpr const uint16_t *const LUT_B5 = detail::LUT_B5_ARRAY.data();

// Compile-time validation: verify critical LUT values
// These static_assert statements ensure the LUTs are correctly generated
static_assert(detail::LUT_R5_ARRAY[0x00] == 0x0000, "R5 LUT: black incorrect");
static_assert(detail::LUT_R5_ARRAY[0xF8] == 0xF800, "R5 LUT: max red incorrect");
static_assert(detail::LUT_G6_ARRAY[0x00] == 0x0000, "G6 LUT: black incorrect");
static_assert(detail::LUT_G6_ARRAY[0xFC] == 0x07E0, "G6 LUT: max green incorrect");
static_assert(detail::LUT_B5_ARRAY[0x00] == 0x0000, "B5 LUT: black incorrect");
static_assert(detail::LUT_B5_ARRAY[0xF8] == 0x001F, "B5 LUT: max blue incorrect");

// Convert RGB888 to RGB565 with optional byte swap
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// dst: Destination buffer for RGB565 pixels (must be pre-allocated)
// src: Source RGB888 data (3 bytes per pixel: R, G, B)
// pixel_count: Number of pixels to convert
// swap_bytes: If true, swap byte order (for LV_COLOR_16_SWAP compatibility)
inline void convert_rgb888_to_rgb565(uint16_t *dst, const uint8_t *src, size_t pixel_count, bool swap_bytes) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;

  if (swap_bytes) {
    // Convert and swap bytes for LV_COLOR_16_SWAP
    for (size_t i = 0; i < pixel_count; ++i) {
      uint16_t c = LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]];
      dst[i] = (uint16_t) ((c >> 8) | (c << 8));  // Swap bytes
      sp += 3;
    }
  } else {
    // Convert without swap
    for (size_t i = 0; i < pixel_count; ++i) {
      dst[i] = (uint16_t) (LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]]);
      sp += 3;
    }
  }
}

// Swap RGB565 byte order in-place
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// buf: RGB565 buffer (modified in-place)
// pixel_count: Number of pixels
inline void swap_rgb565_bytes(uint16_t *buf, size_t pixel_count) noexcept {
  if (!buf)
    return;

  for (size_t i = 0; i < pixel_count; ++i) {
    uint16_t c = buf[i];
    buf[i] = (uint16_t) ((c >> 8) | (c << 8));
  }
}

// Convert RGB888 to RGB32 (LVGL 32-bit mode)
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// dst: Destination buffer for RGB32 pixels (must be pre-allocated)
// src: Source RGB888 data (3 bytes per pixel: R, G, B)
// pixel_count: Number of pixels to convert
//
// Note: LVGL lv_color32_t is always BGRA byte order in memory:
//   struct { uint8_t blue; uint8_t green; uint8_t red; uint8_t alpha; }
inline void convert_rgb888_to_rgb32(uint32_t *dst, const uint8_t *src, size_t pixel_count) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;

  for (size_t i = 0; i < pixel_count; ++i) {
    // lv_color32_t is {blue, green, red, alpha} in memory
    uint32_t c = ((uint32_t) sp[2]) |        // B (byte 0, from src[2])
                 ((uint32_t) sp[1] << 8) |   // G (byte 1, from src[1])
                 ((uint32_t) sp[0] << 16) |  // R (byte 2, from src[0])
                 0xFF000000;                 // A (byte 3)
    dst[i] = c;
    sp += 3;
  }
}

// Convert RGB888 (R,G,B) to BGR888 (LVGL 24-bit mode: lv_color_t {blue, green, red})
// Safe to call from UDP task (pure math, no allocations or API calls)
//
// dst: Destination buffer for BGR888 pixels (must be pre-allocated, 3 bytes per pixel)
// src: Source RGB888 data (3 bytes per pixel: R, G, B)
// pixel_count: Number of pixels to convert
inline void convert_rgb888_to_bgr888(uint8_t *dst, const uint8_t *src, size_t pixel_count) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;
  uint8_t *dp = dst;

  for (size_t i = 0; i < pixel_count; ++i) {
    dp[0] = sp[2];  // B
    dp[1] = sp[1];  // G
    dp[2] = sp[0];  // R
    sp += 3;
    dp += 3;
  }
}

// Convert RGB565 to BGR888 (LVGL 24-bit mode: lv_color_t {blue, green, red})
// Safe to call from UDP task (pure math, no allocations or API calls)
//
// dst: Destination buffer for BGR888 pixels (must be pre-allocated, 3 bytes per pixel)
// src: Source RGB565 data (2 bytes per pixel)
// pixel_count: Number of pixels to convert
// src_big_endian: If true, source is RGB565 big-endian; if false, little-endian
inline void convert_rgb565_to_bgr888(uint8_t *dst, const uint8_t *src, size_t pixel_count, bool src_big_endian) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;
  uint8_t *dp = dst;

  for (size_t i = 0; i < pixel_count; ++i) {
    uint16_t v = src_big_endian ? (uint16_t)((sp[0] << 8) | sp[1])
                                : (uint16_t)((sp[1] << 8) | sp[0]);
    uint8_t r5 = (uint8_t)((v >> 11) & 0x1F);
    uint8_t g6 = (uint8_t)((v >> 5) & 0x3F);
    uint8_t b5 = (uint8_t)(v & 0x1F);

    // Expand to 8-bit and store as BGR
    dp[0] = (uint8_t)((b5 << 3) | (b5 >> 2));  // B
    dp[1] = (uint8_t)((g6 << 2) | (g6 >> 4));  // G
    dp[2] = (uint8_t)((r5 << 3) | (r5 >> 2));  // R
    sp += 2;
    dp += 3;
  }
}

// Convert RGBW to BGR888 (LVGL 24-bit mode, drop W channel)
// Safe to call from UDP task (pure math, no allocations or API calls)
//
// dst: Destination buffer for BGR888 pixels (must be pre-allocated, 3 bytes per pixel)
// src: Source RGBW data (4 bytes per pixel: R, G, B, W)
// pixel_count: Number of pixels to convert
inline void convert_rgbw_to_bgr888(uint8_t *dst, const uint8_t *src, size_t pixel_count) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;
  uint8_t *dp = dst;

  for (size_t i = 0; i < pixel_count; ++i) {
    dp[0] = sp[2];  // B
    dp[1] = sp[1];  // G
    dp[2] = sp[0];  // R (W at sp[3] ignored)
    sp += 4;
    dp += 3;
  }
}

// Convert RGBW to RGB565 (drop W channel) with optional byte swap
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// dst: Destination buffer for RGB565 pixels (must be pre-allocated)
// src: Source RGBW data (4 bytes per pixel: R, G, B, W)
// pixel_count: Number of pixels to convert
// swap_bytes: If true, swap byte order (for LV_COLOR_16_SWAP compatibility)
inline void convert_rgbw_to_rgb565(uint16_t *dst, const uint8_t *src, size_t pixel_count, bool swap_bytes) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;

  if (swap_bytes) {
    // Convert and swap bytes for LV_COLOR_16_SWAP
    for (size_t i = 0; i < pixel_count; ++i) {
      uint16_t c = LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]];
      dst[i] = (uint16_t) ((c >> 8) | (c << 8));  // Swap bytes
      sp += 4;                                    // Skip W channel
    }
  } else {
    // Convert without swap
    for (size_t i = 0; i < pixel_count; ++i) {
      dst[i] = (uint16_t) (LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]]);
      sp += 4;  // Skip W channel
    }
  }
}

// Convert RGBW to RGB32 (drop W channel, use 0xFF for alpha)
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// dst: Destination buffer for RGB32 pixels (must be pre-allocated)
// src: Source RGBW data (4 bytes per pixel: R, G, B, W)
// pixel_count: Number of pixels to convert
//
// Note: LVGL lv_color32_t is always BGRA byte order in memory:
//   struct { uint8_t blue; uint8_t green; uint8_t red; uint8_t alpha; }
inline void convert_rgbw_to_rgb32(uint32_t *dst, const uint8_t *src, size_t pixel_count) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;

  for (size_t i = 0; i < pixel_count; ++i) {
    // lv_color32_t is {blue, green, red, alpha} in memory
    uint32_t c = ((uint32_t) sp[2]) |        // B (byte 0, from src[2])
                 ((uint32_t) sp[1] << 8) |   // G (byte 1, from src[1])
                 ((uint32_t) sp[0] << 16) |  // R (byte 2, from src[0])
                 0xFF000000;                 // A (byte 3, src[3]=W ignored)
    dst[i] = c;
    sp += 4;
  }
}

// Convert RGB888 to RGBW with configurable white channel mode
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// Modes:
// - NONE: RGB passthrough, W=0 (for RGB-only strips)
// - ACCURATE: Extract white and reduce RGB (for RGBW strips, accurate color)
//   - Pure colors stay pure: (255,0,0) → R=255, G=0, B=0, W=0
//   - White is efficient: (255,255,255) → R=0, G=0, B=0, W=255
//   - Mixed colors accurate: (255,200,200) → R=55, G=0, B=0, W=200 (pink = red + white)
//
// dst: Destination buffer for RGBW pixels (must be pre-allocated, 4 bytes per pixel)
// src: Source RGB888 data (3 bytes per pixel: R, G, B)
// pixel_count: Number of pixels to convert
// mode: White channel conversion mode
inline void convert_rgb888_to_rgbw(uint8_t *dst, const uint8_t *src, size_t pixel_count, RGBWMode mode) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;

  if (mode == RGBWMode::ACCURATE) {
    // Extract white and reduce RGB (accurate color)
    for (size_t i = 0; i < pixel_count; ++i) {
      uint8_t r = sp[0];
      uint8_t g = sp[1];
      uint8_t b = sp[2];

      // Extract common white component (minimum of RGB)
      uint8_t w = r < g ? (r < b ? r : b) : (g < b ? g : b);

      // Store chromatic residuals + white
      dst[i * 4 + 0] = r - w;  // R residual (chromatic)
      dst[i * 4 + 1] = g - w;  // G residual (chromatic)
      dst[i * 4 + 2] = b - w;  // B residual (chromatic)
      dst[i * 4 + 3] = w;      // W = common brightness
      sp += 3;
    }
  } else {  // RGBWMode::NONE
    // RGB passthrough, W=0
    for (size_t i = 0; i < pixel_count; ++i) {
      dst[i * 4 + 0] = sp[0];  // R
      dst[i * 4 + 1] = sp[1];  // G
      dst[i * 4 + 2] = sp[2];  // B
      dst[i * 4 + 3] = 0;      // W = 0
      sp += 3;
    }
  }
}

// Convert RGB565 to RGBW with configurable white channel mode
// Safe to call from UDP task (pure math, no allocations or API calls)
// Inline so it can be used across components without linkage issues
//
// Algorithm: Expand RGB565 to RGB888, then apply white channel mode
//
// dst: Destination buffer for RGBW pixels (must be pre-allocated, 4 bytes per pixel)
// src: Source RGB565 data (2 bytes per pixel)
// pixel_count: Number of pixels to convert
// src_big_endian: If true, source is RGB565 big-endian; if false, little-endian
// mode: White channel conversion mode
inline void convert_rgb565_to_rgbw(uint8_t *dst, const uint8_t *src, size_t pixel_count, bool src_big_endian,
                                   RGBWMode mode) noexcept {
  if (!dst || !src)
    return;

  const uint8_t *sp = src;

  for (size_t i = 0; i < pixel_count; ++i) {
    // Extract RGB565 value (handle endianness)
    uint16_t v = src_big_endian ? (uint16_t) ((sp[0] << 8) | sp[1]) : (uint16_t) ((sp[1] << 8) | sp[0]);

    // Extract 5/6/5 bit components
    uint8_t r5 = (uint8_t) ((v >> 11) & 0x1F);
    uint8_t g6 = (uint8_t) ((v >> 5) & 0x3F);
    uint8_t b5 = (uint8_t) (v & 0x1F);

    // Expand to 8-bit (scale + replicate MSBs into LSBs)
    uint8_t r = (uint8_t) ((r5 << 3) | (r5 >> 2));
    uint8_t g = (uint8_t) ((g6 << 2) | (g6 >> 4));
    uint8_t b = (uint8_t) ((b5 << 3) | (b5 >> 2));

    if (mode == RGBWMode::ACCURATE) {
      // Extract white and reduce RGB
      uint8_t w = r < g ? (r < b ? r : b) : (g < b ? g : b);
      dst[i * 4 + 0] = r - w;  // R residual (chromatic)
      dst[i * 4 + 1] = g - w;  // G residual (chromatic)
      dst[i * 4 + 2] = b - w;  // B residual (chromatic)
      dst[i * 4 + 3] = w;      // W = common brightness
    } else {                   // RGBWMode::NONE
      // RGB passthrough, W=0
      dst[i * 4 + 0] = r;  // R
      dst[i * 4 + 1] = g;  // G
      dst[i * 4 + 2] = b;  // B
      dst[i * 4 + 3] = 0;  // W = 0
    }
    sp += 2;
  }
}

}  // namespace ddp
}  // namespace esphome
