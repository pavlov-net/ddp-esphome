# ddp-esphome

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![ESPHome](https://img.shields.io/badge/ESPHome-2026.4+-blue)](https://esphome.io/)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-5.x-orange)](https://docs.espressif.com/projects/esp-idf/)

ESPHome external components for streaming images/video to **LVGL** displays and **AddressableLight** LED strips via **DDP** (Distributed Display Protocol) over UDP.

**📚 Documentation:**
- [DDP Protocol Specification](http://www.3waylabs.com/ddp/) - Official DDP protocol documentation

## Components

This repository provides a modular set of ESPHome components:

- **`ddp`**: Core UDP server that receives DDP packets and dispatches to renderers
- **`ddp_canvas`**: LVGL canvas renderer with triple-buffering support
- **`ddp_light_effect`**: AddressableLight effect for RGB/RGBW LED strips
- **`media_proxy_control`**: WebSocket client for orchestrating media playback (images, videos, etc.) via the [media-proxy server](https://github.com/stuartparmenter/media-proxy)

> Requires **ESPHome 2026.4.0 or newer** (which bundles LVGL 9.5). ESP-IDF only — Arduino is **not** supported.

> **Note:** Legacy components `ddp_stream` and `ws_ddp_control` were removed in v0.7.0. Users on these components should use **v0.6.1** or earlier, or migrate to the new modular architecture. See examples for migration guidance.

### Standalone Usage

The `ddp` component can be used independently with any system that sends DDP packets to UDP port 4048. This includes:

- **[LedFx](https://github.com/LedFx/LedFx)** - Real-time music visualization software with excellent DDP support. Works great with this component. **Note:** LedFx always uses stream ID 1, so configure your renderer with `stream: 1` for compatibility.
- **[WLEDVideoSync](https://github.com/Aircoookie/WLED/wiki/UDP-Realtime-Control#videosync)** - Though compatibility hasn't been extensively tested.

You only need `media_proxy_control` if you want to use the companion media proxy server for video processing and WebSocket orchestration.

### Pixel Format Notes

- **RGB888** (24-bit, DDP type `0x0B`): Standard DDP format, 8 bits per channel. Used by most DDP implementations including WLED and LedFx.
- **RGB565** (16-bit, DDP type `0x62` little-endian / `0x61` big-endian): Extension to DDP spec. Recommended for ESP32 since LVGL typically runs in 16-bit color mode. Reduces bandwidth by ~33%.
- **RGBW** (32-bit, DDP type `0x1B`): Standard DDP format with white channel. Perfect for RGBW LED strips.
- **Automatic rendering**: The components automatically render whatever pixel format they receive (RGB888, RGB565LE, RGB565BE, or RGBW) as long as the DDP packet type flags are set correctly.

---

## Examples

Multiple ready-made example configurations are provided:

- **[media-proxy-canvas.yaml](esphome/examples/media-proxy-canvas.yaml)** - **Recommended starting point**: Canvas video streaming with media proxy control
- **[ddp-test-canvas.yaml](esphome/examples/ddp-test-canvas.yaml)** - Minimal standalone canvas example (no media proxy)
- **[ddp-test-light-rgb.yaml](esphome/examples/ddp-test-light-rgb.yaml)** - RGB LED strip streaming
- **[ddp-test-light-rgbw.yaml](esphome/examples/ddp-test-light-rgbw.yaml)** - RGBW LED strip streaming
- **[ddp-test-multi-renderer.yaml](esphome/examples/ddp-test-multi-renderer.yaml)** - Advanced: Mirror same stream to canvas AND LEDs simultaneously

Move sensitive values like Wi-Fi credentials and video sources to your `secrets.yaml`.

---

## Media Proxy Server

For video processing and WebSocket orchestration, this project works with a companion media proxy server:

**Repository**: https://github.com/stuartparmenter/media-proxy

The media proxy handles:
- Media ingestion, resizing, and encoding
- WebSocket control interface
- Home Assistant add-on support

---

## Security notes

- DDP runs over UDP and is unauthenticated. Run only on a trusted LAN or VPN.  
- Do not expose the media-proxy control port (8788) or DDP port (4048) to the public internet.

---

## Acknowledgements

This project was inspired in part by:
- [LedFx](https://github.com/LedFx/LedFx) - Real-time music visualization with DDP protocol support
- [WLED Video Sync](https://github.com/Aircoookie/WLED/wiki/UDP-Realtime-Control#videosync)
- [DDP Protocol Specification](http://www.3waylabs.com/ddp/)

Many thanks to those projects for pioneering low-latency LED streaming approaches.

---

## Related Projects & Documentation

- [LVGL Documentation](https://docs.lvgl.io/) - Lightweight embedded graphics library used for rendering.
- [ESPHome](https://esphome.io/) - Framework for building firmware for ESP32/ESP8266 devices.
- [ESP WebSocket Client](https://github.com/espressif/esp-protocols/tree/master/components/esp_websocket_client) - WebSocket client component used by `ws_ddp_control`.

## License

MIT - see [LICENSE](LICENSE).
