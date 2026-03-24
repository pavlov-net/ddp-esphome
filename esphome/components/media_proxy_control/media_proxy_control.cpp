// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "media_proxy_control.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"
#include <cstring>
#include <algorithm>
#include <cstdio>
#include <cctype>

extern "C" {
  #include "esp_websocket_client.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
}

static const char *TAG = "media_proxy_control";

namespace esphome {
namespace media_proxy_control {

// ----------------- MediaProxyOutput implementation -----------------

void MediaProxyOutput::start() {
  if (parent_) {
    parent_->start_stream(stream_id_);
  }
}

void MediaProxyOutput::stop() {
  if (parent_) {
    parent_->stop_stream(stream_id_);
  }
}

void MediaProxyOutput::set_src(const std::string &src) {
  src_ = src;
  if (parent_) {
    parent_->send_update(stream_id_);
  }
}

void MediaProxyOutput::set_pace(int pace) {
  pace_ = pace;
  if (parent_) {
    parent_->send_update(stream_id_);
  }
}

void MediaProxyOutput::set_ema(float ema) {
  ema_ = ema;
  if (parent_) {
    parent_->send_update(stream_id_);
  }
}

void MediaProxyOutput::set_expand(int expand) {
  expand_ = expand;
  if (parent_) {
    parent_->send_update(stream_id_);
  }
}

void MediaProxyOutput::set_loop(bool loop) {
  loop_ = loop;
  if (parent_) {
    parent_->send_update(stream_id_);
  }
}

void MediaProxyOutput::set_hw(const std::string &hw) {
  hw_ = hw;
  if (parent_) {
    parent_->send_update(stream_id_);
  }
}

void MediaProxyOutput::set_format(const std::string &fmt) {
  format_ = fmt;
  if (parent_) {
    parent_->send_update(stream_id_);
  }
}

void MediaProxyOutput::set_fit(const std::string &fit) {
  fit_ = fit;
  if (parent_) {
    parent_->send_update(stream_id_);
  }
}

void MediaProxyOutput::dump_config() {
  ESP_LOGCONFIG(TAG, "MediaProxyOutput: stream_id=%u", stream_id_);
}

// ----------------- local helpers (no header changes required) -----------------
static inline void append_json_str(std::string &dst, const char *key, const std::string &val) {
  dst += "\""; dst += key; dst += "\":\""; dst += val; dst += "\"";
}
static inline void append_json_int(std::string &dst, const char *key, long long v) {
  dst += "\""; dst += key; dst += "\":"; dst += std::to_string(v);
}
static inline void append_json_float(std::string &dst, const char *key, double v) {
  char buf[64]; snprintf(buf, sizeof(buf), "\"%s\":%.6f", key, v); dst += buf;
}
static inline void append_json_bool(std::string &dst, const char *key, bool v) {
  dst += "\""; dst += key; dst += "\":"; dst += (v ? "true" : "false");
}

// Task to cleanup websocket client without blocking main thread
static void cleanup_websocket_task(void *client_handle) {
  if (client_handle) {
    esp_websocket_client_stop((esp_websocket_client_handle_t) client_handle);
    esp_websocket_client_destroy((esp_websocket_client_handle_t) client_handle);
  }
  vTaskDelete(NULL);  // Delete this cleanup task
}

// one canonical place to build the stream JSON (used by start_ and send_update_)
static std::string build_stream_json_(const char *type,
                                      uint8_t stream_id,
                                      int w, int h,
                                      uint16_t ddp_port,
                                      const std::string &src,
                                      const std::optional<std::string> &fmt,
                                      const std::optional<uint8_t> &pixcfg,
                                      const std::optional<int> &pace,
                                      const std::optional<float> &ema,
                                      const std::optional<int> &expand,
                                      const std::optional<bool> &loop,
                                      const std::optional<std::string> &hw,
                                      const std::optional<std::string> &fit) {
  std::string json;
  json.reserve(256);
  json += "{";
  append_json_str(json, "type", type);      json += ",";
  append_json_int(json, "out", stream_id);  json += ",";
  append_json_int(json, "w", w);            json += ",";
  append_json_int(json, "h", h);            json += ",";
  append_json_int(json, "ddp_port", ddp_port); json += ",";
  append_json_str(json, "src", src);
  auto add = [&](auto fn){ json += ","; fn(); };
  if (fmt)    add([&](){ append_json_str(json, "fmt", *fmt); });
  if (pixcfg) add([&](){ append_json_int(json, "pixcfg", (int)*pixcfg); });
  if (pace)   add([&](){ append_json_int(json, "pace", *pace); });
  if (ema)    add([&](){ append_json_float(json, "ema", *ema); });
  if (expand) add([&](){ append_json_int(json, "expand", *expand); });
  if (loop)   add([&](){ append_json_bool(json, "loop", *loop); });
  if (hw)     add([&](){ append_json_str(json, "hw", *hw); });
  if (fit)    add([&](){ append_json_str(json, "fit", *fit); });
  json += "}";
  return json;
}

static void log_stream_line_(const char *label,
                             uint8_t stream_id,
                             int w, int h,
                             uint16_t ddp_port,
                             const std::string &src,
                             const std::optional<std::string> &fmt,
                             const std::optional<uint8_t> &pixcfg,
                             const std::optional<int> &pace,
                             const std::optional<float> &ema,
                             const std::optional<int> &expand,
                             const std::optional<bool> &loop,
                             const std::optional<std::string> &hw,
                             const std::optional<std::string> &fit) {
  std::string pace_str = pace ? std::to_string(*pace) : "(unset)";
  std::string ema_str = ema ? std::to_string(*ema) : "(unset)";
  std::string expand_str = expand ? std::to_string(*expand) : "(unset)";

  ESP_LOGI(TAG, "tx %s stream=%u size=%dx%d src=%s ddp_port=%u fmt=%s pixcfg=0x%02X "
                "pace=%s ema=%s expand=%s loop=%s hw=%s fit=%s",
           label,
           (unsigned) stream_id, w, h, src.c_str(), (unsigned) ddp_port,
           (fmt?fmt->c_str():"(unset)"), (unsigned) (pixcfg?*pixcfg:0),
           pace_str.c_str(),
           ema_str.c_str(),
           expand_str.c_str(),
           (loop?(*loop?"true":"false"):"(unset)"),
           (hw?hw->c_str():"(unset)"),
           (fit?fit->c_str():"(unset)"));
}

// map format string -> ("fmt" field, pixcfg byte). For "rgb565" without endian,
// use platform byte order (LVGL 9 removed LV_COLOR_16_SWAP; canvas uses native order).
static inline std::pair<std::string,uint8_t>
resolve_fmt_and_pixcfg_(const std::string &fmt_in, ddp::DdpComponent *ddp) {
  std::string f = fmt_in;
  for (auto &c : f) c = (char)tolower((unsigned char)c);
  if (f == "rgb888")
    return { "rgb888", ddp::DDP_PIXCFG_RGB888 };
  if (f == "rgb565le")
    return { "rgb565le", ddp::DDP_PIXCFG_RGB565_LE };
  if (f == "rgb565be")
    return { "rgb565be", ddp::DDP_PIXCFG_RGB565_BE };
  if (f == "rgbw")
    return { "rgbw", ddp::DDP_PIXCFG_RGBW };
  if (f == "rgb565") {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    return { "rgb565be", ddp::DDP_PIXCFG_RGB565_BE };
#else
    return { "rgb565le", ddp::DDP_PIXCFG_RGB565_LE };
#endif
  }
  // fallback
  return { "rgb888", ddp::DDP_PIXCFG_RGB888 };
}

// ------------- trampoline -------------
// NOTE: This runs on ESP-IDF event task, NOT the main ESPHome thread.
// Most operations should be deferred to main thread via set_timeout().
void MediaProxyControl::ws_event_trampoline(void *arg,
                                             esp_event_base_t /*base*/,
                                             int32_t event_id,
                                             void * /*event_data*/) {
  auto *self = static_cast<MediaProxyControl *>(arg);
  if (!self) return;

  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
      if (self->running_.load()) break;        // ignore duplicate CONNECTED
      ESP_LOGI(TAG, "connected");
      self->running_.store(true);
      self->connected_.store(true);
      // push all main-thread work off the event task
      self->set_timeout(0, [self]() {
        self->connecting_ = false;
        self->reset_backoff_();  // Reset reconnection backoff on successful connect
        self->send_hello_();
        // Replay only active streams on main thread
        for (auto &kv : self->outputs_) {
          if (kv.second.active) {
            self->start_stream(kv.first);
          }
        }
      });
      break;

    case WEBSOCKET_EVENT_DISCONNECTED:
      ESP_LOGW(TAG, "disconnected");
      if (self->running_.load()) {  // Only handle if we were running
        self->running_.store(false);
        self->connected_.store(false);
        self->set_timeout(0, [self]() {
          self->connecting_ = false;
          self->schedule_reconnect_();
        });
      }
      break;

    case WEBSOCKET_EVENT_CLOSED:
      ESP_LOGW(TAG, "connection closed by server");
      if (self->running_.load()) {  // Only handle if we were running
        self->running_.store(false);
        self->connected_.store(false);
        self->set_timeout(0, [self]() {
          self->connecting_ = false;
          self->schedule_reconnect_();
        });
      }
      break;

    case WEBSOCKET_EVENT_ERROR:
      ESP_LOGE(TAG, "connection error");
      if (self->running_.load()) {  // Only handle if we were running
        self->running_.store(false);
        self->connected_.store(false);
        self->set_timeout(0, [self]() {
          self->connecting_ = false;
          self->schedule_reconnect_();
        });
      }
      break;

    default:
      break;
  }
}

// ------------- config/introspection -------------
void MediaProxyControl::dump_config() {
  ESP_LOGCONFIG(TAG, "Media Proxy WebSocket Control:");
  if (!url_.empty() || (bool)url_fn_) {
    ESP_LOGCONFIG(TAG, "  url: (templated or static)");
  } else {
    ESP_LOGCONFIG(TAG, "  ws_host: %s", ws_host_.c_str());
    ESP_LOGCONFIG(TAG, "  ws_port: %d", ws_port_);
  }
  ESP_LOGCONFIG(TAG, "  device_id: (templated)");
  ESP_LOGCONFIG(TAG, "  outputs: %d", (int) outputs_.size());

  // Print each output configuration
  for (const auto &kv : outputs_) {
    uint8_t id = kv.first;
    const OutputInfo &info = kv.second;
    const MediaProxyOutput* output = info.output;
    if (!output) continue;

    int w = 0, h = 0;
    resolve_size_(id, &w, &h);
    ESP_LOGCONFIG(TAG, "  - stream=%u size=%dx%d", (unsigned) id, w, h);

    std::string pace_str = output->pace_ ? std::to_string(*output->pace_) : "(unset)";
    std::string ema_str = output->ema_ ? std::to_string(*output->ema_) : "(unset)";
    std::string expand_str = output->expand_ ? std::to_string(*output->expand_) : "(unset)";

    ESP_LOGCONFIG(TAG, "      src=%s pace=%s ema=%s expand=%s loop=%s hw=%s format=%s",
                  output->src_.c_str(),
                  pace_str.c_str(),
                  ema_str.c_str(),
                  expand_str.c_str(),
                  output->loop_ ? (*output->loop_ ? "true" : "false") : "(unset)",
                  output->hw_ ? output->hw_->c_str() : "(unset)",
                  output->format_ ? output->format_->c_str() : "(unset)");
  }
}

std::string MediaProxyControl::build_uri_() const {
  if (url_fn_) return url_fn_();
  if (!url_.empty()) return url_;
  const std::string host = ws_host_fn_ ? ws_host_fn_() : ws_host_;
  if (host.empty()) return {};
  char buf[256];
  snprintf(buf, sizeof(buf), "ws://%s:%d/control", host.c_str(), ws_port_);
  return std::string(buf);
}

// ------------- lifecycle -------------
void MediaProxyControl::connect() {
  if (running_.load() || connecting_ || client_) return;   // idempotent
  if (!network::is_connected()) {
    ESP_LOGI(TAG, "network not ready; deferring connect with backoff");
    pending_connect_ = true;
    this->set_timeout(reconnect_delay_ms_, [this](){ this->connect(); });
    reconnect_delay_ms_ = std::min<uint32_t>(reconnect_delay_ms_ * 2, MAX_RECONNECT_DELAY_MS);
    return;
  }

  const std::string uri = this->build_uri_();
  if (uri.empty()) {
    ESP_LOGI(TAG, "URI not ready; retrying connect with backoff");
    pending_connect_ = true;
    this->set_timeout(reconnect_delay_ms_, [this](){ this->connect(); });
    reconnect_delay_ms_ = std::min<uint32_t>(reconnect_delay_ms_ * 2, MAX_RECONNECT_DELAY_MS);
    return;
  }

  pending_connect_ = false;
  connecting_ = true;
  this->do_connect_();
}

void MediaProxyControl::do_connect_() {
  const std::string uri = this->build_uri_();
  if (uri.empty()) {
    ESP_LOGW(TAG, "connect(): no URI (provide url: or ws_host:/ws_port:)");
    connecting_ = false;
    return;
  }

  esp_websocket_client_config_t cfg{};
  cfg.uri = uri.c_str();
  cfg.network_timeout_ms   = 10000;    // Network operation timeout (10s, longer than server ping interval)
  cfg.reconnect_timeout_ms = 0;        // Disable ESP-IDF auto-reconnect, we handle it manually
  cfg.pingpong_timeout_sec = 30;       // WebSocket ping/pong timeout (must be > server heartbeat=20s)
  cfg.keep_alive_enable    = true;     // TCP keepalive
  cfg.keep_alive_idle      = 10;
  cfg.keep_alive_interval  = 10;
  cfg.keep_alive_count     = 3;

  ESP_LOGI(TAG, "attempt uri=%s", uri.c_str());
  auto client = esp_websocket_client_init(&cfg);
  if (!client) {
    ESP_LOGE(TAG, "init failed");
    reconnect_delay_ms_ = std::min<uint32_t>(reconnect_delay_ms_ * 2, MAX_RECONNECT_DELAY_MS);
    connecting_ = false;
    pending_connect_ = true;
    this->set_timeout(reconnect_delay_ms_, [this](){ this->connect(); });
    return;
  }
  client_ = client;

  esp_err_t err = esp_websocket_register_events(
      client_, WEBSOCKET_EVENT_ANY, MediaProxyControl::ws_event_trampoline, this);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "register events failed: 0x%X", (unsigned) err);
    esp_websocket_client_destroy(client_);
    client_ = nullptr;
    reconnect_delay_ms_ = std::min<uint32_t>(reconnect_delay_ms_ * 2, MAX_RECONNECT_DELAY_MS);
    connecting_ = false;
    pending_connect_ = true;
    this->set_timeout(reconnect_delay_ms_, [this](){ this->connect(); });
    return;
  }

  err = esp_websocket_client_start(client_);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "started");
    // leave connecting_=true until CONNECTED arrives
  } else {
    ESP_LOGE(TAG, "start failed: 0x%X", (unsigned) err);
    esp_websocket_client_destroy(client_);
    client_ = nullptr;
    reconnect_delay_ms_ = std::min<uint32_t>(reconnect_delay_ms_ * 2, MAX_RECONNECT_DELAY_MS);
    connecting_ = false;
    pending_connect_ = true;
    this->set_timeout(reconnect_delay_ms_, [this](){ this->connect(); });
  }
}

void MediaProxyControl::disconnect() {
  if (!client_) {
    running_.store(false);
    connected_.store(false);
    connecting_ = false;
    pending_connect_ = false;
    return;
  }
  ESP_LOGI(TAG, "disconnect()");
  esp_websocket_client_stop(client_);
  esp_websocket_client_destroy(client_);
  client_ = nullptr;
  running_.store(false);
  connected_.store(false);
  connecting_ = false;
  pending_connect_ = false;
}

void MediaProxyControl::schedule_reconnect_() {
  if (connecting_ || pending_connect_) return;  // avoid duplicate reconnects

  // Use exponential backoff, capped at 30 seconds
  reconnect_delay_ms_ = std::min<uint32_t>(reconnect_delay_ms_ * 2, MAX_RECONNECT_DELAY_MS);

  ESP_LOGI(TAG, "scheduling reconnect in %ums", (unsigned)reconnect_delay_ms_);
  pending_connect_ = true;

  // Cleanup old client in background task to avoid blocking main thread
  // esp_websocket_client_stop() can block for seconds waiting for network operations
  esp_websocket_client_handle_t old_client = client_;
  client_ = nullptr;  // Clear immediately to prevent reuse

  if (old_client) {
    // Create a cleanup task that runs on a separate thread
    TaskHandle_t cleanup_task;
    if (xTaskCreate(cleanup_websocket_task, "ws_cleanup", 2048, old_client, 5, &cleanup_task) != pdPASS) {
      ESP_LOGW(TAG, "failed to create cleanup task, will leak client handle");
    }
  }

  this->set_timeout(reconnect_delay_ms_, [this]() {
    this->connect();
  });
}

void MediaProxyControl::reset_backoff_() {
  // Reset backoff delay on successful connection
  reconnect_delay_ms_ = 1000;
}

// ------------- protocol helpers -------------
void MediaProxyControl::send_hello_() {
  if (!client_) return;
  const std::string dev = this->device_id_();
  char buf[192];
  snprintf(buf, sizeof(buf),
           "{\"type\":\"hello\",\"proto\":\"ddp-ws/1\",\"device_id\":\"%s\"}",
           dev.c_str());
  esp_websocket_client_send_text(client_, buf, strlen(buf), portMAX_DELAY);
  ESP_LOGI(TAG, "tx hello device_id=%s", dev.c_str());
}

void MediaProxyControl::send_text(const std::string &json_utf8) {
  if (!client_ || !running_.load()) return;
  if (json_utf8.empty()) return;
  esp_websocket_client_send_text(client_, json_utf8.c_str(), json_utf8.length(), portMAX_DELAY);
}

// ------------- orchestration API -------------
void MediaProxyControl::register_output(MediaProxyOutput* output, uint8_t stream_id) {
  OutputInfo info;
  info.output = output;
  info.active = false;
  outputs_[stream_id] = info;
}

void MediaProxyControl::start_stream(uint8_t stream_id) {
  auto it = outputs_.find(stream_id);
  if (it == outputs_.end()) {
    ESP_LOGW(TAG, "start_stream: unknown stream_id=%u", (unsigned) stream_id);
    return;
  }

  // Mark stream as active
  it->second.active = true;

  if (!this->is_connected()) {
    ESP_LOGD(TAG, "start: deferring stream=%u until WS connects", (unsigned) stream_id);
    if (!pending_connect_) this->connect();
    return;
  }

  // Get dimensions
  int w = 0, h = 0;
  resolve_size_(stream_id, &w, &h);

  // If size isn't known yet, defer briefly and retry (only if still active)
  if (w == 0 || h == 0) {
    ESP_LOGD(TAG, "start_stream stream=%u deferred: size not ready (w=%d h=%d)",
             (unsigned) stream_id, w, h);
    this->set_timeout(200, [this, stream_id]() {
      // only retry if still active and connected
      auto it = this->outputs_.find(stream_id);
      if (it != this->outputs_.end() && it->second.active && this->is_connected()) {
        this->start_stream(stream_id);
      }
    });
    return;
  }

  // Size is known: send the real start now
  this->send_stream_("start_stream", stream_id);
}

void MediaProxyControl::stop_stream(uint8_t stream_id) {
  auto it = outputs_.find(stream_id);
  if (it != outputs_.end()) {
    it->second.active = false;
  }

  if (!client_ || !running_.load()) return;
  std::string json;
  json.reserve(48);
  json += "{\"type\":\"stop_stream\",\"out\":";
  json += std::to_string(stream_id);
  json += "}";
  ESP_LOGI(TAG, "tx stop_stream stream=%u", (unsigned) stream_id);
  this->send_text(json);
}

void MediaProxyControl::send_update(uint8_t stream_id) {
  auto it = outputs_.find(stream_id);
  if (it == outputs_.end()) return;
  if (it->second.active && this->is_connected()) {
    this->send_stream_("update", stream_id);
  }
}

// --------- size/fmt/escape helpers ----------
std::string MediaProxyControl::json_escape_(const std::string &s) {
  std::string o; o.reserve(s.size()+8);
  for (char c: s) {
    switch (c) {
      case '\"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\b': o += "\\b"; break;
      case '\f': o += "\\f"; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default:
        if ((unsigned char)c < 0x20) { char buf[7]; snprintf(buf, sizeof(buf), "\\u%04x", (unsigned char)c); o += buf; }
        else o += c;
    }
  }
  return o;
}

void MediaProxyControl::resolve_size_(uint8_t stream_id, int *w, int *h) const {
  int W = 0, H = 0;

  // Get dimensions from DDP component
  if (ddp_) {
    ddp_->get_stream_dimensions(stream_id, &W, &H);
    // Note: Don't log failure here - dimensions may not be ready yet during startup
  }

  if (w) *w = W;
  if (h) *h = H;
}

// Produce a fully-resolved config for a given stream (YAML + overrides + ddp)
MediaProxyControl::StreamConfig MediaProxyControl::compute_stream_config_(uint8_t stream_id) const {
  StreamConfig e{};

  // size + port
  this->resolve_size_(stream_id, &e.w, &e.h);
  e.ddp_port = ddp_ ? ddp_->get_port() : 4048;

  // look up output config
  auto oit = outputs_.find(stream_id);
  const MediaProxyOutput* output = (oit != outputs_.end()) ? oit->second.output : nullptr;

  // strings first (escaped); src is now required
  if (output) e.src = json_escape_(output->src_);
  if (output && output->hw_) e.hw = json_escape_(*output->hw_);

  // numeric/toggles — propagate only if set in YAML
  if (output && output->pace_) e.pace = *output->pace_;
  if (output && output->ema_) e.ema = *output->ema_;
  if (output && output->expand_) e.expand = *output->expand_;
  if (output && output->loop_) e.loop = *output->loop_;

  // format → (fmt,pixcfg)
  if (output && output->format_) {
    auto rp = resolve_fmt_and_pixcfg_(*output->format_, ddp_);
    e.fmt = rp.first;
    e.pixcfg = rp.second;
  }

  // fit
  if (output && output->fit_) {
    e.fit = json_escape_(*output->fit_);
  }

  return e;
}

// Single place to build/log/send "start_stream" or "update"
void MediaProxyControl::send_stream_(const char *type, uint8_t stream_id) {
  if (!client_ || !running_.load()) return;
  auto oit = outputs_.find(stream_id);
  if (oit == outputs_.end()) {
    ESP_LOGW(TAG, "%s: unknown stream_id=%u", type, (unsigned) stream_id);
    return;
  }

  StreamConfig e = this->compute_stream_config_(stream_id);
  const std::string json = build_stream_json_(type, stream_id, e.w, e.h, e.ddp_port,
                                              e.src, e.fmt, e.pixcfg,
                                              e.pace, e.ema, e.expand, e.loop, e.hw, e.fit);
  log_stream_line_(type, stream_id, e.w, e.h, e.ddp_port,
                   e.src, e.fmt, e.pixcfg, e.pace, e.ema, e.expand, e.loop, e.hw, e.fit);
  this->send_text(json);
}

}  // namespace media_proxy_control
}  // namespace esphome
