// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// Core DDP protocol implementation - UDP reception and renderer dispatch

#include "ddp.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/components/network/util.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include "esp_timer.h"
#include <sys/time.h>

namespace esphome {
namespace ddp {

static const char* TAG = "ddp";

// ---------- helpers ----------

#if DDP_METRICS
static inline double ewma(double prev, double sample, double alpha = 0.2) {
  return (prev == 0.0) ? sample : (alpha * sample + (1.0 - alpha) * prev);
}
#endif

// -------- public API --------

void DdpComponent::register_renderer(DdpRenderer* renderer) {
  if (!renderer) return;

  uint8_t stream_id = renderer->get_stream_id();
  renderers_[stream_id].insert(renderer);

  ESP_LOGI(TAG, "Registered renderer '%s' for stream %u",
           renderer->get_name(), stream_id);
}

void DdpComponent::unregister_renderer(DdpRenderer* renderer) {
  if (!renderer) return;

  uint8_t stream_id = renderer->get_stream_id();
  auto it = renderers_.find(stream_id);
  if (it != renderers_.end()) {
    it->second.erase(renderer);
    ESP_LOGI(TAG, "Unregistered renderer '%s' for stream %u",
             renderer->get_name(), stream_id);
  }
}

bool DdpComponent::get_stream_dimensions(uint8_t stream_id, int* width, int* height) const {
  auto it = renderers_.find(stream_id);
  if (it == renderers_.end()) return false;

  // Find first renderer with known dimensions
  for (auto* renderer : it->second) {
    if (renderer->get_dimensions(width, height)) {
      return true;
    }
  }

  return false;
}

// -------- lifecycle --------

void DdpComponent::setup() {
  ESP_LOGI(TAG, "setup this=%p port=%u", this, (unsigned) port_);

  this->set_interval("ddp_net", 500, [this]() { this->ensure_socket_(); });

  ESP_LOGI(TAG, "initialized; waiting for network to open UDP %u", this->port_);
}

void DdpComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "DDP Component:");
  ESP_LOGCONFIG(TAG, "  UDP port: %u", port_);
  ESP_LOGCONFIG(TAG, "  Registered streams: %u", (unsigned)renderers_.size());

  for (auto &kv : renderers_) {
    uint8_t stream_id = kv.first;
    auto& renderer_set = kv.second;
    ESP_LOGCONFIG(TAG, "  Stream %u: %u renderer(s)", stream_id, (unsigned)renderer_set.size());
    for (auto* renderer : renderer_set) {
      int w, h;
      bool has_dims = renderer->get_dimensions(&w, &h);
      if (has_dims) {
        ESP_LOGCONFIG(TAG, "    - %s (%dx%d)", renderer->get_name(), w, h);
      } else {
        ESP_LOGCONFIG(TAG, "    - %s (dimensions unknown)", renderer->get_name());
      }
    }
  }
}

void DdpComponent::loop() {
  bool had_activity = false;

#if DDP_METRICS
  // Periodic windowed log (every ~2s)
  int64_t now = esp_timer_get_time();
  for (auto &kv : metrics_) {
    uint8_t id = kv.first;
    StreamMetrics &m = kv.second;

    if (m.log_t0_us == 0) m.log_t0_us = now;
    int64_t dt_us = now - m.log_t0_us;
    if (dt_us >= 2'000'000) {
      double dt_s = (double)dt_us / 1e6;

      double push_fps = m.win_frames_push / dt_s;
      double rx_mbps      = (m.win_rx_bytes * 8.0) / (dt_s * 1e6);
      double rx_wire_mbps = (m.win_rx_wire_bytes * 8.0) / (dt_s * 1e6);
      double lat_ms   = m.dispatch_lat_us_ewma / 1000.0;
      double pkts_per_wakeup = (m.rx_wakeups > 0)
        ? ((double)m.rx_pkts_in_slices / (double)m.rx_wakeups)
        : 0.0;

      // Only log if there was activity in this window
      if (m.win_frames_push > 0 || m.win_rx_bytes > 0) {
        had_activity = true;

#ifdef DDP_METRICS
        // Query renderer metrics if available
        auto renderers_it = renderers_.find(id);
        const RendererMetrics* r_metrics = nullptr;
        DdpRenderer* renderer = nullptr;
        if (renderers_it != renderers_.end() && !renderers_it->second.empty()) {
          // Use first renderer for this stream (usually only one)
          renderer = *renderers_it->second.begin();
          r_metrics = renderer->get_metrics();
        }

        // Build combined log with protocol + renderer metrics
        if (r_metrics && renderer) {
          double render_fps = r_metrics->win_frames_presented / dt_s;
          double render_cov_pct = r_metrics->coverage_ewma * 100.0;
          double render_lat_ms = r_metrics->present_lat_us_ewma / 1000.0;
          double fps_delta = render_fps - push_fps;  // negative = falling behind

          ESP_LOGI(TAG,
            "id=%u rx_pkts=%llu rx_B=%llu win{push=%.1ffps rx=%.2fMb/s wire=%.2fMb/s pkt_gap=%u} "
            "tot{push=%u} build(avg)=%.1f ms lat(avg)=%.1f ms intra(max)=%.1f ms "
            "rx_slc{wakeups=%u pkts/burst=%.2f} "
            "render{display=%.1ffps Δ%.1f cov=%.1f%% lat=%.1fms queue=%.1fms} tot{present=%u}",
            (unsigned)id,
            (unsigned long long)m.rx_pkts,
            (unsigned long long)m.rx_bytes,
            push_fps, rx_mbps, rx_wire_mbps, m.win_pkt_gap,
            m.frames_push,
            m.build_ms_ewma, lat_ms, m.intra_ms_max,
            m.rx_wakeups, pkts_per_wakeup,
            render_fps, fps_delta, render_cov_pct, render_lat_ms, r_metrics->queue_wait_ms_ewma,
            r_metrics->frames_presented
          );

          // Reset renderer windowed counters
          renderer->reset_windowed_metrics();
        } else
#endif
        {
          // Protocol metrics only (no renderer or metrics disabled)
          ESP_LOGI(TAG,
            "id=%u rx_pkts=%llu rx_B=%llu win{push=%.1ffps rx=%.2fMb/s wire=%.2fMb/s pkt_gap=%u} "
            "tot{push=%u} build(avg)=%.1f ms lat(avg)=%.1f ms intra(max)=%.1f ms "
            "rx_slc{wakeups=%u pkts/burst=%.2f}",
            (unsigned)id,
            (unsigned long long)m.rx_pkts,
            (unsigned long long)m.rx_bytes,
            push_fps, rx_mbps, rx_wire_mbps, m.win_pkt_gap,
            m.frames_push,
            m.build_ms_ewma, lat_ms, m.intra_ms_max,
            m.rx_wakeups, pkts_per_wakeup
          );
        }
      }

      // reset window; keep totals & EWMA
      m.rx_wakeups = 0;
      m.rx_pkts_in_slices = 0;
      m.win_frames_push = 0;
      m.win_rx_bytes = 0;
      m.win_rx_wire_bytes = 0;
      m.win_pkt_gap = 0;
      m.intra_ms_max = 0.0;
      m.log_t0_us = now;
    }
  }
#endif

  // Call loop() on all renderers (for widgets that need main-thread service)
  for (auto& kv : renderers_) {
    for (auto* renderer : kv.second) {
      renderer->loop();
    }
  }

  // Loop optimization: disable when idle for 1 second
  if (had_activity) {
    last_activity_us_ = esp_timer_get_time();
    loop_is_disabled_.store(false, std::memory_order_relaxed);
  } else if (last_activity_us_ != 0) {
    int64_t idle_us = esp_timer_get_time() - last_activity_us_;
    if (idle_us > IDLE_TIMEOUT_US) {
      this->disable_loop();
      loop_is_disabled_.store(true, std::memory_order_release);
      last_activity_us_ = 0;
    }
  }
}

// -------- Socket/RX implementation --------

void DdpComponent::ensure_socket_() {
  bool net_up = network::is_connected();
  if (net_up) {
    // Don't reopen while a previous task is still winding down.
    if (sock_ < 0 && !task_stopping_.load()) open_socket_();
  } else {
    if (sock_ >= 0) close_socket_();
  }
}

void DdpComponent::open_socket_() {
  if (sock_ >= 0) return;

  int s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) { ESP_LOGW(TAG, "socket() failed errno=%d", errno); return; }

  int yes = 1;
  (void) setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port_);
  if (bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) {
    ESP_LOGW(TAG, "bind() failed errno=%d", errno);
    ::close(s);
    return;
  }

  // Larger receive buffer to smooth bursts (kept; harmless for latency)
  int rcv = 64 * 1024; // 64 KB
  (void) setsockopt(s, SOL_SOCKET, SO_RCVBUF, &rcv, sizeof(rcv));

  sock_ = s;

  if (!udp_opened_) {
    ESP_LOGI(TAG, "DDP listening on UDP %u", port_);
    udp_opened_ = true;
  }

  if (task_ == nullptr) {
    task_should_exit_.store(false);
    task_stopping_.store(false);
    // Higher prio + pin to opposite core from typical LVGL usage
    xTaskCreatePinnedToCore(&DdpComponent::recv_task_trampoline, "ddp_rx",
                            8192, this, 6, &task_, 1);
  }
}

void DdpComponent::close_socket_() {
  if (task_ != nullptr) {
    task_stopping_.store(true);
    // Signal task to exit and close socket to break out of select()
    task_should_exit_.store(true);
    if (sock_ >= 0) { ::shutdown(sock_, SHUT_RDWR); ::close(sock_); sock_ = -1; }

    // Wait for task to exit gracefully with timeout
    TaskHandle_t t = task_;

    // Give the task some time to exit cleanly
    for (int i = 0; i < 100; i++) {
      if (eTaskGetState(t) == eDeleted) break;
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Force delete if still running
    if (eTaskGetState(t) != eDeleted) {
      vTaskDelete(t);
    }
    task_ = nullptr;
    task_stopping_.store(false);
  } else if (sock_ >= 0) {
    ::shutdown(sock_, SHUT_RDWR); ::close(sock_); sock_ = -1;
  }

  ESP_LOGI(TAG, "DDP socket closed");
}

void DdpComponent::recv_task_trampoline(void* arg) {
  if (arg == nullptr) return;
  DdpComponent* comp = reinterpret_cast<DdpComponent*>(arg);
  comp->recv_task();
}

void DdpComponent::recv_task() {
  std::vector<uint8_t> buf(2048);

  while (true) {
    if (task_should_exit_.load()) break;
    int s = sock_;
    if (s < 0) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(s, &rfds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500;  // wait ~0.5ms to reduce burst splitting

    int r = lwip_select(s + 1, &rfds, nullptr, nullptr, &tv);
    if (r > 0 && FD_ISSET(s, &rfds)) {
      const int MAX_PKTS_PER_SLICE = 128;
      const int MAX_SLICE_US       = 4500;  // tolerate ragged Wi‑Fi bursts
      int handled = 0;
      int64_t slice_start = esp_timer_get_time();

#if DDP_METRICS
      // Attribute per-slice stats to only the streams that actually received packets
      std::map<uint8_t, uint32_t> slice_counts;
#endif

      while (handled < MAX_PKTS_PER_SLICE) {
        ssize_t n = ::recv(s, buf.data(), buf.size(), MSG_DONTWAIT);
        if (n > 0) {
          this->handle_packet_(buf.data(), (size_t)n);

#if DDP_METRICS
          // Attribute bytes/packet to the stream metrics
          if ((size_t)n >= sizeof(DdpHeader)) {
            const DdpHeader* h = reinterpret_cast<const DdpHeader*>(buf.data());
            uint16_t len = ntohs(h->length_be); // pixel payload
            uint8_t stream_id = h->id;

            auto& m = metrics_[stream_id];
            m.rx_pkts          += 1;
            m.rx_bytes         += (uint64_t)len;
            m.win_rx_bytes     += (uint64_t)len;
            m.rx_wire_bytes    += (uint64_t)n;
            m.win_rx_wire_bytes+= (uint64_t)n;
            slice_counts[stream_id] += 1;
          }
#endif
          // Only break if THIS packet was a PUSH (end of frame).
          if ((size_t)n >= sizeof(DdpHeader)) {
            const DdpHeader* h = reinterpret_cast<const DdpHeader*>(buf.data());
            if ((h->flags & 0x41) == 0x41) { // ver|PUSH
              handled++;
              break;
            }
          }

          handled++;
          if ((esp_timer_get_time() - slice_start) >= MAX_SLICE_US)
            break;
          continue;
        }
        break; // no more queued data immediately
      }

#if DDP_METRICS
      for (auto &kv : slice_counts) {
        uint8_t stream_id = kv.first;
        uint32_t cnt = kv.second;
        auto& m = metrics_[stream_id];
        m.rx_wakeups += 1;
        m.rx_pkts_in_slices += (uint64_t)cnt;
      }
#endif
      taskYIELD();
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
  ESP_LOGI(TAG, "ddp_rx task exiting");
  vTaskDelete(nullptr);
}

// -------- Packet handling --------

std::set<DdpRenderer*>* DdpComponent::find_renderers_(uint8_t stream_id) {
  auto it = renderers_.find(stream_id);
  if (it == renderers_.end()) return nullptr;
  return &it->second;
}

void DdpComponent::handle_packet_(const uint8_t *raw, size_t n) {
  if (n < sizeof(DdpHeader)) return;
  auto *h = reinterpret_cast<const DdpHeader*>(raw);

  bool ver  = (h->flags & 0x40) != 0;
  bool push = (h->flags & 0x01) != 0;
  if (!ver) return;

  uint8_t stream_id = h->id;

  // Check if we have any renderers for this stream
  auto* renderers = find_renderers_(stream_id);
  if (!renderers || renderers->empty()) return;

  uint32_t offset = ntohl(h->offset_be);
  uint16_t len    = ntohs(h->length_be);
  uint8_t  cfg    = h->pixcfg;

  // PUSH-only packet: dispatch whatever we've accumulated
  if (len == 0) {
    if (push) handle_push_(stream_id, h);
    return;
  }

  if (sizeof(DdpHeader) + len > n) return;
  const uint8_t* p = raw + sizeof(DdpHeader);

#if DDP_METRICS
  auto& m = metrics_[stream_id];

  // Start-of-frame metrics init (first data packet in frame)
  if (!m.frame_seen_any) {
    m.frames_started += 1;
    m.frame_first_pkt_us = esp_timer_get_time();
    m.frame_bytes_accum = 0;
    m.frame_px_accum = 0;
    m.frame_seen_any = true;
    m.intra_ms_max = 0.0;
    m.last_pkt_us = 0;
  }
#endif

  // Handle payload by format
  size_t bytes_per_pixel;
  PixelFormat format;

  switch (cfg) {
    case DDP_PIXCFG_RGB888:
    case DDP_PIXCFG_RGB_LEGACY:
      bytes_per_pixel = 3;
      format = PixelFormat::RGB888;
      break;
    case DDP_PIXCFG_RGB565_LE:
      bytes_per_pixel = 2;
      format = PixelFormat::RGB565_LE;
      break;
    case DDP_PIXCFG_RGB565_BE:
      bytes_per_pixel = 2;
      format = PixelFormat::RGB565_BE;
      break;
    case DDP_PIXCFG_RGBW:
      bytes_per_pixel = 4;
      format = PixelFormat::RGBW;
      break;
    default:
      return; // unknown format; ignore (skip metrics for invalid packets)
  }

  handle_pixel_data_(stream_id, h, p, len, bytes_per_pixel, format);

#if DDP_METRICS
  // Track max intra-packet gap (diagnostic for burstiness/Jitter)
  int64_t nowp = esp_timer_get_time();
  if (m.last_pkt_us != 0) {
    double gap_ms = (double)(nowp - m.last_pkt_us) / 1000.0;
    if (gap_ms > m.intra_ms_max) m.intra_ms_max = gap_ms;
  }
  m.last_pkt_us = nowp;
#endif

  // If this packet says PUSH, finalize and dispatch frame
  if (push) {
    handle_push_(stream_id, h);
  }
}

// Handle PUSH flag - signal renderers that frame is complete (UDP TASK CONTEXT)
void DdpComponent::handle_push_(uint8_t stream_id, const DdpHeader* hdr) {
  auto* renderers = find_renderers_(stream_id);
  if (!renderers) return;

#if DDP_METRICS
  auto& m = metrics_[stream_id];
  m.frames_push += 1;
  m.win_frames_push += 1;

  // Build time (first packet -> PUSH)
  if (m.frame_first_pkt_us) {
    int64_t now_us = esp_timer_get_time();
    double build_ms = (double)(now_us - m.frame_first_pkt_us) / 1000.0;
    m.build_ms_ewma = ewma(m.build_ms_ewma, build_ms, 0.2);
  }

  // Dispatch latency (first packet -> push complete)
  if (m.frame_first_pkt_us != 0) {
    int64_t now = esp_timer_get_time();
    double lat_us = (double)(now - m.frame_first_pkt_us);
    m.dispatch_lat_us_ewma = ewma(m.dispatch_lat_us_ewma, lat_us);
  }

#endif

  // Signal all renderers - they will swap buffers and set invalidate flags
  for (auto* renderer : *renderers) {
    renderer->on_push();  // UDP task - just sets atomic flags!
  }

  // Wake loop if it was disabled due to inactivity
  if (loop_is_disabled_.load(std::memory_order_acquire)) {
    this->enable_loop_soon_any_context();
  }

  // Wake main loop immediately for low-latency frame presentation
  App.wake_loop_threadsafe();

#if DDP_METRICS
  // Reset frame assembly state
  m.frame_seen_any = false;
  m.frame_first_pkt_us = 0;
#endif
}

// Generic pixel data handler - all formats (UDP TASK CONTEXT)
void DdpComponent::handle_pixel_data_(uint8_t stream_id, const DdpHeader* hdr,
                                      const uint8_t* p, size_t len,
                                      size_t bytes_per_pixel, PixelFormat format) {
  // Validate alignment
  if ((ntohl(hdr->offset_be) % bytes_per_pixel) != 0 || (len % bytes_per_pixel) != 0) return;

  size_t pixel_offset = ntohl(hdr->offset_be) / bytes_per_pixel;
  size_t pixel_count = len / bytes_per_pixel;

  // Dispatch to all renderers for this stream (streaming!)
  auto* renderers = find_renderers_(stream_id);
  if (!renderers) return;

  for (auto* renderer : *renderers) {
    // Call renderer from UDP task - writes to pre-allocated buffers
    renderer->on_data(pixel_offset, p, format, pixel_count);
  }

#if DDP_METRICS
  auto& m = metrics_[stream_id];
  m.frame_bytes_accum += len;
  m.frame_px_accum += pixel_count;
#endif
}

} // namespace ddp
} // namespace esphome
