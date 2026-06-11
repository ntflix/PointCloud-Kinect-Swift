#include "kinect2_bridge.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>

static constexpr int kDepthWidth = 512;
static constexpr int kDepthHeight = 424;

struct kinect2_device {
  libfreenect2::Freenect2 ctx;
  libfreenect2::Freenect2Device *dev = nullptr;
  libfreenect2::SyncMultiFrameListener *listener = nullptr;
  libfreenect2::PacketPipeline *pipeline = nullptr;
  libfreenect2::Registration *registration = nullptr;

  // Allocated once, reused each frame
  libfreenect2::Frame *undistorted = nullptr;
  libfreenect2::Frame *registered = nullptr;

  libfreenect2::FrameMap frames;
  bool started = false;
  bool hasFrame = false;
  bool hasApplied =
      false; // tracks whether apply() was called for current frame
};

extern "C" int kinect2_depth_width() { return kDepthWidth; }
extern "C" int kinect2_depth_height() { return kDepthHeight; }

extern "C" kinect2_device_t *kinect2_open_default(void) {
  auto *k = new kinect2_device();

  if (k->ctx.enumerateDevices() == 0) {
    delete k;
    return nullptr;
  }

  std::string serial = k->ctx.getDefaultDeviceSerialNumber();

  const char *envPipeline = std::getenv("LIBFREENECT2_PIPELINE");
  std::string requested = envPipeline ? envPipeline : "";

  auto tryOpen = [&](libfreenect2::PacketPipeline *p) -> bool {
    auto *d = k->ctx.openDevice(serial, p);
    if (d) {
      k->dev = d;
      k->pipeline = p;
      return true;
    }
    delete p;
    return false;
  };

  bool opened = false;
  if (requested == "cpu")
    opened = tryOpen(new libfreenect2::CpuPacketPipeline());
  else if (requested == "cl")
    opened = tryOpen(new libfreenect2::OpenCLPacketPipeline());
  else if (requested == "gl")
    opened = tryOpen(new libfreenect2::OpenGLPacketPipeline());
  else {
    opened = tryOpen(new libfreenect2::CpuPacketPipeline()) ||
             tryOpen(new libfreenect2::OpenCLPacketPipeline()) ||
             tryOpen(new libfreenect2::OpenGLPacketPipeline());
  }

  if (!opened) {
    delete k;
    return nullptr;
  }

  int frameTypes = libfreenect2::Frame::Color | libfreenect2::Frame::Ir |
                   libfreenect2::Frame::Depth;

  k->listener = new libfreenect2::SyncMultiFrameListener(frameTypes);
  k->dev->setColorFrameListener(k->listener);
  k->dev->setIrAndDepthFrameListener(k->listener);

  return k;
}

extern "C" int kinect2_start(kinect2_device_t *k) {
  if (!k || !k->dev || k->started)
    return k ? 0 : -1;

  if (!k->dev->start())
    return -1;
  k->started = true;

  // Must be called AFTER start() - documented requirement
  k->registration = new libfreenect2::Registration(
      k->dev->getIrCameraParams(), k->dev->getColorCameraParams());

  k->undistorted = new libfreenect2::Frame(kDepthWidth, kDepthHeight, 4);
  k->registered = new libfreenect2::Frame(kDepthWidth, kDepthHeight, 4);

  return 0;
}

extern "C" void kinect2_stop(kinect2_device_t *k) {
  if (!k)
    return;
  if (k->hasFrame && k->listener) {
    k->listener->release(k->frames);
    k->hasFrame = false;
    k->hasApplied = false;
  }
  if (k->started && k->dev) {
    k->dev->stop();
    k->started = false;
  }
}

extern "C" void kinect2_close(kinect2_device_t *k) {
  if (!k)
    return;
  kinect2_stop(k);
  if (k->dev) {
    k->dev->close();
    k->dev = nullptr;
  }
  delete k->undistorted;
  delete k->registered;
  delete k->registration;
  delete k->listener;
  delete k->pipeline;
  delete k;
}

extern "C" int kinect2_wait_frame(kinect2_device_t *k, int timeout_ms) {
  if (!k || !k->listener || !k->started)
    return -1;

  if (k->hasFrame) {
    k->listener->release(k->frames);
    k->hasFrame = false;
    k->hasApplied = false;
  }

  if (!k->listener->waitForNewFrame(k->frames, timeout_ms))
    return -1;

  k->hasFrame = true;
  k->hasApplied = false;
  return 0;
}

// ─── Internal helper: call apply() exactly once per frame ─────────────────
static bool ensure_applied(kinect2_device_t *k) {
  if (!k || !k->hasFrame || !k->registration)
    return false;
  if (k->hasApplied)
    return true;

  auto rgbIt = k->frames.find(libfreenect2::Frame::Color);
  auto depthIt = k->frames.find(libfreenect2::Frame::Depth);
  if (rgbIt == k->frames.end() || depthIt == k->frames.end())
    return false;

  libfreenect2::Frame *rgb = rgbIt->second;
  libfreenect2::Frame *depth = depthIt->second;

  // apply() - enable_filter=true by default filters pixels not visible
  // to both cameras. This is correct and expected behaviour.
  k->registration->apply(rgb, depth, k->undistorted, k->registered);

  k->hasApplied = true;
  return true;
}

// ─── Point cloud ──────────────────────────────────────────────────────────
extern "C" size_t kinect2_get_point_cloud(kinect2_device_t *k,
                                          kinect2_point_t *out,
                                          size_t max_points,
                                          int include_invalid) {
  if (!k || !out || max_points == 0)
    return 0;
  if (!ensure_applied(k))
    return 0;

  const uint8_t *reg = k->registered->data;
  size_t n = 0;

  for (int r = 0; r < kDepthHeight && n < max_points; ++r) {
    for (int c = 0; c < kDepthWidth && n < max_points; ++c) {
      float x = 0.0f, y = 0.0f, z = 0.0f;

      // getPointXYZ takes the UNDISTORTED depth frame (output of apply())
      // r = row (y), c = column (x). Returns metres.
      k->registration->getPointXYZ(k->undistorted, r, c, x, y, z);

      bool valid =
          std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && z > 0.0f;

      if (!valid && !include_invalid)
        continue;

      int idx = (r * kDepthWidth + c) * 4;

      kinect2_point_t p{};
      p.x = valid ? x : 0.0f;
      p.y = valid ? y : 0.0f;
      p.z = valid ? z : 0.0f;
      p.b = reg[idx + 0]; // registered frame is BGRX per API docs
      p.g = reg[idx + 1];
      p.r = reg[idx + 2];
      p.a = 255;

      out[n++] = p;
    }
  }
  return n;
}

// ─── Diagnostics ──────────────────────────────────────────────────────────
extern "C" size_t kinect2_count_valid_depth_pixels(kinect2_device_t *k) {
  if (!k || !k->hasFrame)
    return 0;

  // Count valid pixels in the UNDISTORTED frame (post-apply),
  // not the raw depth frame - these differ significantly due to the filter.
  if (!ensure_applied(k))
    return 0;

  const float *data = reinterpret_cast<const float *>(k->undistorted->data);
  size_t valid = 0;
  for (int i = 0; i < kDepthWidth * kDepthHeight; ++i) {
    float z = data[i];
    if (std::isfinite(z) && z > 0.0f)
      ++valid;
  }
  return valid;
}

extern "C" int kinect2_debug_center_xyz(kinect2_device_t *k, float *x, float *y,
                                        float *z) {
  if (!k || !x || !y || !z)
    return -1;
  if (!ensure_applied(k))
    return -1;

  int r = kDepthHeight / 2;
  int c = kDepthWidth / 2;
  k->registration->getPointXYZ(k->undistorted, r, c, *x, *y, *z);
  return 0;
}

extern "C" int kinect2_debug_first_valid_xyz_pixel(kinect2_device_t *k,
                                                   int *row, int *col,
                                                   float *raw_depth_mm,
                                                   float *x, float *y,
                                                   float *z) {
  if (!k || !row || !col || !raw_depth_mm || !x || !y || !z)
    return -1;
  if (!ensure_applied(k))
    return -1;

  auto depthIt = k->frames.find(libfreenect2::Frame::Depth);
  if (depthIt == k->frames.end())
    return -1;

  const float *raw = reinterpret_cast<const float *>(depthIt->second->data);

  for (int r = 8; r < kDepthHeight - 8; ++r) {
    for (int c = 8; c < kDepthWidth - 8; ++c) {
      float rawZ = raw[r * kDepthWidth + c];
      if (!(std::isfinite(rawZ) && rawZ > 0.0f))
        continue;

      float px = 0.0f, py = 0.0f, pz = 0.0f;
      k->registration->getPointXYZ(k->undistorted, r, c, px, py, pz);

      if (std::isfinite(px) && std::isfinite(py) && std::isfinite(pz) &&
          pz > 0.0f) {
        *row = r;
        *col = c;
        *raw_depth_mm = rawZ;
        *x = px;
        *y = py;
        *z = pz;
        return 0;
      }
    }
  }
  return -1;
}