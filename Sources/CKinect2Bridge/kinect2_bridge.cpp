#include "kinect2_bridge.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>

struct kinect2_device {
  libfreenect2::Freenect2 ctx;
  libfreenect2::Freenect2Device *dev = nullptr;
  libfreenect2::SyncMultiFrameListener *listener = nullptr;
  libfreenect2::PacketPipeline *pipeline = nullptr;
  libfreenect2::Registration *registration = nullptr;

  libfreenect2::Frame *undistorted = nullptr;
  libfreenect2::Frame *registered = nullptr;

  libfreenect2::FrameMap frames;
  bool started = false;
  bool hasFrame = false;
};

static constexpr int kDepthWidth = 512;
static constexpr int kDepthHeight = 424;

extern "C" int kinect2_depth_width(void) { return kDepthWidth; }

extern "C" int kinect2_depth_height(void) { return kDepthHeight; }

extern "C" kinect2_device_t *kinect2_open_default(void) {
  auto *k = new kinect2_device();

  if (k->ctx.enumerateDevices() == 0) {
    delete k;
    return nullptr;
  }

#if defined(__APPLE__)
  k->pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
  k->pipeline = new libfreenect2::OpenGLPacketPipeline();
#endif

  std::string serial = k->ctx.getDefaultDeviceSerialNumber();
  k->dev = k->ctx.openDevice(serial, k->pipeline);
  if (k->dev == nullptr) {
    delete k->pipeline;
    delete k;
    return nullptr;
  }

  int frameTypes = libfreenect2::Frame::Color | libfreenect2::Frame::Ir |
                   libfreenect2::Frame::Depth;

  k->listener = new libfreenect2::SyncMultiFrameListener(frameTypes);
  k->dev->setColorFrameListener(k->listener);
  k->dev->setIrAndDepthFrameListener(k->listener);

  k->registration = new libfreenect2::Registration(
      k->dev->getIrCameraParams(), k->dev->getColorCameraParams());

  k->undistorted = new libfreenect2::Frame(kDepthWidth, kDepthHeight, 4);
  k->registered = new libfreenect2::Frame(kDepthWidth, kDepthHeight, 4);

  return k;
}

extern "C" int kinect2_start(kinect2_device_t *k) {
  if (k == nullptr || k->dev == nullptr) {
    return -1;
  }

  if (k->started) {
    return 0;
  }

  if (!k->dev->start()) {
    return -1;
  }

  k->started = true;
  return 0;
}

extern "C" void kinect2_stop(kinect2_device_t *k) {
  if (k == nullptr || k->dev == nullptr) {
    return;
  }

  if (k->hasFrame && k->listener != nullptr) {
    k->listener->release(k->frames);
    k->hasFrame = false;
  }

  if (k->started) {
    k->dev->stop();
    k->started = false;
  }
}

extern "C" void kinect2_close(kinect2_device_t *k) {
  if (k == nullptr) {
    return;
  }

  kinect2_stop(k);

  if (k->dev != nullptr) {
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
  if (k == nullptr || k->listener == nullptr || !k->started) {
    return -1;
  }

  if (k->hasFrame) {
    k->listener->release(k->frames);
    k->hasFrame = false;
  }

  if (!k->listener->waitForNewFrame(k->frames, timeout_ms)) {
    return -1;
  }

  k->hasFrame = true;
  return 0;
}

extern "C" size_t kinect2_get_point_cloud(kinect2_device_t *k,
                                          kinect2_point_t *out_points,
                                          size_t max_points,
                                          int include_invalid) {
  if (k == nullptr || out_points == nullptr || max_points == 0 ||
      !k->hasFrame) {
    return 0;
  }

  auto rgbIt = k->frames.find(libfreenect2::Frame::Color);
  auto depthIt = k->frames.find(libfreenect2::Frame::Depth);

  if (rgbIt == k->frames.end() || depthIt == k->frames.end()) {
    return 0;
  }

  libfreenect2::Frame *rgb = rgbIt->second;
  libfreenect2::Frame *depth = depthIt->second;

  k->registration->apply(rgb, depth, k->undistorted, k->registered);

  size_t n = 0;

  for (int r = 0; r < kDepthHeight && n < max_points; ++r) {
    for (int c = 0; c < kDepthWidth && n < max_points; ++c) {
      float x = 0.0f;
      float y = 0.0f;
      float z = 0.0f;
      float packed = 0.0f;

      k->registration->getPointXYZRGB(k->undistorted, k->registered, r, c, x, y,
                                      z, packed);

      bool valid =
          std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && z > 0.0f;
      if (!valid && !include_invalid) {
        continue;
      }

      const uint8_t *p = reinterpret_cast<const uint8_t *>(&packed);

      kinect2_point_t point{};
      point.x = valid ? x : 0.0f;
      point.y = valid ? y : 0.0f;
      point.z = valid ? z : 0.0f;
      point.b = p[0];
      point.g = p[1];
      point.r = p[2];
      point.a = 255;

      out_points[n++] = point;
    }
  }

  return n;
}