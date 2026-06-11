#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct kinect2_device kinect2_device_t;

typedef struct {
  float x, y, z;
  uint8_t r, g, b, a;
} kinect2_point_t;

kinect2_device_t *kinect2_open_default(void);
void kinect2_close(kinect2_device_t *dev);
int kinect2_start(kinect2_device_t *dev);
void kinect2_stop(kinect2_device_t *dev);
int kinect2_wait_frame(kinect2_device_t *dev, int timeout_ms);
int kinect2_depth_width(void);
int kinect2_depth_height(void);

size_t kinect2_get_point_cloud(kinect2_device_t *dev,
                               kinect2_point_t *out_points, size_t max_points,
                               int include_invalid);

size_t kinect2_count_valid_depth_pixels(kinect2_device_t *dev);
int kinect2_debug_center_xyz(kinect2_device_t *dev, float *x, float *y,
                             float *z);
int kinect2_debug_first_valid_xyz_pixel(kinect2_device_t *dev, int *row,
                                        int *col, float *raw_depth_mm, float *x,
                                        float *y, float *z);

#ifdef __cplusplus
}
#endif