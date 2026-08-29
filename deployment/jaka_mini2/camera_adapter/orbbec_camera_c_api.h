#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void* orbbec_dual_camera_create(const char* base_serial, const char* wrist_serial, uint32_t width, uint32_t height,
                                uint32_t fps, uint64_t max_skew_us);
void orbbec_dual_camera_destroy(void* handle);
int orbbec_dual_camera_open(void* handle);
int orbbec_dual_camera_capture(void* handle, uint8_t* base_rgb, size_t base_size, uint8_t* wrist_rgb,
                               size_t wrist_size, uint64_t* timestamp_us, uint64_t* skew_us);
int orbbec_dual_camera_close(void* handle);
const char* orbbec_dual_camera_last_error(void* handle);

#ifdef __cplusplus
}
#endif
