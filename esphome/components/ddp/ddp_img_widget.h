// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#pragma once

// Only compile if LVGL component is available
#ifdef USE_LVGL

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"
#include <stdint.h>
#include <stdbool.h>

// Forward declarations for C++ types (opaque pointers from C perspective)
#ifdef __cplusplus
namespace esphome { namespace ddp {
    class DdpComponent;
}}
typedef esphome::ddp::DdpComponent DdpComponent;
#else
typedef struct DdpComponent DdpComponent;
#endif

/**********************
 * WIDGET STRUCT
 **********************/

/**
 * DDP Image widget data (C struct, LVGL-compatible)
 * IMPORTANT: lv_obj_t MUST be first member for LVGL compatibility!
 */
typedef struct {
    lv_obj_t obj;              // Base LVGL object (MUST BE FIRST!)

    // DDP configuration
    DdpComponent* ddp_parent;  // Parent DDP component
    uint8_t stream_id;         // DDP stream ID

    // Image dimensions
    int16_t img_w;             // Image width (-1 = auto-detect)
    int16_t img_h;             // Image height (-1 = auto-detect)

    // Buffer configuration
    uint8_t back_buffers;      // 0=none, 1=double, 2=triple

    // Image descriptor (owned by widget)
    lv_img_dsc_t img_dsc;

    // Internal implementation pointer (C++ object)
    void* impl_ptr;            // Opaque pointer to C++ implementation (DdpImgWidgetImpl*)

} lv_ddpimg_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Class descriptor for DDP image widget
 * External linkage - defined in .cpp file
 */
extern const lv_obj_class_t lv_ddpimg_class;

/**
 * Create a DDP image widget
 * @param parent Parent LVGL object
 * @return Created widget (lv_obj_t*)
 */
lv_obj_t* lv_ddpimg_create(lv_obj_t* parent);

/**
 * Configure the widget (called from ESPHome Python codegen)
 */
void lv_ddpimg_set_ddp_parent(lv_obj_t* obj, DdpComponent* parent);
void lv_ddpimg_set_stream_id(lv_obj_t* obj, uint8_t stream_id);
void lv_ddpimg_set_size(lv_obj_t* obj, int16_t w, int16_t h);
void lv_ddpimg_set_back_buffers(lv_obj_t* obj, uint8_t count);

/**
 * Initialize widget (allocate buffers, register with DDP)
 * Called after all configuration is set
 */
void lv_ddpimg_init(lv_obj_t* obj);

#ifdef __cplusplus
} // extern "C"
#endif

#endif  // USE_LVGL
