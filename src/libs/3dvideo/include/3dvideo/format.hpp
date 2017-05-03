#pragma once

#include <cstdint>


enum class Field : uint16_t
{
    // high-level markup
    MAGIC = 0x3D4D,
    VERSION = 0x0010,
    METADATA_SECTION = 0x0020,
    FRAME_SECTION = 0x0030,

    // camera, sensor and data format properties, usually same for all dataset frames
    COLOR_RESOLUTION = 0x0100,
    DEPTH_RESOLUTION = 0x0101,
    COLOR_INTRINSICS = 0x0110,
    DEPTH_INTRINSICS = 0x0111,

    COLOR_FORMAT = 0x0120,
    DEPTH_FORMAT = 0x0121,

    // frame data
    FRAME_NUMBER = 0x0f00,
    COLOR = 0x0f10,
    COLOR_TIMESTAMP = 0x0f11,
    DEPTH = 0x0f20,
    DEPTH_TIMESTAMP = 0x0f21,
    CLOUD = 0x0f30,
    CLOUD_NUM_POINTS = 0x0f31,
};

constexpr uint32_t FORMAT_VERSION = 1;

enum class ColorDataFormat : uint8_t
{
    BGR = 0x10,
    YUV_NV21 = 0x20,  // Tango
};

enum class DepthDataFormat : uint8_t
{
    UNSIGNED_16BIT_MM = 0x10,
    CLOUD_FLOAT_METERS = 0xf0,
};
