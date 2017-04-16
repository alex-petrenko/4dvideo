#pragma once

#include <cstdint>


enum class Field : uint16_t
{
    // high-level markup
    MAGIC = 0x3D4D,
    VERSION = 0x0010,
    METADATA_SECTION = 0x0020,
    DATA_SECTION = 0x0030,
    FRAME_SECTION = 0x0040,

    // camera, sensor and data format properties, usually same for all dataset frames
    COLOR_RESOLUTION = 0x0100,
    DEPTH_RESOLUTION = 0x0110,
    
    // frame data
    FRAME_NUMBER = 0x0f00,
    COLOR = 0x0f10,
    DEPTH = 0x0f20,
};

enum class Format : uint32_t
{
    CURRENT_VERSION = 1,
};
