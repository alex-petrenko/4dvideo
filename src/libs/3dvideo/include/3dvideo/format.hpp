#pragma once

#include <cstdint>


enum class Field : uint32_t
{
    MAGIC = 0x3D4D,
    VERSION = 0x0010,
    METADATA_SIZE = 0x0020,
    METADATA_SECTION = 0x0030,
    FRAME_SECTION = 0x0040,
    FRAME_NUMBER = 0x0050,
};

enum class Format : uint32_t
{
    CURRENT_VERSION = 1,

    METADATA_SIZE = 1 << 8,
};
