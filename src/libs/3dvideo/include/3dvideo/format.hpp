#pragma once


enum class Field
{
    MAGIC = 0x3D4D,
    VERSION = 0x0000,
    METADATA_SECTION = 0x0001,
    FRAME_NUMBER = 0x0002,
};

#define CURRENT_FORMAT_VERSION 1
