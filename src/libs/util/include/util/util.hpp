#pragma once

#include <util/macro.hpp>


#define ARR_LENGTH(arr) sizeof(arr) / sizeof(arr[0])

template<typename T>
FORCE_INLINE T sqr(T x)
{
    return x * x;
}

void memcpyStride(char *dst, const char *src, int elemSize, int numElements, int ofs, int stride);
