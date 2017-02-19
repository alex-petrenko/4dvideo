#pragma once

#include <algorithm>

#include <util/macro.hpp>


#define ARR_LENGTH(arr) sizeof(arr) / sizeof(arr[0])

template<typename T>
FORCE_INLINE T sqr(T x)
{
    return x * x;
}

template<typename T>
void endianSwap(T *x)
{
    int size = sizeof(*x);
    std::reverse((char *)x, (char *)x + size);
}

void memcpyStride(char *dst, const char *src, int elemSize, int numElements, int ofs, int stride);
