#include "version.h"

/* Written by tools/version.sh before each build */
#if __has_include("build_version.h")
#include "build_version.h"
#else
#define BUILD_HASH  0u
#define BUILD_FLAGS VERSION_FLAG_NONE
#endif

void version_pack(uint8_t *d)
{
    const uint16_t hi = (uint16_t)((uint32_t)BUILD_HASH >> 16);
    const uint16_t lo = (uint16_t)((uint32_t)BUILD_HASH & 0xFFFF);

    d[0] = (uint8_t)(hi & 0xFF);
    d[1] = (uint8_t)(hi >> 8);
    d[2] = (uint8_t)(lo & 0xFF);
    d[3] = (uint8_t)(lo >> 8);
    d[4] = BUILD_FLAGS;
}
