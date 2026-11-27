#pragma once

#ifdef __cplusplus
extern "C" {

// make global allocation a compile error
inline void* operator new(std::size_t) = delete;
inline void* operator new[](std::size_t) = delete;
inline void operator delete(void*) noexcept = delete;
inline void operator delete[](void*) noexcept = delete;
#endif

// C++ entry point called from main.c
void go();

#ifdef __cplusplus
}
#endif