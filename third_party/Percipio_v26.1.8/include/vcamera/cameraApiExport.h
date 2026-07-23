#pragma once

#ifdef _WIN32
#if defined(CAMERA_API_EXPORTS)
#define CAMERA_API_EXPORT __declspec(dllexport)
#else
#define CAMERA_API_EXPORT __declspec(dllimport)
#endif
#else
#if defined(CAMERA_API_EXPORTS)
#define CAMERA_API_EXPORT __attribute__((visibility("default")))
#else
#define CAMERA_API_EXPORT __attribute__((visibility("default")))
#endif
#endif