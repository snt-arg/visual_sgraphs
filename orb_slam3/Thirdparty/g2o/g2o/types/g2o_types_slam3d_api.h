#ifndef G2O_TYPES_SLAM3D_API_H
#define G2O_TYPES_SLAM3D_API_H

#include "../../config.h"

#ifdef _MSC_VER
// We are using a Microsoft compiler:
#ifdef G2O_SHARED_LIBS
#ifdef types_slam3d_EXPORTS
#define G2O_TYPES_SLAM3D_API __declspec(dllexport)
#else
#define G2O_TYPES_SLAM3D_API __declspec(dllimport)
#endif
#else
#define G2O_TYPES_SLAM3D_API
#endif

#else
// Not Microsoft compiler so set empty definition:
#define G2O_TYPES_SLAM3D_API
#endif

#endif // G2O_TYPES_SLAM3D_API_H
