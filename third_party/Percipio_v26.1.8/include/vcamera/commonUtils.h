#pragma once

#include "cameraApiExport.h"
#include "cameraDefines.h"
#include "value.h"

#include <string>

namespace vcamera {
CAMERA_API_EXPORT std::string GetValueString(Value& value);
CAMERA_API_EXPORT std::string GetAccessModeString(AccessMode access_mode);
CAMERA_API_EXPORT std::string GetFeatureTypeString(FeatureType feature_type);
CAMERA_API_EXPORT std::string GetInterfaceTypeString(InterfaceType interface_type);
CAMERA_API_EXPORT std::string GetCameraStateString(CameraState cam_state);
CAMERA_API_EXPORT bool IsAccessModeReadable(AccessMode access_mode);
CAMERA_API_EXPORT bool IsAccessModeWritable(AccessMode access_mode);
CAMERA_API_EXPORT std::string GetRawPixelFormatString(RawPixelFormat raw_pxfmt);
CAMERA_API_EXPORT std::string GetImageModeString(const ImageMode& raw_pxfmt);
CAMERA_API_EXPORT std::string GetCameraIntrinsicString(const CameraIntrinsic& intrinsic);
CAMERA_API_EXPORT std::string GetCameraExtrinsicString(const CameraExtrinsic& extrinsic);
CAMERA_API_EXPORT std::string GetCameraDistortionString(const CameraDistortion& distortion);
CAMERA_API_EXPORT std::string GetCalibInfoString(const CalibInfo& calib_info);

}
