#pragma once
#include "cameraApiExport.h"

#include <cstdint>
#include <string>

namespace vcamera {
enum class CAMERA_API_EXPORT CameraApiStatusCode : int32_t {
  Success,
  Failure,
  ArrayInfoInvalid,
  ArrayInvalid,
  CalibrationInfoInvalid,
  CameraInvalid,
  ComponentInvalid,
  DeviceInvalid,
  DeviceError,
  DeviceIdle,
  DeviceBusy,
  DeviceLost,
  DeviceInterfaceInvalid,
  DeviceInterfaceTypeError,
  DeviceInfoInvalid,
  FeatureInvalid,
  FeatureInfoInvalid,
  FeatureTypeError,
  FrameInvalid,
  FrameMetadataInvalid,
  FrameBufferInvalid,
  FrameBufferConsumerInvalid,
  FrameSetInvalid,
  FrameSetStreamInvalid,
  FrameSetConsumerInvalid,
  TriggerModeError,
  NotExist,
  NotImplemented,
  NotPermitted,
  NotSupported,
  OutOfMemory,
  OutOfIndexRange,
  OutOfValueRange,
  ParameterInvalid,
  StructureInfoInvalid,
  StructureInvalid,
  Timeout,
  ValueInvalid,
  ValueTypeError,
  ValueInfoInvalid,

  NullCameraHandle,
  UserSetIsFull,
  IpcChannelNotInited,
  CameraNotConnected,
};

class CAMERA_API_EXPORT CameraApiStatus {
 public:
  CameraApiStatus() { code_ = CameraApiStatusCode::Success; }
  CameraApiStatus(CameraApiStatusCode code, const std::string& message) : code_(code), message_(message) {}
  bool IsSuccess() { return code_ == CameraApiStatusCode::Success; }
  CameraApiStatusCode code() { return code_; }
  std::string message() { return message_; }

 private:
  CameraApiStatusCode code_;
  std::string message_;
};

#define DECLARE_CAMERA_API_STATUS(name) extern const CameraApiStatus name;

DECLARE_CAMERA_API_STATUS(Success)
DECLARE_CAMERA_API_STATUS(Failure)
DECLARE_CAMERA_API_STATUS(ArrayInfoInvalid)
DECLARE_CAMERA_API_STATUS(ArrayInvalid)
DECLARE_CAMERA_API_STATUS(CalibrationInfoInvalid)
DECLARE_CAMERA_API_STATUS(CameraInvalid)
DECLARE_CAMERA_API_STATUS(ComponentInvalid)
DECLARE_CAMERA_API_STATUS(DeviceInvalid)
DECLARE_CAMERA_API_STATUS(DeviceError)
DECLARE_CAMERA_API_STATUS(DeviceIdle)
DECLARE_CAMERA_API_STATUS(DeviceBusy)
DECLARE_CAMERA_API_STATUS(DeviceLost)
DECLARE_CAMERA_API_STATUS(DeviceInterfaceInvalid)
DECLARE_CAMERA_API_STATUS(DeviceInterfaceTypeError)
DECLARE_CAMERA_API_STATUS(DeviceInfoInvalid)
DECLARE_CAMERA_API_STATUS(FeatureInvalid)
DECLARE_CAMERA_API_STATUS(FeatureInfoInvalid)
DECLARE_CAMERA_API_STATUS(FeatureTypeError)
DECLARE_CAMERA_API_STATUS(FrameInvalid)
DECLARE_CAMERA_API_STATUS(FrameMetadataInvalid)
DECLARE_CAMERA_API_STATUS(FrameBufferInvalid)
DECLARE_CAMERA_API_STATUS(FrameBufferConsumerInvalid)
DECLARE_CAMERA_API_STATUS(FrameSetInvalid)
DECLARE_CAMERA_API_STATUS(FrameSetStreamInvalid)
DECLARE_CAMERA_API_STATUS(FrameSetConsumerInvalid)
DECLARE_CAMERA_API_STATUS(TriggerModeError)
DECLARE_CAMERA_API_STATUS(NotExist)
DECLARE_CAMERA_API_STATUS(NotImplemented)
DECLARE_CAMERA_API_STATUS(NotPermitted)
DECLARE_CAMERA_API_STATUS(NotSupported)
DECLARE_CAMERA_API_STATUS(OutOfMemory)
DECLARE_CAMERA_API_STATUS(OutOfIndexRange)
DECLARE_CAMERA_API_STATUS(OutOfValueRange)
DECLARE_CAMERA_API_STATUS(ParameterInvalid)
DECLARE_CAMERA_API_STATUS(StructureInfoInvalid)
DECLARE_CAMERA_API_STATUS(StructureInvalid)
DECLARE_CAMERA_API_STATUS(Timeout)
DECLARE_CAMERA_API_STATUS(ValueInvalid)
DECLARE_CAMERA_API_STATUS(ValueTypeError)
DECLARE_CAMERA_API_STATUS(ValueInfoInvalid)
DECLARE_CAMERA_API_STATUS(NullCameraHandle)
DECLARE_CAMERA_API_STATUS(UserSetIsFull)
DECLARE_CAMERA_API_STATUS(IpcChannelNotInited)
DECLARE_CAMERA_API_STATUS(CameraNotConnected)

#undef DECLARE_CAMERA_API_STATUS
}  // namespace vcamera
