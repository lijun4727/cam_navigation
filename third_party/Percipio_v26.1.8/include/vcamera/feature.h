#pragma once
#include "cameraApiExport.h"
#include "cameraDefines.h"
#include "value.h"

#include <memory>
#include <cstdint>
#include <string>
#include <type_traits>

namespace ucv{
  namespace media {
    class FeatureImpl;
  }
}

namespace vcamera {
class CAMERA_API_EXPORT Feature {
 public:
  Feature() = default;
  Feature(const Feature& other);
  Feature& operator=(const Feature& other);
  ~Feature();

  bool IsValid();
  std::string GetName();
  FeatureType GetType();
  CameraApiStatus GetAccessMode(AccessMode& mode);
  CameraApiStatus GetValue(Value& value);

  /// @brief Set the feature value. The Value type must match the feature type. The type check is strict, no automatic type conversion is performed. For convinience, you can also use the overloaded SetValue functions for basic types, which will perform automatic type conversion when the feature type is compatible.
  /// @param value The value to set.
  /// @return CameraApiStatus
  CameraApiStatus SetValue(const Value& value);

  /// @brief Set a Bool feature value.
  /// @param value The bool value to set.
  /// @return CameraApiStatus
  CameraApiStatus SetValue(bool value);

  /// @brief Set an Int64 or Enumeration feature value.
  ///        If the feature type is Float64, the value is automatically converted to double.
  /// @param value The value to set.
  /// @return CameraApiStatus
  CameraApiStatus SetValue(int64_t value);
  /// @brief Set an Int64 or Enumeration feature value with an int literal.
  ///        Delegates to SetValue(int64_t).
  /// @param value The int value to set.
  /// @return CameraApiStatus
  CameraApiStatus SetValue(int value);

  /// @brief Set a Float64 feature value.
  ///        If the feature type is Int64 or Enumeration, the value is automatically converted to int64_t.
  /// @param value The value to set.
  /// @return CameraApiStatus
  CameraApiStatus SetValue(double value);
  CameraApiStatus GetRange(Int64Range& range);
  CameraApiStatus GetRange(Float64Range& range); 
  CameraApiStatus GetEnumItems(std::vector<EnumItem>& enum_items);

 private:
  Feature(ucv::media::FeatureImpl* feature_impl);
  ucv::media::FeatureImpl* impl_ = nullptr;

  friend class Camera;
};
}  // namespace vcamera
extern template class std::vector<vcamera::Feature>;