#pragma once

#include "cameraDefines.h"
namespace vcamera {
class CAMERA_API_EXPORT Value {
 public:
  Value();
  Value(const Value&);
  Value(Value&&) noexcept;

  ~Value();

  Value& operator=(const Value&);
  Value& operator=(Value&&) noexcept;

  explicit Value(bool val);

  explicit Value(int8_t value);
  explicit Value(uint8_t val);

  explicit Value(int16_t value);
  explicit Value(uint16_t val);

  explicit Value(int32_t value);
  explicit Value(uint32_t val);

  explicit Value(int64_t value);
  explicit Value(uint64_t val);

  explicit Value(float val);
  explicit Value(double val);

  explicit Value(const std::string& val);

  explicit Value(const std::vector<Value>& values);
  explicit Value(std::vector<Value>&& values);

public:
  ValueType GetType() const noexcept;

  bool IsUndefined() const noexcept;

  bool IsBool() const noexcept;

  bool IsInt8() const noexcept;
  bool IsUInt8() const noexcept;

  bool IsInt16() const noexcept;
  bool IsUInt16() const noexcept;

  bool IsInt32() const noexcept;
  bool IsUInt32() const noexcept;

  bool IsInt64() const noexcept;
  bool IsUInt64() const noexcept;

  bool IsFloat() const noexcept;
  bool IsDouble() const noexcept;

  bool IsString() const noexcept;

  bool IsArray() const noexcept;
  bool IsDictionary() const noexcept;

 public:
  // unsafe convert ,  must sure ValueType is match
  operator bool() const noexcept;
  operator int8_t() const noexcept;
  operator uint8_t() const noexcept;

  operator int16_t() const noexcept;
  operator uint16_t() const noexcept;

  operator int32_t() const noexcept;
  operator uint32_t() const noexcept;

  operator int64_t() const noexcept;
  operator uint64_t() const noexcept;

  operator float() const noexcept;
  operator double() const noexcept;

  operator std::string() const;
  /// @brief Returns the array elements. Only valid when GetType() == ValueType::Array.
  /// @return A copy of the underlying array of elements.
  std::vector<Value> GetArray() const;

  /// @brief Reinterprets the array elements as a flat byte buffer.
  ///
  /// Each element is serialized in native byte order using its raw storage size:
  /// | Element type | Bytes per element |
  /// |--------------|-------------------|
  /// | Int8 / UInt8 | 1                 |
  /// | Int16 / UInt16 | 2               |
  /// | Int32 / UInt32 / Float32 | 4     |
  /// | Int64 / UInt64 / Float64 | 8     |
  ///
  /// @note Only valid when GetType() == ValueType::Array and every element
  ///       is one of the numeric types listed above. Returns an empty vector for
  ///       an empty array or unsupported element types.
  /// @return Byte representation of the array contents.
  std::vector<uint8_t> GetByteArray() const;

 private:
  ValueType type_;

 protected:
  union Data {
    bool bool_value_;
    int8_t int8_value_;
    uint8_t uint8_value_;
    int16_t int16_value_;
    uint16_t uint16_value_;
    int32_t int32_value_;
    uint32_t uint32_value_;
    int64_t int64_value_;
    uint64_t uint64_value_;
    float float32_value_;
    double float64_value_;
  } data_;
  std::string string_value_;
  std::vector<Value> array_value_;
};
}  // namespace vcamera
extern template class std::vector<vcamera::Value>;
extern template class std::vector<std::pair<std::string, vcamera::Value>>;