#pragma once

#include "cameraApiExport.h"
#include "cameraApiStatus.h"
#include "cameraDefines.h"
#include "feature.h"
#include "cameraUtils.h"
#include "userSetManager.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace vcamera {

/**
 * @class Camera
 * @brief Main camera control class for the Percipio SDK
 *
 * The Camera class provides a high-level interface for controlling camera devices.
 * It supports camera connection/disconnection, capture control, feature management,
 * sensor configuration, and callback registration for frame and event handling.
 */
class CAMERA_API_EXPORT Camera {
 public:
  /**
   * @brief Default constructor
   * Creates an uninitialized camera object
   */
  Camera();

  /**
   * @brief Copy constructor
   */
  Camera(const Camera& other);

  /*
  * @brief Move constructor
  */
  Camera(Camera&& other) noexcept;

  /**
   * @brief Copy assignment operator
   */
  Camera& operator=(const Camera& other);

  Camera& operator=(Camera&& other) noexcept;

  /**
   * @brief Destructor
   * Cleans up camera resources and disconnects if connected
   */
  ~Camera();

  /**
   * @brief Retrieves camera information
   * @param camera_info Reference to CameraInfo structure to be filled
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus GetCameraInfo(CameraInfo& camera_info) const;

  /**
   * @brief Connects to the camera
   * @param session_key Optional session key for camera access (default: empty)
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus Connect(std::string session_key = "");

  /**
   * @brief Disconnects from the camera
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus Disconnect();

  /**
   * @brief Gets the current camera state
   * @param camera_status Reference to CameraState to be filled with current status
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus GetCameraState(CameraState& camera_status);

  /**
   * @brief Starts image capture
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus StartCapture();

  /**
   * @brief Stops image capture
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus StopCapture();

  /**
   * @brief Retrieves a camera feature by name
   * @param feature_name Name of the feature to retrieve
   * @param feature Reference to Feature object to be filled
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus GetFeature(const std::string& feature_name, Feature& feature);

  /**
   * @brief Retrieves all features from the camera
   * @param features Reference to list of features be filled
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus GetAllFeatures(std::vector<Feature>& features);

  /**
   * @brief Fires a software
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus FireSoftwareTrigger();

  /**
   * @brief Gets available image modes for a specific sensor
   * @param sensor_name Name of the sensor, such as SensorType::DEPTH
   * @param resolutions Vector to be filled with available ImageMode options
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus GetImageModes(std::string sensor_name, std::vector<ImageMode>& resolutions);

  /**
   * @brief Gets the current image mode for a specific sensor
   * @param sensor_name Name of the sensor
   * @param image_mode Reference to ImageMode to be filled with current mode
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus GetCurrentImageMode(std::string sensor_name, ImageMode& image_mode);

  /**
   * @brief Sets the image mode for a specific sensor
   * @param sensor_name Name of the sensor
   * @param mode ImageMode to set
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus SetImageMode(std::string sensor_name, const ImageMode& mode);

  /**
   * @brief Checks if a specific sensor exists on the camera
   * @param sensor_name Name of the sensor to check
   * @param has_sensor Reference to bool that will be set to true if sensor exists
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus HasSensor(std::string sensor_name, bool& has_sensor);

  /**
   * @brief Checks if a specific sensor is enabled
   * @param sensor_name Name of the sensor to check
   * @param is_enabled Reference to bool that will be set to true if sensor is enabled
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus IsSensorEnabled(std::string sensor_name, bool& is_enabled);

  /**
   * @brief Enables or disables a specific sensor
   * @param sensor_name Name of the sensor to configure
   * @param enable True to enable the sensor, false to disable
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus SetSensorEnabled(std::string sensor_name, bool enable);

  /**
   * @brief Enables or disables undistortion for a specific sensor
   * @param sensor_name Name of the sensor to configure
   * @param enable True to enable undistortion, false to disable
   * @return CameraApiStatus indicating success or failure
   */
  CameraApiStatus SetUndistortionEnabled(std::string sensor_name, bool enable);

  CameraApiStatus IsMapDepthToTextureEnabled(bool& enabled);
  CameraApiStatus SetMapDepthToTextureEnabled(bool enable);

  /// @deprecated This function is deprecated, use IsMapDepthToTextureEnabled instead.
  CameraApiStatus IsMapDepthToColorEnabled(bool& enabled);
  /// @deprecated This function is deprecated, use SetMapDepthToTextureEnabled instead.
  CameraApiStatus SetMapDepthToColorEnabled(bool enable);

  /*
   * @brief Saves features to a file
   * @param file_path Path to the file where features will be saved, must use UTF-8 encoding.
   *                  For simplicity, we recommend to use path only including ASCII characters.
   *                  Make sure the folder of the file path exists before calling this function.
   *                  This function won't create any folder.
   * @return CameraApiStatus indicating success or failure
   * usage example: SaveFeaturesToFile(u8"D:/test/1.json");
  */
  CameraApiStatus SaveFeaturesToFile(const std::string& file_path);


  /*
   * @brief Loads camera features from a file
   * @param file_path Path to the file from which features will be loaded, must use UTF-8 encoding.
   *                  For simplicity, we recommend to use path only including ASCII characters.
   *                  Make sure the file exists before calling this function.
   * @param error_message Reference to string that will contain error details if the operation fails
   * @param restore_sensor_enable_state Whether to restore the sensor enable state.
   * @param restore_acquisition_and_trigger Whether to restore the acquisition and trigger related features, such as
   *                                        AcquisitionMode, etc.
   * @return CameraApiStatus indicating success or failure
   * usage example: LoadFeaturesFromFile(u8"D:/test/1.json", error_msg);
  */
  CameraApiStatus LoadFeaturesFromFile(const std::string& file_path, std::string& error_message,
                                       bool restore_sensor_enable_state = true,
                                       bool restore_acquisition_and_trigger = true);

  /*
   * @brief Save features to camera's internal storage. Only cameras *without* userset are supported, for
   * example GM461 does not support this function, but FM815 (non-genicam firmware) supports it.
   */
  CameraApiStatus SaveFeaturesToStorage();

  /*
   * @brief Loads features from camera's internal storage. Make sure you've already saved feature data in
   * the storage before calling this function, otherwise it will fail. Only cameras *without* userset are supported, for
   * example GM461 does not support this function, but FM815 (non-genicam firmware) supports it.
   * @param error_message Reference to string that will contain error details if the operation fails
   * @param restore_sensor_enable_state Whether to restore the sensor enable state.
   * @param restore_acquisition_and_trigger Whether to restore the acquisition and trigger related features, such as
   *                                        AcquisitionMode, etc.
   */
  CameraApiStatus LoadFeaturesFromStorage(std::string& error_message, bool restore_sensor_enable_state = true,
                                          bool restore_acquisition_and_trigger = true);

  /**
   * @brief Gets the user set manager for camera configuration management
   * @return Reference to the UserSetManager instance
   */
  UserSetManager& GetUserSetManager() { return user_set_manager_; }

  /**
   * @brief Callback function type for frame set events
   * Called when a new frame set is available from the camera
   */
  using FrameSetCallback = std::function<void(const FrameSet& frame_set)>;

  /**
   * @brief Registers a callback for frame set events
   * @param callback Function to be called when new frame sets are available
   */
  void RegisterFrameSetCallback(FrameSetCallback callback);

  /**
   * @brief Registers a callback for camera events
   * @param callback Function to be called when camera events occur
   */
  void RegisterCameraEventCallback(CameraEventCallback callback);

 private:
  friend class CameraFactory;
  
  /**
   * @brief Constructor with camera implementation
   * @param camera_impl Pointer to the underlying camera implementation
   */
  Camera(ucv::media::ICameraCapture* camera_impl);

  ucv::media::ICameraCapture* camera_impl_{nullptr};  ///< Pointer to underlying camera implementation
  UserSetManager user_set_manager_;                  ///< Manager for camera user sets and configurations

};

/**
 * @class CameraFactory
 * @brief Factory class for creating Camera instances
 *
 * The CameraFactory provides static methods to create Camera objects using
 * different identification methods such as serial number, IP address, or
 * camera information structure.
 */
class CAMERA_API_EXPORT CameraFactory {
 public:
  /**
   * @brief Creates a Camera instance using serial number
   * @param serial_number The serial number of the camera to connect to
   * @return Camera object configured for the specified device
   */
  static Camera GetCameraBySerialNumber(std::string serial_number);

  /**
   * @brief Creates a Camera instance using IP address
   * @param ip_address The IP address of the camera to connect to
   * @return Camera object configured for the specified device
   */
  static Camera GetCameraByIpAddress(std::string ip_address);

  /**
   * @brief Creates a Camera instance using camera information. Useful when you have multiple network interfaces.
   * @param camera_info CameraInfo that specifies SN + interface IP
   * @return Camera object configured for the specified device
   */
  static Camera GetCameraByCameraInfo(const CameraInfo& camera_info);

  /**
   * @brief Creates a Camera instance using sn, camera ip, and interface ip.
   * 
   * @param serial_number The serial number of the camera to connect to.
   * @param ip_address The IP address of the camera to connect to
   * @param interface_ip_address The IP address of the network interface to use for connecting to the camera. This
   * parameter is optional and can be left empty if not needed.
   * @code
   * // Usage:
   * 
   * Camera cam = GetCamera("207000000001", "");  // Specify SN
   * 
   * Camera cam = GetCamera("", "192.168.1.20");  // Specify camera IP
   * 
   * Camera cam = GetCamera("207000000001", "", "192.168.1.5");  // Specify SN + interface IP
   * 
   * Camera cam = GetCamera("", "192.168.1.20", "192.168.1.5");  // Specify camera IP + interface IP
   * @endcode
   * @return 
   */
  static Camera GetCamera(std::string serial_number, std::string ip_address, std::string interface_ip_address = "");
};

}  // namespace vcamera
