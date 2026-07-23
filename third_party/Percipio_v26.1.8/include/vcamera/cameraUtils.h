#pragma once

#include "cameraApiExport.h"
#include "cameraDefines.h"

#include <string>

namespace vcamera {
	// Forward declaration of implementation class
	class CameraUtilsImpl;

	class CAMERA_API_EXPORT CameraUtils {
	public:
        /**
        * @brief Initialize the camera util class.
        * 
        * @param use_direct_call whether to use direct call mode.
		         true: direct call, suitable for most users.
				 false: IPC call, suitable for advanced users.
        * @return CameraApiStatus indicating success or failure
        */
        static CameraApiStatus Init(bool use_direct_call);

        /**
        * @brief Uninitialize the camera utilities and release resources.
		*        Not calling this function is all right.
        */
        static void Uninit();

		/**
         * @brief Enable or disable Vcamera SDK log.
         *
         * @param log_dest log destination
         * @param enable true to enable, false to disable
         * @return CameraApiStatus indicating success or failure
         */
        static CameraApiStatus EnableVcameraSdkLog(LogDestination log_dest, bool enable);

        /**
        * @brief Set the log level for the Vcamera SDK.
        * 
        * @param level the log level to set
        * @return CameraApiStatus indicating success or failure
        */
		static CameraApiStatus SetVcameraSdkLogLevel(LogLevel level);

        /**
        * @brief Discover all available cameras on the network.
        * 
        * @return std::vector<CameraInfo> list of discovered cameras
        */
        static std::vector<CameraInfo> DiscoverCameras();

		/**
		 * @brief Set camera ip address.
		 * 
		 * \param camera_info camera info of the camera
		 * \param ip new ip address
		 * \param mask new subnet mask
		 * \param gateway new gateway
		 * \return 
		 */
		static CameraApiStatus SetIpAddress(
			const CameraInfo& camera_info,
			const std::string& ip,
			const std::string& mask = "",
			const std::string& gateway = "");

		/**
		 * @brief Set camera ip address.
		 * 
		 * \param mac MAC address of the camera
		 * \param ip new ip address
		 * \param mask new subnet mask
		 * \param gateway new gateway
		 * \return 
		 */
		static CameraApiStatus SetIpAddress(
			const std::string& mac,
			const std::string& ip,
			const std::string& mask = "",
			const std::string& gateway = "");

		/**
		 * Set camera ip address to dynamic.
		 * 
		 * \param camera_info camera info of the camera
		 * \return 
		 */
		static CameraApiStatus SetIpToDynamic(const CameraInfo& camera_info);

		/**
		 * Set camera ip address to dynamic.
		 * 
		 * \param mac MAC address of the camera
		 * \return 
		 */
		static CameraApiStatus SetIpToDynamic(const std::string& mac);

	private:
		CameraUtils() = delete;
		~CameraUtils() = delete;
		CameraUtils(const CameraUtils&) = delete;
		CameraUtils& operator=(const CameraUtils&) = delete;

		static CameraUtilsImpl& GetImpl();
	};
}
