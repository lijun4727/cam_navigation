#pragma once
#include "cameraApiStatus.h"
#include <string>
#include <vector>
#include <functional>

namespace ucv {
class PointCloud;
namespace media {
class FrameSet;
class Frame;
class ICameraCapture;
class Feature;
}  // namespace media
}  // namespace ucv

using UcvFrameSet = ucv::media::FrameSet;
using UcvFrame = ucv::media::Frame;
namespace vcamera {
	namespace SensorType {
        extern CAMERA_API_EXPORT const std::string Depth;
        extern CAMERA_API_EXPORT const std::string Texture;
        extern CAMERA_API_EXPORT const std::string Left;
        extern CAMERA_API_EXPORT const std::string Right;
        extern CAMERA_API_EXPORT const std::string Color; /// @deprecated Use Texture instead.
	}

    enum class LogLevel : int32_t {
        Verbose = 0,
        Debug = 1,
        Info = 2,
        Warn = 3,
        Error = 4,
        Fatal = 5,
    };

    enum class LogDestination : int32_t {
        Console = 0,
        File = 1,
        Server = 2,
    };

    enum class ServerType : int32_t {
        Udp,
        Tcp,
    };

	enum class CAMERA_API_EXPORT CameraState : int32_t {
        NotFound,
		Occupied,
		Opened,
		Closed,
		Capturing,
        Offlined,
        Error,
	};

	enum class CAMERA_API_EXPORT InterfaceType : int32_t {
		Unknown = 0,
		USB = 1,
		Network = 2,
	};

	struct CAMERA_API_EXPORT UsbInfo {
        UsbInfo(){}
        UsbInfo(int32_t bus, int32_t address):
			bus(bus),
			address(address){}
		int32_t bus;
		int32_t address;
	};

	struct CAMERA_API_EXPORT NetworkInfo {
        NetworkInfo(){}
        NetworkInfo(std::string &mac, std::string &ip, std::string &netmask, std::string &gateway, std::string &broadcast):
			mac(mac),
			ip(ip),
			netmask(netmask),
			gateway(gateway),
			broadcast(broadcast){}
		std::string mac;
		std::string ip;
		std::string netmask;
		std::string gateway;
		std::string broadcast;
	};

	struct CAMERA_API_EXPORT InterfaceInfo {
        InterfaceInfo(){};
        InterfaceInfo(InterfaceType interface_type, std::string &name, std::string &id, const NetworkInfo &network_info):
			interface_type(interface_type),
			name(name),
			id(id),
			network_info(network_info){}
		InterfaceType interface_type;
		std::string name;
		std::string id;
		NetworkInfo network_info;
	};

	struct CAMERA_API_EXPORT CameraInfo {
        CameraInfo(){}
        CameraInfo(const InterfaceInfo &interface_info,const NetworkInfo &network_info,const UsbInfo &usb_info,std::string &serial_number,std::string &name,std::string &model,std::string &vendor,std::string &firmware_version):
			interface_info(interface_info),
			network_info(network_info),
			usb_info(usb_info),
			serial_number(serial_number),
			name(name),
			model(model),
			vendor(vendor),
			firmware_version(firmware_version){}
		InterfaceInfo interface_info;
		NetworkInfo network_info;
		UsbInfo usb_info;

		std::string serial_number;
		std::string name;
		std::string model;
		std::string vendor;
		std::string firmware_version;

        CameraState state = CameraState::Closed;
	};

	struct CAMERA_API_EXPORT UserSet {
	public:
        UserSet();
        UserSet(const std::string& sid, const std::string& name, int32_t num_id);
		std::string sid;
		std::string name;
    private:
        int32_t num_id;

        friend class UserSetManager;
	};

	struct CAMERA_API_EXPORT CameraIntrinsic {
        float At(const int32_t row, const int32_t col) const;
        int width;
        int height;
		float data[3 * 3];
	};

	struct CAMERA_API_EXPORT CameraExtrinsic {
        CameraExtrinsic();
        float At(const int32_t row, const int32_t col) const;
		float data[4 * 4];
	};

	struct CAMERA_API_EXPORT CameraDistortion {
        float data[12] = {};  // Zero-initializes all elements
	};

	struct CAMERA_API_EXPORT CameraRotation {
		float data[9];
	};

	struct CAMERA_API_EXPORT CalibInfo {
		CameraIntrinsic intrinsic;
		CameraExtrinsic extrinsic;
		CameraDistortion distortion;
		//enum LensType,
	};

    struct CAMERA_API_EXPORT PointXYZ {
          float x, y, z;
        };

enum class CAMERA_API_EXPORT RawPixelFormat : int32_t {
          Undefined = 0,
          Mono,

          Bayer8GB,
          Bayer8BG,
          Bayer8GR,
          Bayer8RG,

          Bayer8GRBG,
          Bayer8RGGB,
          Bayer8GBRG,
          Bayer8BGGR,

          CSIMono10,

          CSIBayer10GRBG,
          CSIBayer10RGGB,
          CSIBayer10GBRG,
          CSIBayer10BGGR,

          CSIMono12,
          CSIBayer12GRBG,
          CSIBayer12RGGB,
          CSIBayer12GBRG,

          CSIBayer12BGGR,

          Depth16,
          YVYU,
          YUYV,
          Mono16,

          TOFIRMono16,

          RGB,
          BGR,
          JPEG,
          MJPG,

          RGB48,
          BGR48,
          XYZ48,

          // Ty Camera IMU
          IMU,

          // genicam
          Mono8,
          BayerGB8,
          BayerBG8,
          BayerGR8,
          BayerRG8,
          Mono10p,
          CSIMono10P,
          BayerGB10p,
          CSIBayerGB10P,
          BayerBG10p,
          CSIBayerBG10P,
          BayerGR10p,
          CSIBayerGR10P,
          BayerRG10p,
          CSIBayerRG10P,
          Mono12p,
          CSIMono12P,
          BayerGB12p,
          CSIBayerGB12P,
          BayerBG12p,
          CSIBayerBG12P,
          BayerGR12p,
          CSIBayerGR12P,
          BayerRG12p,
          CSIBayerRG12P,
          CSIMono14P,
          CSIBayerGB14P,
          CSIBayerBG14P,
          CSIBayerGR14P,
          CSIBayerRG14P,
          BayerGB16,
          BayerBG16,
          BayerGR16,
          BayerRG16,
          RGB8,
          BGR8,
          YUV422_8,
          YUV422_8_UYVY,
          YCbCr420_8_YY_CbCr_Planar,
          YCbCr420_8_YY_CrCb_Planar,
          YCbCr420_8_YY_CbCr_Semiplanar,
          YCbCr420_8_YY_CrCb_Semiplanar,
          Coord3D_C16,
          Coord3D_ABC16,
          Coord3D_ABC32f,
          TofIR_FourGroup_Mono16,
        };


    enum class CAMERA_API_EXPORT PixelFormat : uint16_t {
        UNDEFINED = 0,      // Unspecified/invalid pixel format (used to indicate unsupported input).
        DEPTH16 = 1,        // Depth image: 16-bit unsigned integer per pixel (multiply depth scale to obtain depth in millimeters).
        DEPTH32F = 2,       // Depth image: 32-bit float per pixel (rarely used, normally don't need care).
        PointCloud = 3,     // Point cloud (reserved for future use).
        GRAY8 = 4,          // Grayscale image: 8-bit unsigned integer per pixel.
        GRAY16 = 5,         // Grayscale image: 16-bit unsigned integer per pixel.
        GRAY32F = 6,        // Grayscale image: 32-bit float per pixel (rarely used, normally don't need care).
        BGR24 = 7,          // Color image: 24-bit BGR (8 bits per channel: B, G, R).
        RGB24 = BGR24,  // Deprecated: use BGR24 instead.
    };

	struct CAMERA_API_EXPORT ImageMode {
		int width;
		int height;
        RawPixelFormat pixel_format;

        ImageMode() : width(0), height(0), pixel_format(RawPixelFormat::Undefined) {}

        ImageMode(int w, int h, RawPixelFormat format) : width(w), height(h), pixel_format(format) {}
	};

	class CAMERA_API_EXPORT Image {
	public:
        Image(UcvFrame* frame=nullptr);
        ~Image();

        Image(const Image& other);
        Image(Image&& other) noexcept;

        Image& operator=(const Image& other);
        Image& operator=(Image&& other) noexcept;
        bool IsValid() const;

        int32_t width() const;
        int32_t height() const;
        PixelFormat pixel_format() const;
		const uint8_t* data() const;

		uint64_t time_stamp() const;

        /**
        * @brief This function is currently an empty implementation and serves no purpose for now; it is merely a placeholder.
        * 
        * \return 
        */
        int64_t frame_index() const;
        std::string sensor() const;
        CalibInfo calib_info() const;
		double scale_unit() const;
    private:
        UcvFrame *frame_;
        friend class FrameSet;
        friend class ImageProc;
	};

	class CAMERA_API_EXPORT FrameSet {
	public:
        Image GetImage(const std::string& sensor_name) const;

        FrameSet(const FrameSet& other);
        FrameSet(FrameSet&& other) noexcept;

        FrameSet& operator=(const FrameSet& other);
        FrameSet& operator=(FrameSet&& other) noexcept;

        ~FrameSet();

	private:
        FrameSet(UcvFrameSet* frame_set_impl);
        UcvFrameSet* frame_set_impl_;

        friend class Camera;
	};

    class CAMERA_API_EXPORT PointCloud {
           public:
            PointCloud(ucv::PointCloud *pc = nullptr);
            PointCloud(const PointCloud &other);
            PointCloud(PointCloud&& other) noexcept;
            ~PointCloud();

            PointCloud& operator=(const PointCloud& other);
            PointCloud& operator=(PointCloud&& other) noexcept;

            std::vector<PointXYZ> GetPoints() const;

            bool hasPositions() const;
            int getPointCount() const;

           private:
            ucv::PointCloud *pointCloud_;
            friend class ImageProc;
        };

	enum class CAMERA_API_EXPORT CameraEventCode : int32_t {
		Closed,
		Opened,
		Started,
		Stopped,
		Offlined,
		Error,
	};
	
	std::string CAMERA_API_EXPORT CameraEventCodeToString(CameraEventCode code);

	using CameraEventCallback = std::function<void(CameraEventCode event_code, int error_code)>;

	enum class CAMERA_API_EXPORT AccessMode : int32_t {
          NotAvailable = 0,
          Readable = 1,
          Writable = 2,
          ReadWritable = 3,
        };

	enum class CAMERA_API_EXPORT FeatureType : int32_t {
        Undefined,
        Bool,
        Int64,
		Float64,
        Enumeration,
		String,
        ByteArray,
        Dictionary,
	};

    enum class CAMERA_API_EXPORT ValueType : int32_t {
        Undefined,

        Bool,

        Int8,
        UInt8,

        Int16,
        UInt16,

        Int32,
        UInt32,

        Int64,
        UInt64,

        Float32,
        Float64,

        String,

        Array,

        Dictionary,
    };

	template <typename T>
    struct NumericRange {
      T minValue;
      T maxValue;
      T step;
    };
	using Int64Range = NumericRange<int64_t>;
    using Float64Range = NumericRange<double>;


	struct CAMERA_API_EXPORT EnumItem {
      std::string name;
      int64_t value;
    };
}
extern template class std::vector<vcamera::ImageMode>;
extern template class std::vector<vcamera::CameraInfo>;
extern template class std::function<void(const vcamera::FrameSet&)>;
extern template class std::vector<vcamera::EnumItem>;