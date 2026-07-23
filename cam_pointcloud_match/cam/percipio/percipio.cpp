#include "percipio.h"

#include "common/common_kit.h"
#include "common/math/opencv_kit.h"

#include <vcamera/camera.h>

#include <atomic>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace {

using namespace vcamera;

// 将 Percipio SDK 的 Image 转成 OpenCV Mat。
// 这里统一做深拷贝，避免回调返回后 SDK 内部缓冲区失效导致悬空引用。
Mat ImageToMat(const Image& image)
{
    if (!image.IsValid()) {
        return {};
    }

    const auto width = image.width();
    const auto height = image.height();
    const auto* data = image.data();
    if (!data || width <= 0 || height <= 0) {
        return {};
    }

    switch (image.pixel_format()) {
    case PixelFormat::DEPTH16:
        return cv::Mat(height, width, CV_16UC1, const_cast<uint8_t*>(data)).clone();
    case PixelFormat::DEPTH32F:
        return cv::Mat(height, width, CV_32FC1, const_cast<uint8_t*>(data)).clone();
    case PixelFormat::GRAY8:
        return cv::Mat(height, width, CV_8UC1, const_cast<uint8_t*>(data)).clone();
    case PixelFormat::GRAY16:
        return cv::Mat(height, width, CV_16UC1, const_cast<uint8_t*>(data)).clone();
    case PixelFormat::GRAY32F:
        return cv::Mat(height, width, CV_32FC1, const_cast<uint8_t*>(data)).clone();
    case PixelFormat::RGB24: {
        return cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(data)).clone();
    }
    default:
        return {};
    }
}

// 将 SDK 提供的 3x3 相机内参转成 OpenCV 使用的 cv::Mat。
// 该函数主要用于双目三角化路径中的 OpenCV 接口调用。
cv::Mat IntrinsicToCvMat(const CameraIntrinsic& intrinsic)
{
    cv::Mat K(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            K.at<double>(r, c) = static_cast<double>(intrinsic.At(r, c));
        }
    }
    return K;
}

// 将 SDK 提供的畸变参数转成 OpenCV 风格的畸变向量。
// 保留 SDK 提供的全部 12 个参数，避免双目去畸变时丢失高阶项。
cv::Mat DistortionToCvMat(const CameraDistortion& distortion)
{
    cv::Mat D = cv::Mat::zeros(1, 12, CV_64F);
    for (int i = 0; i < 12; ++i) {
        D.at<double>(0, i) = static_cast<double>(distortion.data[i]);
    }
    return D;
}

// 将 SDK 的 4x4 外参矩阵转成 OpenCV 的 cv::Mat。
// 该函数主要用于 BackProject3DFrom2D 中的坐标系变换推导。
cv::Mat ExtrinsicToCvMat(const CameraExtrinsic& extrinsic)
{
    cv::Mat T(4, 4, CV_64F);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            T.at<double>(r, c) = static_cast<double>(extrinsic.At(r, c));
        }
    }
    return T;
}

// 从 4x4 齐次变换矩阵中拆出旋转矩阵 R 和平移向量 t。
// TriangulatePointToCamera 需要这种分离后的输入格式。
void RtFromTransform(const cv::Mat& T, cv::Mat& R, cv::Mat& t)
{
    R = T(cv::Rect(0, 0, 3, 3)).clone();
    t = T(cv::Rect(3, 0, 1, 3)).clone();
}

// 将 SDK 的 4x4 外参直接转成轻量的 cv::Matx44d。
// 与 cv::Mat 相比，Matx 在频繁逐点变换时更轻量，避免不必要的堆分配。
cv::Matx44d ExtrinsicToMatx44d(const CameraExtrinsic& extrinsic)
{
    return cv::Matx44d(
        extrinsic.data[0], extrinsic.data[1], extrinsic.data[2], extrinsic.data[3],
        extrinsic.data[4], extrinsic.data[5], extrinsic.data[6], extrinsic.data[7],
        extrinsic.data[8], extrinsic.data[9], extrinsic.data[10], extrinsic.data[11],
        extrinsic.data[12], extrinsic.data[13], extrinsic.data[14], extrinsic.data[15]);
}

// 使用 4x4 外参矩阵将一个 3D 点从源坐标系变换到目标坐标系。
// 这里输入的点通常来自深度相机坐标系，输出目标多为彩色相机坐标系。
Point3D TransformPoint(const cv::Matx44d& T, const vcamera::PointXYZ& pt)
{
    const double x = static_cast<double>(pt.x);
    const double y = static_cast<double>(pt.y);
    const double z = static_cast<double>(pt.z);
    return Point3D(
        T(0, 0) * x + T(0, 1) * y + T(0, 2) * z + T(0, 3),
        T(1, 0) * x + T(1, 1) * y + T(1, 2) * z + T(1, 3),
        T(2, 0) * x + T(2, 1) * y + T(2, 2) * z + T(2, 3));
}

struct ProjectionParams {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double k1 = 0.0;
    double k2 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    double k3 = 0.0;
};

// 预提取彩色相机投影所需参数。
// 在 GetPointCloud 的主循环外只构造一次，减少逐点访问结构体数组带来的开销。
ProjectionParams BuildProjectionParams(const vcamera::CalibInfo& calib)
{
    ProjectionParams params;
    params.fx = static_cast<double>(calib.intrinsic.data[0]);
    params.fy = static_cast<double>(calib.intrinsic.data[4]);
    params.cx = static_cast<double>(calib.intrinsic.data[2]);
    params.cy = static_cast<double>(calib.intrinsic.data[5]);
    params.k1 = static_cast<double>(calib.distortion.data[0]);
    params.k2 = static_cast<double>(calib.distortion.data[1]);
    params.p1 = static_cast<double>(calib.distortion.data[2]);
    params.p2 = static_cast<double>(calib.distortion.data[3]);
    params.k3 = static_cast<double>(calib.distortion.data[4]);
    return params;
}

// 手写实现 3D 点到像素点的投影过程。
// 这里使用 Brown-Conrady 畸变模型，避免逐点调用 cv::projectPoints 带来的额外开销。
// 输入点必须已经位于彩色相机坐标系下。
bool ProjectPointToPixel(const ProjectionParams& params, const Point3D& point, cv::Point2d* pixel)
{
    if (!pixel || !IsValidPoint3D(point) || point.z() <= 0.0 ||
        params.fx == 0.0 || params.fy == 0.0) {
        return false;
    }

    const double xn = point.x() / point.z();
    const double yn = point.y() / point.z();
    const double r2 = xn * xn + yn * yn;
    const double radial = 1.0 + params.k1 * r2 + params.k2 * r2 * r2 + params.k3 * r2 * r2 * r2;
    const double xy2 = 2.0 * xn * yn;
    const double x2 = xn * xn;
    const double y2 = yn * yn;

    const double xd = xn * radial + params.p1 * xy2 + params.p2 * (r2 + 2.0 * x2);
    const double yd = yn * radial + params.p1 * (r2 + 2.0 * y2) + params.p2 * xy2;

    pixel->x = params.fx * xd + params.cx;
    pixel->y = params.fy * yd + params.cy;

    if (!std::isfinite(pixel->x) || !std::isfinite(pixel->y)) {
        return false;
    }
    return true;
}

// 根据畸变系数粗略判断畸变模型。
// 当前工程里 TriangulatePointToCamera 只需要区分“无畸变”和“Brown-Conrady”。
int DistortionModelFromCalib(const CalibInfo& calib)
{
    for (int i = 0; i < 12; ++i) {
        if (std::abs(calib.distortion.data[i]) > 1e-12f) {
            return DISTORTION_BROWN_CONRADY;
        }
    }
    return DISTORTION_NONE;
}

// 读取指定 feature 并写入布尔值。
// SDK 的 feature 是按名称查找的，因此这里封装成统一的辅助函数。
bool SetFeatureValue(Camera& camera, const std::string& name, bool value)
{
    Feature feature;
    auto status = camera.GetFeature(name, feature);
    if (!status.IsSuccess() || !feature.IsValid()) {
        return false;
    }
    return feature.SetValue(value).IsSuccess();
}

// 读取指定 feature 并写入浮点值。
bool SetFeatureValue(Camera& camera, const std::string& name, double value)
{
    Feature feature;
    auto status = camera.GetFeature(name, feature);
    if (!status.IsSuccess() || !feature.IsValid()) {
        return false;
    }
    return feature.SetValue(value).IsSuccess();
}

// 读取指定 feature 并写入整型值。
bool SetFeatureValue(Camera& camera, const std::string& name, int value)
{
    Feature feature;
    auto status = camera.GetFeature(name, feature);
    if (!status.IsSuccess() || !feature.IsValid()) {
        return false;
    }
    return feature.SetValue(value).IsSuccess();
}

// 将 SDK 的通用 Value 尽量转换成数值类型。
// 该函数主要用于轮询 feature 是否已经被相机真正应用。
std::optional<double> GetFeatureNumericValue(Camera& camera, const std::string& name)
{
    Feature feature;
    auto status = camera.GetFeature(name, feature);
    if (!status.IsSuccess() || !feature.IsValid()) {
        return std::nullopt;
    }

    Value value;
    status = feature.GetValue(value);
    if (!status.IsSuccess()) {
        return std::nullopt;
    }

    if (value.IsDouble()) return static_cast<double>(value);
    if (value.IsFloat()) return static_cast<double>(value);
    if (value.IsInt64()) return static_cast<double>(static_cast<int64_t>(value));
    if (value.IsUInt64()) return static_cast<double>(static_cast<uint64_t>(value));
    if (value.IsInt32()) return static_cast<double>(static_cast<int32_t>(value));
    if (value.IsUInt32()) return static_cast<double>(static_cast<uint32_t>(value));
    if (value.IsInt16()) return static_cast<double>(static_cast<int16_t>(value));
    if (value.IsUInt16()) return static_cast<double>(static_cast<uint16_t>(value));
    if (value.IsInt8()) return static_cast<double>(static_cast<int8_t>(value));
    if (value.IsUInt8()) return static_cast<double>(static_cast<uint8_t>(value));
    if (value.IsBool()) return static_cast<bool>(value) ? 1.0 : 0.0;
    return std::nullopt;
}

// 如果设备存在对应 sensor，则设置其启用/禁用状态。
// 某些相机型号不一定具备所有 sensor，因此这里先查询再设置。
bool SetSensorEnabledIfPresent(Camera& camera, const std::string& sensor, bool enabled)
{
    bool hasSensor = false;
    auto status = camera.HasSensor(sensor, hasSensor);
    if (!status.IsSuccess() || !hasSensor) {
        return false;
    }
    return camera.SetSensorEnabled(sensor, enabled).IsSuccess();
}

// 根据项目传入的流配置，开启需要的深度/彩色/左右红外流。
void EnableRequestedSensors(Camera& camera, const CameConfig& config)
{
    auto need = [&](STREAM_TYPE type) {
        return (config.stremType & static_cast<int>(type)) != 0;
    };

    SetSensorEnabledIfPresent(camera, SensorType::Depth, need(STREAM_TYPE::DEPTH));
    SetSensorEnabledIfPresent(camera, SensorType::Texture, need(STREAM_TYPE::VIDEO));
    SetSensorEnabledIfPresent(camera, SensorType::Left, need(STREAM_TYPE::IR) || need(STREAM_TYPE::LEFT_IR));
    SetSensorEnabledIfPresent(camera, SensorType::Right, need(STREAM_TYPE::RIGHT_IR));
}

// 浮点容差比较，用于 feature 生效判断。
bool IsCloseTo(double lhs, double rhs, double eps = 1e-3)
{
    return std::abs(lhs - rhs) <= eps;
}

// 轮询等待某个数值型 feature 逼近期望值。
// 某些设置下发后不会立刻在设备端生效，因此需要短时间轮询确认。
bool WaitForFeatureNumericValue(Camera& camera, const std::string& name, double expectedValue, int msTimeout, double eps = 1e-3)
{
    auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(msTimeout);
    const auto pollInterval = std::chrono::milliseconds(100);
    while (std::chrono::steady_clock::now() - start <= timeout) {
        auto value = GetFeatureNumericValue(camera, name);
        if (value && IsCloseTo(*value, expectedValue, eps)) {
            return true;
        }
        std::this_thread::sleep_for(pollInterval);
    }
    return false;
}

} // namespace

// PercipioCam 的内部实现体。
// 这里集中保存 SDK 相机对象、最新帧缓存以及标定信息，避免在头文件暴露 SDK 细节。
struct PercipioCam::Impl {
    vcamera::Camera camera;
    mutable std::mutex mutex;
    std::condition_variable frameCv;
    std::atomic_bool connected{ false };
    uint64_t frameSeq = 0;

    Mat colorFrame;
    Mat depthFrame;
    Mat leftFrame;
    Mat rightFrame;

    //畸变映射表
    cv::Mat m_distMap1;
    cv::Mat m_distMap2;

    vcamera::CalibInfo colorCalib{};
    vcamera::CalibInfo depthCalib{};
    vcamera::CalibInfo leftCalib{};
    vcamera::CalibInfo rightCalib{};
    bool hasColorCalib = false;
    bool hasDepthCalib = false;
    bool hasLeftCalib = false;
    bool hasRightCalib = false;

    double depthScaleUnit = 1.0;
};

PercipioCam::~PercipioCam()
{
    Close();
}

// 打开 Percipio 相机并启动连续采集。
// 设计上这里不做点云生成，只做：
// 1. SDK 初始化与设备连接
// 2. 开启需要的流
// 3. 注册回调并缓存最新图像与标定
bool PercipioCam::Open(const CameConfig& camConfig)
{
    Close();
    IDepthCamera::Open(camConfig);
    impl_ = std::make_shared<Impl>();

    // 统一失败清理逻辑，确保任何阶段失败都能回收 SDK 资源。
    auto failOpen = [&]() {
        try {
            if (impl_ && impl_->connected) {
                impl_->camera.StopCapture();
                impl_->camera.Disconnect();
            }
        }
        catch (...) {
        }
        impl_.reset();
        vcamera::CameraUtils::Uninit();
        return false;
    };

    try {
        auto status = vcamera::CameraUtils::Init(true);
        if (!status.IsSuccess()) {
            return failOpen();
        }

        auto devices = vcamera::CameraUtils::DiscoverCameras();
        if (devices.empty()) {
            return failOpen();
        }
    
        impl_->camera = vcamera::CameraFactory::GetCameraByCameraInfo(devices[0]);
        status = impl_->camera.Connect();
        if (!status.IsSuccess()) {
            return failOpen();
        }

        impl_->connected = true;
        impl_->camera.StopCapture();

        //std::string error_message;
        //status = impl_->camera.LoadFeaturesFromFile("D:\\percipio.json", error_message);

        EnableRequestedSensors(impl_->camera, camConfig);

        // 使用 shared_ptr 副本注册回调，避免外部 Close 时 impl_ 被 reset 造成悬空访问。
		auto impl = impl_;
		impl_->camera.RegisterFrameSetCallback([impl, camConfig, this]
		(const vcamera::FrameSet& frameset)
			{
				vcamera::Image depthImage = frameset.GetImage(vcamera::SensorType::Depth);
				vcamera::Image colorImage = frameset.GetImage(vcamera::SensorType::Texture);
				vcamera::Image leftImage = frameset.GetImage(vcamera::SensorType::Left);
				vcamera::Image rightImage = frameset.GetImage(vcamera::SensorType::Right);

				// 这里仅缓存“最新一帧”的原始图像和标定信息。
				// 刻意不在回调线程里做点云生成，避免高频回调占满 CPU。
				Mat depthMat = ImageToMat(depthImage);
				Mat colorMat = ImageToMat(colorImage);
				Mat leftMat = ImageToMat(leftImage);
				Mat rightMat = ImageToMat(rightImage);

				{
                    auto hasDistMat = this->initDistMap();
					// 只在短临界区内替换缓存，尽量缩短锁持有时间。
					std::lock_guard<std::mutex> lock(impl->mutex);
					if (!colorMat.empty()) {
						if (hasDistMat)
							cv::remap(colorMat, impl->colorFrame, impl->m_distMap1, impl->m_distMap2, cv::INTER_LINEAR);
						else
							impl->colorFrame = std::move(colorMat);
						impl->colorCalib = colorImage.calib_info();
						impl->hasColorCalib = true;
					}
					if (!depthMat.empty()) {
						impl->depthFrame = std::move(depthMat);
						impl->depthCalib = depthImage.calib_info();
						impl->hasDepthCalib = true;
						impl->depthScaleUnit = depthImage.scale_unit();
					}
					if (!leftMat.empty()) {
						impl->leftFrame = leftMat;
						impl->leftCalib = leftImage.calib_info();
						impl->hasLeftCalib = true;
					}
					if (!rightMat.empty()) {
						impl->rightFrame = std::move(rightMat);
						impl->rightCalib = rightImage.calib_info();
						impl->hasRightCalib = true;
					}

					if (!depthImage.IsValid() && !colorImage.IsValid() && !leftImage.IsValid() && !rightImage.IsValid()) {
						return;
					}

					if (((camConfig.stremType & (int)STREAM_TYPE::DEPTH) && !impl->hasDepthCalib) ||
						((camConfig.stremType & (int)STREAM_TYPE::VIDEO) && !impl->hasColorCalib) ||
						((camConfig.stremType & (int)STREAM_TYPE::LEFT_IR) && !impl->hasLeftCalib) ||
						((camConfig.stremType & (int)STREAM_TYPE::RIGHT_IR) && !impl->hasRightCalib))
					{
						return;
					}

					++impl->frameSeq;
				}

				// 有新帧到达后通知 Snapshot 等待线程。
				impl->frameCv.notify_all();
			});

        m_camName = devices[0].model;

        // 0: SingleFrame
        // 1: MultiFrame
        // 2: Continuous
        SetFeatureValue(impl->camera, "AcquisitionMode", 0);    
        // 0: harware line 0
        // 8: SoftTrigger
        SetFeatureValue(impl->camera, "TriggerSource", 8);
        SetFeatureValue(impl->camera, "DeviceStreamAsyncMode", 0);
        //SetFeatureValue(impl->camera, "DeviceDataCompressType", 2);
        SetFeatureValue(impl->camera, "DeviceLinkHeartbeatTimeout", 1000 * 10);
        //impl->camera.SetUndistortionEnabled(SensorType::Texture, true);

        //ROI相关
        //auto ret = SetFeatureValue(impl->camera, "Left/RegionSelector", 0);
        //ret = SetFeatureValue(impl->camera, "Left/OffsetX", 320);
        //ret = SetFeatureValue(impl->camera, "Left/OffsetY", 240);
        //ret = SetFeatureValue(impl->camera, "Left/Width", 640);
        //ret = SetFeatureValue(impl->camera, "Left/Height", 480);
        //ret = SetFeatureValue(impl->camera, "Left/RegionMode", 1);

        //ret = SetFeatureValue(impl->camera, "Right/RegionSelector", 0);
        //ret = SetFeatureValue(impl->camera, "Right/OffsetX", 320);
        //ret = SetFeatureValue(impl->camera, "Right/OffsetY", 240);
        //ret = SetFeatureValue(impl->camera, "Right/Width", 640);
        //ret = SetFeatureValue(impl->camera, "Right/Height", 480);
        //ret = SetFeatureValue(impl->camera, "Right/RegionMode", 1);

        //ret = SetFeatureValue(impl->camera, "Texture/RegionSelector", 0);
        //ret = SetFeatureValue(impl->camera, "Texture/OffsetX", 320);
        //ret = SetFeatureValue(impl->camera, "Texture/OffsetY", 240);
        //ret = SetFeatureValue(impl->camera, "Texture/Width", 640);
        //ret = SetFeatureValue(impl->camera, "Texture/Height", 480);
        //ret = SetFeatureValue(impl->camera, "Texture/RegionMode", 1);

        if (m_camName.find("PM807-E1") != std::string::npos)
        {
            ImageMode image_mode;
            image_mode.pixel_format = RawPixelFormat::YUV422_8;
            image_mode.width = 1280;
            image_mode.height = 960;
            impl->camera.SetImageMode(SensorType::Texture, image_mode);

            SetFeatureValue(impl->camera, "Texture/ExposureAuto", false);
            SetFeatureValue(impl->camera, "Texture/ExposureTime", 15000);

            SetFeatureValue(impl->camera, "LightController0/LightBrightness", 100);
            SetFeatureValue(impl->camera, "LightController0/LightEnable", !config_.enableFloodLight);
            SetFeatureValue(impl->camera, "LightController1/LightBrightness", 5);
            SetFeatureValue(impl->camera, "LightController1/LightEnable", config_.enableFloodLight);

			SetFeatureValue(impl->camera, "Left/ExposureTime", 33245.0);
			SetFeatureValue(impl->camera, "Right/ExposureTime", 33245.0);
        }
        else if(m_camName.find("FM855-E1") != std::string::npos)
        {
            SetFeatureValue(impl->camera, "Texture/IntExposureTime", 432);
            SetFeatureValue(impl->camera, "Texture/AnalogGain", 1);
            SetFeatureValue(impl->camera, "Texture/RedGain", 23);
            SetFeatureValue(impl->camera, "Texture/GreenGain", 25);
            SetFeatureValue(impl->camera, "Texture/BlueGain", 50);
            SetFeatureValue(impl->camera, "Left/DigitalGain", 32);
            SetFeatureValue(impl->camera, "Left/AnalogGain", 2);
            SetFeatureValue(impl->camera, "Right/DigitalGain", 32);
            SetFeatureValue(impl->camera, "Right/AnalogGain", 2);
            SetFeatureValue(impl->camera, "LightController1/LightBrightness", 100);
            SetFeatureValue(impl->camera, "LightController2/LightBrightness", 100);
            SetFeatureValue(impl->camera, "LightController1/LightEnable", config_.enableFloodLight);
            SetFeatureValue(impl->camera, "LightController2/LightEnable", config_.enableFloodLight);
            SetFeatureValue(impl->camera, "LightController0/LightBrightness", config_.enableFloodLight ? 0 : 100);
        }
        else
        {
            ImageMode image_mode;
            image_mode.pixel_format = RawPixelFormat::YUV422_8;
            image_mode.width = 1280;
            image_mode.height = 960;
            impl->camera.SetImageMode(SensorType::Texture, image_mode);

            SetFeatureValue(impl->camera, "Texture/BalanceWhiteAuto", false);
            SetFeatureValue(impl->camera, "Texture/ExposureAuto", false);
            SetFeatureValue(impl->camera, "Texture/ExposureTime", 15000);

            SetFeatureValue(impl->camera, "LightController0/LightBrightness", 100);
            SetFeatureValue(impl->camera, "LightController0/LightEnable", !config_.enableFloodLight);
            SetFeatureValue(impl->camera, "LightController1/LightBrightness", 10);
            SetFeatureValue(impl->camera, "LightController1/LightEnable", config_.enableFloodLight);

            SetFeatureValue(impl->camera, "Left/ExposureTime", 22492.0);
            SetFeatureValue(impl->camera, "Right/ExposureTime", 22492.0);
        }

        //SetFeatureValue(impl->camera, "Left/DigitalGain", 32);
        //SetFeatureValue(impl->camera, "Left/AnalogGain", 2);
        impl->camera.SetUndistortionEnabled(SensorType::Left, true);

        //SetFeatureValue(impl->camera, "Right/DigitalGain", 32);
        //SetFeatureValue(impl->camera, "Right/AnalogGain", 2);
        impl->camera.SetUndistortionEnabled(SensorType::Right, true);

        //SetFeatureValue(impl->camera, "LightController1/LightBrightness", 100);
        //SetFeatureValue(impl->camera, "LightController2/LightBrightness", 100);

        //SetFeatureValue(impl->camera, "LightController1/LightEnable", config_.enableFloodLight);
        //SetFeatureValue(impl->camera, "LightController2/LightEnable", config_.enableFloodLight);

        //auto v = GetFeatureNumericValue(impl->camera, "DeviceStreamAsyncMode");
        //auto b = SetFeatureValue(impl->camera, "DeviceStreamAsyncMode", 2);
        //v = GetFeatureNumericValue(impl->camera, "DeviceStreamAsyncMode");

        //SetFeatureValue(impl->camera, "Left/IntExposureTime", config_.enableFloodLight ? 100 : 997);
        //SetFeatureValue(impl->camera, "Right/IntExposureTime", config_.enableFloodLight ? 100 : 997);

        status = impl_->camera.StartCapture();
        if (!status.IsSuccess()) {
            return failOpen();
        }

        if (!Snapshot())
            return false;

        return true;
    }
    catch (...) {
        return failOpen();
    }
}

// 关闭相机并释放 SDK 资源。
// 这里先把 impl_ 移走，再进行 Stop/Disconnect，减少并发场景下外部重复访问的风险。
void PercipioCam::Close()
{
    IDepthCamera::Close();
    auto impl = std::move(impl_);
    if (!impl) {
        return;
    }

    try {
        impl->connected = false;
        impl->frameCv.notify_all();
        impl->camera.StopCapture();
        impl->camera.Disconnect();
    }
    catch (...) {
    }

    vcamera::CameraUtils::Uninit();
}

// 等待一帧新的缓存图像到达。
// 这里比较的是 frameSeq，而不是简单看 hasFrame，确保每次 Snapshot 拿到的是“新帧”。
bool PercipioCam::Snapshot(int32_t timeout)
{
    auto impl = impl_;
    if (!impl) {
        return false;
    }

    auto status = impl->camera.FireSoftwareTrigger();
    if(!status.IsSuccess()) {
        return false;
	}
    // 等待新帧到达，或设备断开，或外部 Close。
    std::unique_lock<std::mutex> lock(impl->mutex);
    const auto currentSeq = impl->frameSeq;
    auto ret = impl->frameCv.wait_for(
        lock,
        std::chrono::milliseconds(timeout),
        [&]() {
            return exit_ || !impl->connected || impl->frameSeq > currentSeq;
        }) && impl->frameSeq > currentSeq;
    return ret;
}

// 生成点云。
// 整体流程：
// 1. 从缓存中取出最新深度帧、彩色帧和标定
// 2. 逐像素将 depth 反投影为 3D 点（depth 坐标系）
// 3. 如有彩色标定，则显式变换到 color 坐标系
// 4. 将 3D 点投影回彩色图像取色
// 5. 统一转换到项目当前使用的显示坐标系（x/y 取反）
PointCloudPtr PercipioCam::GetPointCloud(float maxZ, float maxX, float maxY)
{
    auto impl = impl_;
    if (!impl) {
        return nullptr;
    }

    cv::Mat depthFrame;
    Mat colorFrame;
    vcamera::CalibInfo depthCalib{};
    vcamera::CalibInfo colorCalib{};
    double depthScaleUnit = 1.0;
    bool hasDepthCalib = false;
    bool hasColorCalib = false;
    {
        std::lock_guard<std::mutex> lock(impl->mutex);
        depthFrame = impl->depthFrame;
        colorFrame = impl->colorFrame;
        depthCalib = impl->depthCalib;
        colorCalib = impl->colorCalib;
        depthScaleUnit = impl->depthScaleUnit;
        hasDepthCalib = impl->hasDepthCalib;
        hasColorCalib = impl->hasColorCalib;
    }

    if (depthFrame.empty() || !hasDepthCalib) {
        return nullptr;
    }

    const double fx = static_cast<double>(depthCalib.intrinsic.data[0]);
    const double fy = static_cast<double>(depthCalib.intrinsic.data[4]);
    const double cx = static_cast<double>(depthCalib.intrinsic.data[2]);
    const double cy = static_cast<double>(depthCalib.intrinsic.data[5]);
    if (fx == 0.0 || fy == 0.0 || depthScaleUnit == 0.0) {
        return nullptr;
    }

    const double invFx = 1.0 / fx;
    const double invFy = 1.0 / fy;

    auto cloud = std::make_shared<::PointCloud>();
    cloud->points_.reserve(static_cast<size_t>(depthFrame.rows) * static_cast<size_t>(depthFrame.cols));
    cloud->colors_.reserve(static_cast<size_t>(depthFrame.rows) * static_cast<size_t>(depthFrame.cols));

    // 若同时具备深度和颜色标定，则显式计算 depth->color 变换。
    const bool hasColor = !colorFrame.empty() && hasColorCalib;
    const bool useColorCoords = hasDepthCalib && hasColorCalib;
    cv::Matx44d depthToColor;
    ProjectionParams projectionParams;
    if (useColorCoords) {
        depthToColor = ExtrinsicToMatx44d(colorCalib.extrinsic).inv() * ExtrinsicToMatx44d(depthCalib.extrinsic);
        projectionParams = BuildProjectionParams(colorCalib);
    }

    // 给当前 3D 点追加颜色。
    // 若无法投影到彩色图，或彩色图缺失，则退化为白色。
    auto appendColor = [&](const Point3D& point) {
        if (!hasColor) {
            cloud->colors_.emplace_back(1.0, 1.0, 1.0);
            return;
        }

        cv::Point2d pixel;
        if (!ProjectPointToPixel(projectionParams, point, &pixel)) {
            cloud->colors_.emplace_back(1.0, 1.0, 1.0);
            return;
        }

        const int col = static_cast<int>(std::lround(pixel.x));
        const int row = static_cast<int>(std::lround(pixel.y));
        if (row < 0 || row >= colorFrame.rows || col < 0 || col >= colorFrame.cols) {
            cloud->colors_.emplace_back(1.0, 1.0, 1.0);
            return;
        }

        if (colorFrame.type() == CV_8UC3) {
            const auto bgr = colorFrame.at<cv::Vec3b>(row, col);
            cloud->colors_.emplace_back(bgr[2] / 255.0, bgr[1] / 255.0, bgr[0] / 255.0);
            return;
        }

        if (colorFrame.type() == CV_8UC1) {
            const auto gray = colorFrame.at<uint8_t>(row, col) / 255.0;
            cloud->colors_.emplace_back(gray, gray, gray);
            return;
        }

        cloud->colors_.emplace_back(1.0, 1.0, 1.0);
    };

    // 深度为 16 位整型时，通常 rawDepth * scaleUnit = 实际深度。
    if (depthFrame.type() == CV_16UC1) {
        for (int v = 0; v < depthFrame.rows; ++v) {
            const auto* row = depthFrame.ptr<uint16_t>(v);
            const double yDepth = (static_cast<double>(v) - cy) * invFy;
            for (int u = 0; u < depthFrame.cols; ++u) {
                const uint16_t rawDepth = row[u];
                if (rawDepth == 0) {
                    continue;
                }

                const double zDepth = static_cast<double>(rawDepth) * depthScaleUnit;
                const vcamera::PointXYZ depthPoint {
                    static_cast<float>((static_cast<double>(u) - cx) * zDepth * invFx),
                    static_cast<float>(yDepth * zDepth),
                    static_cast<float>(zDepth)
                };
                // 若具备彩色标定，则先转到彩色相机坐标系；否则保持深度坐标系。
                const Point3D point = useColorCoords ? TransformPoint(depthToColor, depthPoint)
                    : Point3D(depthPoint.x, depthPoint.y, depthPoint.z);

                // 项目当前约定：为了便于显示，X/Y 取反，Z 保持朝前。
                const double x = -point.x();
                const double y = -point.y();
                const double z = point.z();
                if (z <= 0.0 || z > maxZ || std::abs(x) > maxX || std::abs(y) > maxY) {
                    continue;
                }

                cloud->points_.emplace_back(x, y, z);
                appendColor(point);
            }
        }
    }
    // 深度为 32 位浮点时，处理逻辑相同，只是原始深度读取类型不同。
    else if (depthFrame.type() == CV_32FC1) {
        for (int v = 0; v < depthFrame.rows; ++v) {
            const auto* row = depthFrame.ptr<float>(v);
            const double yDepth = (static_cast<double>(v) - cy) * invFy;
            for (int u = 0; u < depthFrame.cols; ++u) {
                const float rawDepth = row[u];
                if (!std::isfinite(rawDepth) || rawDepth <= 0.0f) {
                    continue;
                }

                const double zDepth = static_cast<double>(rawDepth) * depthScaleUnit;
                const vcamera::PointXYZ depthPoint {
                    static_cast<float>((static_cast<double>(u) - cx) * zDepth * invFx),
                    static_cast<float>(yDepth * zDepth),
                    static_cast<float>(zDepth)
                };
                const Point3D point = useColorCoords ? TransformPoint(depthToColor, depthPoint)
                    : Point3D(depthPoint.x, depthPoint.y, depthPoint.z);

                const double x = -point.x();
                const double y = -point.y();
                const double z = point.z();
                if (z <= 0.0 || z > maxZ || std::abs(x) > maxX || std::abs(y) > maxY) {
                    continue;
                }

                cloud->points_.emplace_back(x, y, z);
                appendColor(point);
            }
        }
    }
    else {
        // 未知深度格式，当前不支持。
        return nullptr;
    }

    return cloud;
}

// 获取缓存图像。
// 返回时统一 clone，避免调用者持有内部缓存的引用并跨线程访问。
void PercipioCam::GetFrame(UNMAP_MAT& frames) const
{
    auto impl = impl_;
    if (!impl) {
        return;
    }

    std::lock_guard<std::mutex> lock(impl->mutex);
    for (auto& it : frames) {
        switch (it.first) {
        case STREAM_TYPE::VIDEO:
            if (!impl->colorFrame.empty()) it.second = impl->colorFrame.clone();
            break;
        case STREAM_TYPE::DEPTH:
            if (!impl->depthFrame.empty()) it.second = impl->depthFrame.clone();
            break;
        case STREAM_TYPE::IR:
        case STREAM_TYPE::LEFT_IR:
            if (!impl->leftFrame.empty()) it.second = impl->leftFrame.clone();
            break;
        case STREAM_TYPE::RIGHT_IR:
            if (!impl->rightFrame.empty()) it.second = impl->rightFrame.clone();
            break;
        default:
            break;
        }
    }
}

// 使用左右红外像素点进行三角化，并将结果变换到目标坐标系。
// 若当前存在彩色标定，则目标坐标系优先选彩色相机；否则退回深度相机。
Point3D PercipioCam::BackProject3DFrom2D(
    float left_u,
    float left_v,
    float right_u, 
    float right_v,
    const CameraParameter& camParam)
{
    Point3D point = InvalidPoint3D();
    const bool ok = TriangulatePointToCamera(
        cv::Point2f(left_u, left_v),
        cv::Point2f(right_u, right_v),
        camParam,
        &point);
    if (!ok || !IsValidPoint3D(point)) {
        return InvalidPoint3D();
    }

    return Point3D(-point.x(), -point.y(), point.z());
}

// 获取彩色相机内参。
// 若当前没有彩色标定，则退回深度相机内参，保证调用方至少能拿到一套可用参数。
bool PercipioCam::GetColorIntrinsics(CameraIntrinsics& out) const
{
    auto impl = impl_;
    if (!impl) {
        return false;
    }

    std::lock_guard<std::mutex> lock(impl->mutex);
    const vcamera::CalibInfo* calib = nullptr;
    if (impl->hasColorCalib) {
        calib = &impl->colorCalib;
    }
    else if (impl->hasDepthCalib) {
        calib = &impl->depthCalib;
    }

    if (!calib) {
        return false;
    }

    out.width = calib->intrinsic.width;
    out.height = calib->intrinsic.height;
    out.fx = calib->intrinsic.data[0];
    out.fy = calib->intrinsic.data[4];
    out.ppx = calib->intrinsic.data[2];
    out.ppy = calib->intrinsic.data[5];
    return true;
}

// 根据内参计算垂直视场角 VFOV。
float PercipioCam::GetVFOV()
{
    CameraIntrinsics intrinsics;
    if (!GetColorIntrinsics(intrinsics) || intrinsics.fy <= 0.0 || intrinsics.height <= 0) {
        return -1.0f;
    }

    const double vfovRad = 2.0 * std::atan(static_cast<double>(intrinsics.height) / (2.0 * intrinsics.fy));
    return static_cast<float>(vfovRad * 180.0 / std::acos(-1.0));
}

// 设置相机选项。
// 当前封装的两个选项：
// 1. IR_EXPOSURE：左右红外曝光时间
// 2. EMITTER：光源/补光使能
/*
* PM807-E1
AcquisitionMode: Enumeration, ReadWritable
AcquisitionFrameRateEnable: Bool, ReadWritable
AcquisitionFrameRate: Float64, NotAvailable
CaptureTimeStatistic: Int64, Readable
DepthScaleUnit: Float64, ReadWritable
DepthSgbmDisparityNumber: Int64, ReadWritable
DepthSgbmDisparityOffset: Int64, ReadWritable
DepthSgbmHFilterHalfWin: Bool, ReadWritable
DepthSgbmImageNumber: Int64, ReadWritable
DepthSgbmLRC: Bool, ReadWritable
DepthSgbmLRCDiff: Int64, ReadWritable
DepthSgbmMatchWinHeight: Int64, ReadWritable
DepthSgbmMatchWinWidth: Int64, ReadWritable
DepthSgbmMedFilter: Bool, ReadWritable
DepthSgbmMedFilterThresh: Int64, ReadWritable
DepthRangeMin: Int64, ReadWritable
DepthRangeMax: Int64, ReadWritable
DepthSgbmSemiParamP1: Int64, ReadWritable
DepthSgbmSemiParamP1Scale: Int64, ReadWritable
DepthSgbmSemiParamP2: Int64, ReadWritable
DepthSgbmTextureFilterThreshold: Int64, ReadWritable
DepthSgbmTextureFilterValueOffset: Int64, Readable
DepthSgbmUniqueAbsDiff: Int64, ReadWritable
DepthSgbmUniqueFactor: Int64, ReadWritable
DepthSgbmUniqueMaxCost: Int64, ReadWritable
DeviceBuildHash: String, Readable
DeviceCalibrationTime: String, Readable
DeviceCharacterSet: Enumeration, Readable
DeviceConfigVersion: String, Readable
DeviceConnectionSelector: Int64, ReadWritable
DeviceDataCompressType: Enumeration, ReadWritable
DeviceErrCode: Int64, Readable
DeviceFirmwareVersion: String, Readable
DeviceFrameRecvTimeOut: Int64, ReadWritable
DeviceGeneratedTime: String, Readable
DeviceHardWareModel: String, Readable
DeviceHardwareVersion: String, Readable
DeviceLinkHeartbeatMode: Enumeration, ReadWritable
DeviceLinkHeartbeatTimeout: Int64, ReadWritable
DeviceLinkSelector: Int64, ReadWritable
DeviceLinkSpeed: Int64, Readable
DeviceManufacturerInfo: String, Readable
DeviceModelName: String, Readable
DeviceSerialNumber: String, Readable
DeviceStreamAsyncMode: Enumeration, ReadWritable
DeviceStreamChannelCount: Int64, Readable
DeviceStreamChannelLink: Int64, Readable
DeviceStreamChannelPacketSize: Int64, ReadWritable
DeviceStreamChannelSelector: Int64, ReadWritable
DeviceStreamChannelType: Enumeration, Readable
DeviceTLVersionMajor: Int64, Readable
DeviceTLVersionMinor: Int64, Readable
DeviceTechModel: String, Readable
DeviceTimeSyncMode: Enumeration, ReadWritable
DeviceType: Enumeration, Readable
DeviceVendorName: String, Readable
DeviceVersion: String, Readable
GevCCP: Enumeration, ReadWritable
GevCurrentDefaultGateway: Int64, Readable
GevCurrentIPAddress: Int64, Readable
GevCurrentIPConfigurationDHCP: Bool, ReadWritable
GevCurrentIPConfigurationLLA: Bool, Readable
GevCurrentIPConfigurationPersistentIP: Bool, ReadWritable
GevCurrentSubnetMask: Int64, Readable
GevFirstURL: String, Readable
GevGVCPPendingAck: Bool, ReadWritable
GevGVCPPendingTimeout: Int64, Readable
GevInterfaceSelector: Int64, Readable
GevMACAddress: Int64, Readable
GevPersistentDefaultGateway: Int64, ReadWritable
GevPersistentIPAddress: Int64, ReadWritable
GevPersistentSubnetMask: Int64, ReadWritable
GevSCDA: Int64, ReadWritable
GevSCPD: Int64, ReadWritable
GevSCPDirection: Int64, Readable
GevSCPHostPort: Int64, ReadWritable
GevSCPInterfaceIndex: Int64, Readable
GevSCPSPacketSize: Int64, ReadWritable
GevSCSP: Int64, Readable
GevSecondURL: String, Readable
GevStreamChannelSelector: Int64, ReadWritable
IRUndistortion: Bool, ReadWritable
NTPServerIP: Int64, ReadWritable
PayloadSize: Int64, Readable
SourceCount: Int64, Readable
SourceIDValue: Int64, Readable
TLParamsLocked: Int64, ReadWritable
TimeSyncAck: Bool, Readable
TriggerActivation: Enumeration, NotAvailable
TriggerDelay: Float64, NotAvailable
TriggerDurationUs: Int64, ReadWritable
TriggerMode: Enumeration, Readable
TriggerSelector: Enumeration, ReadWritable
TriggerSource: Enumeration, ReadWritable
Depth/ComponentEnable: Bool, ReadWritable
Depth/PixelFormat: Enumeration, ReadWritable
Depth/BinningHorizontal: Enumeration, ReadWritable
Depth/BinningVertical: Enumeration, ReadWritable
Depth/ExposureAuto: Bool, NotAvailable
Depth/ExposureTargetBrightness: Int64, NotAvailable
Depth/ExposureTime: Float64, NotAvailable
Depth/AutoFunctionAOIHeight: Int64, NotAvailable
Depth/AutoFunctionAOIOffsetX: Int64, NotAvailable
Depth/AutoFunctionAOIOffsetY: Int64, NotAvailable
Depth/AutoFunctionAOIWidth: Int64, NotAvailable
Depth/BalanceWhiteAuto: Bool, NotAvailable
Depth/HDRControl: Undefined, NotAvailable
Depth/HDREnable: Bool, NotAvailable
Depth/Intrinsic: Undefined, Readable
Depth/Distortion: Undefined, NotAvailable
Depth/Extrinsic: Undefined, NotAvailable
Depth/Extrinsic2: Undefined, NotAvailable
Depth/Intrinsic2: Undefined, NotAvailable
Depth/IntrinsicHeight: Int64, Readable
Depth/IntrinsicWidth: Int64, Readable
Depth/Rotation: Undefined, NotAvailable
Depth/SensorHeight: Int64, Readable
Depth/SensorWidth: Int64, Readable
Depth/AnalogGain: Float64, NotAvailable
Depth/DigitalGain: Float64, NotAvailable
Depth/RedGain: Float64, NotAvailable
Depth/GreenGain: Float64, NotAvailable
Depth/BlueGain: Float64, NotAvailable
Texture/ComponentEnable: Bool, ReadWritable
Texture/PixelFormat: Enumeration, ReadWritable
Texture/BinningHorizontal: Enumeration, ReadWritable
Texture/BinningVertical: Enumeration, ReadWritable
Texture/ExposureAuto: Bool, ReadWritable
Texture/ExposureTargetBrightness: Int64, NotAvailable
Texture/ExposureTime: Float64, ReadWritable
Texture/AutoFunctionAOIHeight: Int64, NotAvailable
Texture/AutoFunctionAOIOffsetX: Int64, NotAvailable
Texture/AutoFunctionAOIOffsetY: Int64, NotAvailable
Texture/AutoFunctionAOIWidth: Int64, NotAvailable
Texture/BalanceWhiteAuto: Bool, ReadWritable
Texture/HDRControl: Undefined, NotAvailable
Texture/HDREnable: Bool, NotAvailable
Texture/Intrinsic: Undefined, Readable
Texture/Distortion: Undefined, Readable
Texture/Extrinsic: Undefined, Readable
Texture/Extrinsic2: Undefined, Readable
Texture/Intrinsic2: Undefined, NotAvailable
Texture/IntrinsicHeight: Int64, Readable
Texture/IntrinsicWidth: Int64, Readable
Texture/Rotation: Undefined, NotAvailable
Texture/SensorHeight: Int64, Readable
Texture/SensorWidth: Int64, Readable
Texture/AnalogGain: Float64, ReadWritable
Texture/DigitalGain: Float64, NotAvailable
Texture/RedGain: Float64, NotAvailable
Texture/GreenGain: Float64, NotAvailable
Texture/BlueGain: Float64, NotAvailable
Left/ComponentEnable: Bool, ReadWritable
Left/PixelFormat: Enumeration, ReadWritable
Left/BinningHorizontal: Enumeration, ReadWritable
Left/BinningVertical: Enumeration, ReadWritable
Left/ExposureAuto: Bool, NotAvailable
Left/ExposureTargetBrightness: Int64, NotAvailable
Left/ExposureTime: Float64, ReadWritable
Left/AutoFunctionAOIHeight: Int64, NotAvailable
Left/AutoFunctionAOIOffsetX: Int64, NotAvailable
Left/AutoFunctionAOIOffsetY: Int64, NotAvailable
Left/AutoFunctionAOIWidth: Int64, NotAvailable
Left/BalanceWhiteAuto: Bool, NotAvailable
Left/HDRControl: Undefined, NotAvailable
Left/HDREnable: Bool, NotAvailable
Left/Intrinsic: Undefined, Readable
Left/Distortion: Undefined, Readable
Left/Extrinsic: Undefined, Readable
Left/Extrinsic2: Undefined, NotAvailable
Left/Intrinsic2: Undefined, Readable
Left/IntrinsicHeight: Int64, Readable
Left/IntrinsicWidth: Int64, Readable
Left/Rotation: Undefined, Readable
Left/SensorHeight: Int64, Readable
Left/SensorWidth: Int64, Readable
Left/AnalogGain: Float64, ReadWritable
Left/DigitalGain: Float64, ReadWritable
Left/RedGain: Float64, NotAvailable
Left/GreenGain: Float64, NotAvailable
Left/BlueGain: Float64, NotAvailable
Right/ComponentEnable: Bool, ReadWritable
Right/PixelFormat: Enumeration, ReadWritable
Right/BinningHorizontal: Enumeration, ReadWritable
Right/BinningVertical: Enumeration, ReadWritable
Right/ExposureAuto: Bool, NotAvailable
Right/ExposureTargetBrightness: Int64, NotAvailable
Right/ExposureTime: Float64, ReadWritable
Right/AutoFunctionAOIHeight: Int64, NotAvailable
Right/AutoFunctionAOIOffsetX: Int64, NotAvailable
Right/AutoFunctionAOIOffsetY: Int64, NotAvailable
Right/AutoFunctionAOIWidth: Int64, NotAvailable
Right/BalanceWhiteAuto: Bool, NotAvailable
Right/HDRControl: Undefined, NotAvailable
Right/HDREnable: Bool, NotAvailable
Right/Intrinsic: Undefined, Readable
Right/Distortion: Undefined, Readable
Right/Extrinsic: Undefined, Readable
Right/Extrinsic2: Undefined, NotAvailable
Right/Intrinsic2: Undefined, Readable
Right/IntrinsicHeight: Int64, Readable
Right/IntrinsicWidth: Int64, Readable
Right/Rotation: Undefined, Readable
Right/SensorHeight: Int64, Readable
Right/SensorWidth: Int64, Readable
Right/AnalogGain: Float64, ReadWritable
Right/DigitalGain: Float64, ReadWritable
Right/RedGain: Float64, NotAvailable
Right/GreenGain: Float64, NotAvailable
Right/BlueGain: Float64, NotAvailable
LightController0/LightBrightness: Int64, ReadWritable
LightController0/LightEnable: Bool, ReadWritable
LightController1/LightBrightness: Int64, ReadWritable
LightController1/LightEnable: Bool, NotAvailable
Left/DeviceTemperature: Float64, Readable
Right/DeviceTemperature: Float64, Readable
Texture/DeviceTemperature: Float64, Readable
* 
* FM855-1
GevPersistentIPAddress: Int64, ReadWritable
GevPersistentSubnetMask: Int64, ReadWritable
GevPersistentDefaultGateway: Int64, ReadWritable
PacketDelay: Int64, ReadWritable
NTPServerIP: Int64, ReadWritable
GevSCPSPacketSize: Int64, ReadWritable
DeviceLinkHeartbeatTimeout: Int64, ReadWritable
TriggerDelay: Int64, ReadWritable
TriggerDurationUs: Int64, ReadWritable
CaptureTimeStatistic: Int64, Readable
DeviceStreamAsyncMode: Enumeration, ReadWritable
DeviceTimeSyncMode: Enumeration, ReadWritable
GevGVSPResend: Bool, ReadWritable
DeviceLinkHeartbeatMode: Bool, ReadWritable
TriggerOutIO: Bool, Writable
TimeSyncAck: Bool, Readable
AcquisitionMode: Enumeration, ReadWritable
AcquisitionFrameRateEnable: Bool, ReadWritable
AcquisitionFrameRate: Float64, ReadWritable
TriggerParamLedGain: Int64, ReadWritable
TriggerSource: Enumeration, ReadWritable
TriggerParamLedExpo: Int64, ReadWritable
DepthSgbmImageNumber: Int64, ReadWritable
DepthSgbmDisparityNumber: Int64, ReadWritable
DepthSgbmDisparityOffset: Int64, ReadWritable
DepthSgbmMatchWinHeight: Int64, ReadWritable
DepthSgbmSemiParamP1: Int64, ReadWritable
DepthSgbmSemiParamP2: Int64, ReadWritable
DepthSgbmUniqueFactor: Int64, ReadWritable
DepthSgbmUniqueAbsDiff: Int64, ReadWritable
DepthSgbmMatchWinWidth: Int64, ReadWritable
DepthSgbmLRCDiff: Int64, ReadWritable
DepthSgbmMedFilterThresh: Int64, ReadWritable
DepthSgbmSemiParamP1Scale: Int64, ReadWritable
DepthSgbmTextureFilterValueOffset: Int64, Readable
DepthSgbmTextureFilterThreshold: Int64, ReadWritable
DepthScaleUnit: Float64, ReadWritable
Depth/PixelFormat: Enumeration, ReadWritable
DepthSgbmHFilterHalfWin: Bool, ReadWritable
DepthSgbmMedFilter: Bool, ReadWritable
DepthSgbmLRC: Bool, ReadWritable
Depth/Intrinsic: Dictionary, Readable
Depth/CalibrationData: Dictionary, Readable
Left/IntExposureTime: Int64, ReadWritable
Left/DigitalGain: Int64, ReadWritable
Left/AnalogGain: Int64, ReadWritable
Left/PixelFormat: Enumeration, Readable
Left/IRUndistortion: Bool, ReadWritable
Left/Intrinsic: Dictionary, Readable
Left/Distortion: Dictionary, Readable
Left/CalibrationData: Dictionary, Readable
Left/Intrinsic2: Dictionary, Readable
Right/IntExposureTime: Int64, ReadWritable
Right/DigitalGain: Int64, ReadWritable
Right/AnalogGain: Int64, ReadWritable
Right/PixelFormat: Enumeration, Readable
Right/IRUndistortion: Bool, ReadWritable
Right/Intrinsic: Dictionary, Readable
Right/Extrinsic: Dictionary, Readable
Right/Distortion: Dictionary, Readable
Right/CalibrationData: Dictionary, Readable
Right/Intrinsic2: Dictionary, Readable
Texture/IntExposureTime: Int64, ReadWritable
Texture/RedGain: Int64, ReadWritable
Texture/GreenGain: Int64, ReadWritable
Texture/BlueGain: Int64, ReadWritable
Texture/AnalogGain: Int64, ReadWritable
Texture/PixelFormat: Enumeration, ReadWritable
Texture/Intrinsic: Dictionary, Readable
Texture/Extrinsic: Dictionary, Readable
Texture/Distortion: Dictionary, Readable
Texture/CalibrationData: Dictionary, Readable
LightController1/LightBrightness: Int64, ReadWritable
LightController2/LightBrightness: Int64, ReadWritable
LightController0/LightBrightness: Int64, ReadWritable
LightController1/LightEnable: Bool, ReadWritable
LightController2/LightEnable: Bool, ReadWritable
*/
bool PercipioCam::SetOption(OPTION_TYPE optionType, float value, int msTimeout)
{
    auto impl = impl_;
    if (!impl) {
        return false;
    }

    try {
        switch (optionType) {
        case OPTION_TYPE_IR_EXPOSURE: {
            bool ok = false;
            if (m_camName.find("PM807-E1") != std::string::npos)
            {
                return true;
            }
            else if(m_camName.find("FM855-E1") != std::string::npos)
            {
                ok |= SetFeatureValue(impl->camera, "Left/IntExposureTime", static_cast<double>(value));
                ok |= SetFeatureValue(impl->camera, "Right/IntExposureTime", static_cast<double>(value));
                if (!ok) {
                    return false;
                }

                //auto leftValue = GetFeatureNumericValue(impl->camera, "Left/IntExposureTime");
                //auto rightValue = GetFeatureNumericValue(impl->camera, "Right/IntExposureTime");
                //if ((leftValue && IsCloseTo(*leftValue, value)) || (rightValue && IsCloseTo(*rightValue, value))) {
                //    return true;
                //}

                //const auto waitLeft = WaitForFeatureNumericValue(impl->camera, "Left/IntExposureTime", static_cast<double>(value), msTimeout);
                //const auto waitRight = WaitForFeatureNumericValue(impl->camera, "Right/IntExposureTime", static_cast<double>(value), msTimeout);
                //return waitLeft || waitRight;
			}
            else
            {
                return true;
            }
			return true;
        }
        case OPTION_TYPE_EMITTER: {
            //是否打开红外光泛光
            bool enableFloodLight = (value == 0.f);
            int stremType;
            if (!enableFloodLight) {
                stremType = (int)STREAM_TYPE::VIDEO | (int)STREAM_TYPE::DEPTH;
            }
            else {
                stremType = (int)STREAM_TYPE::VIDEO | (int)STREAM_TYPE::LEFT_IR | (int)STREAM_TYPE::RIGHT_IR;
            }

            if (config_.stremType != stremType) {
                config_.stremType = stremType;
                config_.enableFloodLight = enableFloodLight;
                return this->Open(config_);
            }

            if(config_.enableFloodLight != enableFloodLight)
            {
                bool ok = false;
                config_.enableFloodLight = enableFloodLight;
                if (m_camName.find("PM807-E1") != std::string::npos)
                {
                    ok |= SetFeatureValue(impl->camera, "LightController0/LightEnable", !config_.enableFloodLight);
                    ok |= SetFeatureValue(impl->camera, "LightController1/LightEnable", config_.enableFloodLight);
                }
                else if (m_camName.find("FM855-E1") != std::string::npos)
                {
                    ok |= SetFeatureValue(impl->camera, "LightController1/LightEnable", config_.enableFloodLight);
                    ok |= SetFeatureValue(impl->camera, "LightController2/LightEnable", config_.enableFloodLight);
                    ok |= SetFeatureValue(impl->camera, "LightController0/LightBrightness", config_.enableFloodLight ? 0 : 100);
                }
                else
                {
                    ok |= SetFeatureValue(impl->camera, "LightController0/LightEnable", !config_.enableFloodLight);
                    ok |= SetFeatureValue(impl->camera, "LightController1/LightEnable", config_.enableFloodLight);
                }
                return ok;
            }          
          
            return true;
        }
        default:
            return false;
        }
    }
    catch (...) {
        return false;
    }
}

float PercipioCam::GetPointCloudExposure()
{
    return 997.f;
}

float PercipioCam::GetTrackExposure()
{
    return 50.f;
}

bool PercipioCam::GetCamParam(CameraParameter& camParam) const
{
	if (IDepthCamera::GetCamParam(camParam))
	{
		return true;
	}

    auto impl = impl_;
    if (!impl) {
        return false;
    }

    vcamera::CalibInfo leftCalib;
    vcamera::CalibInfo rightCalib;
    vcamera::CalibInfo colorCalib;
    bool hasLeft = false;
    bool hasRight = false;
    bool hasColor = false;
    {
        std::lock_guard<std::mutex> lock(impl->mutex);
        hasLeft = impl->hasLeftCalib;
        hasRight = impl->hasRightCalib;
        hasColor = impl->hasColorCalib;
        if (hasLeft) leftCalib = impl->leftCalib;
        if (hasRight) rightCalib = impl->rightCalib;
        if (impl->hasColorCalib) colorCalib = impl->colorCalib;
    }

    if (!hasLeft || !hasRight || !hasColor) {
        return false;
    }

    cv::Mat KLeft = IntrinsicToCvMat(leftCalib.intrinsic);
    cv::Mat DLeft = DistortionToCvMat(leftCalib.distortion);
    cv::Mat KRight = IntrinsicToCvMat(rightCalib.intrinsic);
    cv::Mat DRight = DistortionToCvMat(rightCalib.distortion);
    cv::Mat KColor = IntrinsicToCvMat(colorCalib.intrinsic);
    cv::Mat DColor = DistortionToCvMat(colorCalib.distortion);

    // Open() 中已启用 Left/IRUndistortion 和 Right/IRUndistortion，
    // 当前检测到的左右像素坐标属于去畸变后的图像域。
    // 这里若再次按原始畸变参数执行 undistortPoints，会引入轻微旋转/偏斜误差。
    //DLeft = cv::Mat::zeros(DLeft.size(), DLeft.type());
    //DRight = cv::Mat::zeros(DRight.size(), DRight.type());

    //图漾相机中：外参矩阵是以左相机为基准
    //leftCalib.intrinsic是单位矩阵
    //rightCalib.extrinsic左->右的变换矩阵
    //colorCalib.extrinsic彩色->左的变换矩阵
    //const auto TLeft = ExtrinsicToCvMat(leftCalib.extrinsic);
    auto LeftToRight = ExtrinsicToCvMat(rightCalib.extrinsic);
    auto ColorToLeft = ExtrinsicToCvMat(colorCalib.extrinsic);

    //std::cout << "DLeft:\n" << DLeft << std::endl;
    //std::cout << "DRight:\n" << DRight << std::endl;
    //std::cout << "KLeft:\n" << KLeft << std::endl;
    //std::cout << "KRight:\n" << KRight << std::endl;
    //std::cout << "DColor:\n" << DColor << std::endl;
    //std::cout << "KColor:\n" << KColor << std::endl;
    //std::cout << "ColorToLeft:\n" << ColorToLeft << std::endl;
    //std::cout << "LeftToRight:\n" << LeftToRight << std::endl;

    auto LeftToColor = ColorToLeft.inv();

    auto& t1 = LeftToRight.at<double>(0, 3);
    auto& t2 = LeftToRight.at<double>(1, 3);
    auto& t3 = LeftToRight.at<double>(2, 3);
    auto t = t1 * t1 + t2 * t2 + t3 * t3;
    LeftToRight = cv::Mat::eye(4, 4, CV_64F);
    LeftToRight.at<double>(0, 3) = -std::sqrt(t);
    camParam.distModeLeft = DISTORTION_NONE;
    camParam.distModeRight = DISTORTION_NONE;
    camParam.distModeRight = DISTORTION_BROWN_CONRADY;
    camParam.DLeft = DLeft;
    camParam.DRight = DRight;
    camParam.KLeft = KLeft;
    camParam.KRight = KRight;
    camParam.DColor = DColor;
    camParam.KColor = KColor;
    camParam.LeftToColor = LeftToColor;
    camParam.LeftToRight = LeftToRight;

    /*std::cout << "====================最终返回:========================" << std::endl;
	std::cout << "DLeft:\n" << DLeft << std::endl;
    std::cout << "DRight:\n" << DRight << std::endl;
    std::cout << "KLeft:\n" << KLeft << std::endl;
    std::cout << "KRight:\n" << KRight << std::endl;
    std::cout << "DColor:\n" << DColor << std::endl;
    std::cout << "KColor:\n" << KColor << std::endl;
    std::cout << "LeftToColor:\n" << LeftToColor << std::endl;
    std::cout << "LeftToRight:\n" << LeftToRight << std::endl;*/

    camParam_ = camParam;

    return true;
}

bool PercipioCam::initDistMap() const
{
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (!impl_->m_distMap1.empty() && !impl_->m_distMap2.empty())
            return true;
    }

	CameraParameter camParam;
	CameraIntrinsics colorIntr;
	if (!this->GetCamParam(camParam) ||
		!this->GetColorIntrinsics(colorIntr))
	{
		return false;
	}

	cv::Size imageSize(colorIntr.width, colorIntr.height);
	// 可选：计算优化后的相机矩阵（控制 alpha）
	// 生成映射表（一次生成，重复使用）
	cv::initUndistortRectifyMap(
		camParam.KColor,
		camParam.DColor,
		cv::Mat(),
        camParam.KColor,
		imageSize,
		CV_16SC2,
        impl_->m_distMap1,
        impl_->m_distMap2);

	return !impl_->m_distMap1.empty() && !impl_->m_distMap2.empty();
}

