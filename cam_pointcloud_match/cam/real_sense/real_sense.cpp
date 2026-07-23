#include "real_sense.h"
#include "common/math/opencv_kit.h"
#include "common/common_kit.h"

#include <librealsense2/rs.hpp>
#include <librealsense2-gl/rs_processing_gl.hpp>

using namespace rs2;

RealSenseCam::~RealSenseCam()
{
}

bool RealSenseCam::Open(const CameConfig& camConfig)
{
    try {
        IDepthCamera::Open(camConfig);
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH, 0, 0, 0, RS2_FORMAT_Z16);
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, 0, 0, RS2_FORMAT_Y8);
        cfg.enable_stream(RS2_STREAM_INFRARED, 2, 0, 0, RS2_FORMAT_Y8);
        cfg.enable_stream(RS2_STREAM_COLOR, 0, 1280, 720, RS2_FORMAT_RGB8, 30);
        //cfg.enable_all_streams();
        pipe_ = std::make_shared<rs2::pipeline>();
        pipe_->start(cfg);
        return true;
    }
    catch(...) {
        return false;
    }
}

void RealSenseCam::Close()
{
    IDepthCamera::Close();
    try {
        pipe_->stop();
    }
    catch (...) {
    }
}

bool RealSenseCam::Snapshot(int32_t timeout)
{
    frameSet_ = nullptr;
    try {
        auto framesSet = pipe_->wait_for_frames(timeout);
        if (!framesSet.size())
            return false;

        rs2::gl::align align_to_color(RS2_STREAM_COLOR);
        framesSet = align_to_color.process(framesSet);
        frameSet_ = std::make_shared<rs2::frameset>(framesSet);
        return true;
    }
    catch (...) {
	    return false;
	}

}

PointCloudPtr RealSenseCam::GetPointCloud(float maxZ, float maxX, float maxY)
{
    auto color = frameSet_->get_color_frame();
    auto depth = frameSet_->get_depth_frame();
    if (!depth)
        return nullptr;

    if (!color)
        color = frameSet_->get_infrared_frame();

    rs2::pointcloud pc;
    if (color)
        pc.map_to(color);

    auto points = pc.calculate(depth);
    size_t num_points = points.size();
    auto cloud = std::make_shared<PointCloud>();

    const rs2::vertex* vertices = points.get_vertices();
    const rs2::texture_coordinate* tex_coords = points.get_texture_coordinates();

    int width = 0;
    int height = 0;
    const uint8_t* color_data = nullptr;
    if (color) {
        width = color.get_width();
        height = color.get_height();
        color_data = reinterpret_cast<const uint8_t*>(color.get_data());
    }

    const bool depth_aligned_to_color =
        (color && depth.get_width() == color.get_width() && depth.get_height() == color.get_height());

    // depth -> color 外参（米）
    bool has_extr = false;
    rs2_extrinsics extr{};
    if (color && !depth_aligned_to_color) {
        try {
            auto depth_profile = depth.get_profile().as<rs2::video_stream_profile>();
            auto color_profile = color.get_profile().as<rs2::video_stream_profile>();
            extr = depth_profile.get_extrinsics_to(color_profile);
            has_extr = true;
        }
        catch (...) {
            has_extr = false;
        }
    }

    // 过滤阈值（传入为 mm，转换为 m 比较）
    double maxZ_m = maxZ * 0.001;
    double maxX_m = maxX * 0.001;
    double maxY_m = maxY * 0.001;

    for (size_t i = 0; i < num_points; ++i) {
        const auto& v = vertices[i];
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
            continue;

        // depth 坐标系（米）
        double dx = v.x;
        double dy = v.y;
        double dz = v.z;

        // 过滤（米）
        if (std::abs(dx) > maxX_m || std::abs(dy) > maxY_m || dz > maxZ_m || dz <= 0.0)
            continue;

        // 转到 color 相机坐标系（米）
        double cx = dx, cy = dy, cz = dz;
        if (has_extr) {
            cx = extr.rotation[0] * dx + extr.rotation[1] * dy + extr.rotation[2] * dz + extr.translation[0];
            cy = extr.rotation[3] * dx + extr.rotation[4] * dy + extr.rotation[5] * dz + extr.translation[1];
            cz = extr.rotation[6] * dx + extr.rotation[7] * dy + extr.rotation[8] * dz + extr.translation[2];
        }

        // 转为 mm, 因为深度相机坐标系Y轴向下，X轴向右，为了便于在VTK显示，两者取反
        cloud->points_.emplace_back(-cx * 1000.0, -cy * 1000.0, cz * 1000.0);

        if (color && color_data) {
            float u = tex_coords[i].u;
            float v = tex_coords[i].v;
            int u_idx = std::min(std::max(int(u * width + 0.5f), 0), width - 1);
            int v_idx = std::min(std::max(int(v * height + 0.5f), 0), height - 1);
            int idx = v_idx * width + u_idx;
            const uint8_t* rgb = color_data + idx * 3;
            cloud->colors_.emplace_back(rgb[0] / 255.0, rgb[1] / 255.0, rgb[2] / 255.0);
        }
        else {
            cloud->colors_.emplace_back(1.0, 1.0, 1.0);
        }
    }
    return cloud;
}

bool RealSenseCam::GetColorIntrinsics(CameraIntrinsics& out) const
{
    if (!pipe_)
        return false;

    try {
        if (frameSet_) {
            auto color = frameSet_->get_color_frame();
            if (color && color.is<rs2::video_frame>()) {
                auto vsp = color.get_profile().as<rs2::video_stream_profile>();
                auto intr = vsp.get_intrinsics();
                out.width = intr.width;
                out.height = intr.height;
                out.fx = intr.fx;
                out.fy = intr.fy;
                out.ppx = intr.ppx;
                out.ppy = intr.ppy;
                return true;
            }
        }

        auto profile = pipe_->get_active_profile();
        for (auto sp : profile.get_streams()) {
            if (sp.stream_type() == RS2_STREAM_COLOR) {
                auto vsp = sp.as<rs2::video_stream_profile>();
                auto intr = vsp.get_intrinsics();
                out.width = intr.width;
                out.height = intr.height;
                out.fx = intr.fx;
                out.fy = intr.fy;
                out.ppx = intr.ppx;
                out.ppy = intr.ppy;
                return true;
            }
        }
    }
    catch (...) {
    }

    return false;
}

bool RealSenseCam::GetPointAtPixel(
    const rs2::depth_frame& depth,
    const rs2::video_frame& frame,
    int x,
    int y,
    Point3D* p,
    float depth_min,
    float depth_max)
{
    rs2::pointcloud pc;
    auto points = pc.calculate(depth);
    const int depth_width = depth.get_width();
    const int depth_height = depth.get_height();
    if (points.size() != static_cast<size_t>(depth_width * depth_height))
        return false;

    auto depth_scale = depth.get_units();
    rs2::video_frame other = frame.as<rs2::video_frame>();
    auto depth_profile = depth.get_profile().as<rs2::video_stream_profile>();
    auto other_profile = other.get_profile().as<rs2::video_stream_profile>();
    auto depth_intrin = depth_profile.get_intrinsics();
    auto other_intrin = other_profile.get_intrinsics();
    auto other_to_depth = other_profile.get_extrinsics_to(depth_profile);
    auto depth_to_other = depth_profile.get_extrinsics_to(other_profile);
    auto depth_data = reinterpret_cast<const uint16_t*>(depth.get_data());

    float from_pixel[2] = { (float)x, (float)y };
    float to_pixel[2] = { -1.f, -1.f };

    rs2_project_color_pixel_to_depth_pixel(
        to_pixel,
        depth_data,
        depth_scale,
        depth_min * depth_scale,
        depth_max * depth_scale,
        &depth_intrin,
        &other_intrin,
        &other_to_depth,
        &depth_to_other,
        from_pixel);

    if (to_pixel[0] < 0 || to_pixel[1] < 0)
        return false;

    auto depth_x = static_cast<int>(std::lround(to_pixel[0]));
    auto depth_y = static_cast<int>(std::lround(to_pixel[1]));
    if (depth_x < 0 || depth_y < 0 || depth_x >= depth_width || depth_y >= depth_height)
        return false;

    auto vertices = points.get_vertices();
    auto idx = depth_y * depth_width + depth_x;
    auto& v = vertices[idx];
    Point3D point(v.x / depth_scale, v.y / depth_scale, v.z / depth_scale);
    if (point.z() == 0.f || point.z() < depth_min || point.z() > depth_max)
        return false;

    if (p)
        *p = point;
    return true;
}

bool RealSenseCam::GetPointAtPixel(
    int x,
    int y,
    Point3D* p,
    STREAM_TYPE streamType)
{
    rs2::frame frame;
    switch (streamType)
    {
    case STREAM_TYPE::VIDEO: {
        frame = frameSet_->get_color_frame();
        break;
    }
    case STREAM_TYPE::IR: {
        frame = frameSet_->get_infrared_frame();
        break;
    }
    case STREAM_TYPE::LEFT_IR: {
        frame = frameSet_->get_infrared_frame(1);
        break;
    }
    case STREAM_TYPE::RIGHT_IR: {
        frame = frameSet_->get_infrared_frame(2);
        break;
    }
    default:
        break;
    }

    auto depth = frameSet_->get_depth_frame();
    return GetPointAtPixel(depth, frame.as<rs2::video_frame>(), x, y, p);
}

PointCloudPtr RealSenseCam::GetPointColudFromPixel(
    const VecPixelCoord& pixelCoords,
    std::vector<size_t>* failPCIndexes,
    VecRGB* rgb)
{
    return PointCloudPtr();
}

int RealSenseCam::GetWidth() const
{
    return 0;
}

int RealSenseCam::GetHeight() const
{
    return 0;
}

Mat RealSenseCam::VideoFrameToMat(const rs2::video_frame& frame) const
{
    int width = frame.get_width();
    int height = frame.get_height();
    rs2_format format = frame.get_profile().format();

    switch (format) {
    case RS2_FORMAT_RGB8: {
        cv::Mat rgb(height, width, CV_8UC3, (void*)frame.get_data());
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }
    case RS2_FORMAT_BGR8: {
        return cv::Mat(height, width, CV_8UC3, (void*)frame.get_data()).clone();
    }
    case RS2_FORMAT_RGBA8: {
        cv::Mat rgba(height, width, CV_8UC4, (void*)frame.get_data());
        cv::Mat bgr;
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
        return bgr;
    }
    case RS2_FORMAT_BGRA8: {
        cv::Mat bgra(height, width, CV_8UC4, (void*)frame.get_data());
        cv::Mat bgr;
        cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
        return bgr;
    }
    case RS2_FORMAT_YUYV: {
        cv::Mat yuyv(height, width, CV_8UC2, (void*)frame.get_data());
        cv::Mat bgr;
        cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUY2);
        return bgr;
    }
    case RS2_FORMAT_Y8: {
        return cv::Mat(height, width, CV_8UC1, (void*)frame.get_data()).clone();
    }
    case RS2_FORMAT_Z16: {
        return cv::Mat(height, width, CV_16U, (void*)frame.get_data()).clone();
    }
    default:
        return cv::Mat();
    }
}

void RealSenseCam::GetFrame(UNMAP_MAT& frames) const
{
    rs2::colorizer color_map;
    frameSet_->apply_filter(color_map);
    for (auto& it : frames) {
        rs2::frame frame;
        switch (it.first) {
        case STREAM_TYPE::VIDEO: {
            frame = frameSet_->get_color_frame();
            break;
        }
        case STREAM_TYPE::DEPTH: {
            frame = frameSet_->get_depth_frame();
            break;
        }
        case STREAM_TYPE::IR: {
            frame = frameSet_->get_infrared_frame();
            break;
        }
        case STREAM_TYPE::LEFT_IR: {
            frame = frameSet_->get_infrared_frame(1);
            break;
        }
        case STREAM_TYPE::RIGHT_IR: {
            frame = frameSet_->get_infrared_frame(2);
            break;
        }
        default:
            break;
        }

        if (frame && frame.is<rs2::video_frame>()) {
            rs2::video_frame videoFrame = frame.as<rs2::video_frame>();
            cv::Mat frameMat = VideoFrameToMat(videoFrame);
            if (!frameMat.empty())
                it.second = frameMat;
        }
    }
}

namespace {
    int MapRs2DistortionToModel(int rs2_model) {
        switch (rs2_model) {
        case 0: return DISTORTION_NONE;
        case 1: // RS2_DISTORTION_MODIFIED_BROWN_CONRADY
        case 4: // RS2_DISTORTION_BROWN_CONRADY
            return DISTORTION_BROWN_CONRADY;
        case 3: // RS2_DISTORTION_FTHETA
        case 5: // RS2_DISTORTION_KANNALA_BRANDT4
            return DISTORTION_FISHEYE;
        default:
            return DISTORTION_OTHER;
        }
    }

    //  ڲ ת  
    inline void RsIntrinsicsToCv(const rs2_intrinsics& intrin, cv::Mat& K, cv::Mat& D) {
        K = (cv::Mat_<double>(3, 3) <<
            intrin.fx, 0, intrin.ppx,
            0, intrin.fy, intrin.ppy,
            0, 0, 1);
        D = cv::Mat::zeros(1, 5, CV_64F);
        for (int i = 0; i < 5; ++i)
            D.at<double>(0, i) = intrin.coeffs[i];
    }

    inline void RsExtrinsicsToCv(const rs2_extrinsics& extr,cv::Mat& T) {
        T = cv::Mat(4, 4, CV_64F);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                T.at<double>(i, j) = extr.rotation[j * 3 + i];
        for (int i = 0; i < 3; ++i)
            T.at<double>(i, 3) = extr.translation[i];
        T.at<double>(3, 0) = 0.0;
        T.at<double>(3, 1) = 0.0;
        T.at<double>(3, 2) = 0.0;
        T.at<double>(3, 3) = 1.0;
    }
}

Point3D RealSenseCam::BackProject3DFrom2D(
    float left_u,
    float left_v,
    float right_u,
    float right_v,
    const CameraParameter& camParam)
{
    Point3D point_m = InvalidPoint3D();
    cv::Point2f left_pt((float)left_u, (float)left_v);
    cv::Point2f right_pt((float)right_u, (float)right_v);
    auto ret = TriangulatePointToCamera(
        left_pt,
        right_pt,
        camParam,
        &point_m);

    if (ret) {
        Point3D point_color_mm;
        point_color_mm.x() = -point_m.x() * 1000.0;
        point_color_mm.y() = -point_m.y() * 1000.0;
        point_color_mm.z() = point_m.z() * 1000.0;
        return point_color_mm;
    }

    return InvalidPoint3D();
}

float RealSenseCam::GetVFOV()
{
    if (frameSet_) {
        // 优先使用彩色相机
        auto color = frameSet_->get_color_frame();
        if (color && color.is<rs2::video_frame>()) {
            auto vsp = color.get_profile().as<rs2::video_stream_profile>();
            auto intr = vsp.get_intrinsics();
            double vfov_rad = 2.0 * std::atan(static_cast<double>(intr.height) / (2.0 * intr.fy));
            double vfov_deg = vfov_rad * 180.0 / std::acos(-1.0);
            return static_cast<float>(vfov_deg);
        }

        // 回退到深度相机
        auto depth = frameSet_->get_depth_frame();
        if (depth && depth.is<rs2::video_frame>()) {
            auto vsp = depth.get_profile().as<rs2::video_stream_profile>();
            auto intr = vsp.get_intrinsics();
            double vfov_rad = 2.0 * std::atan(static_cast<double>(intr.height) / (2.0 * intr.fy));
            double vfov_deg = vfov_rad * 180.0 / std::acos(-1.0);
            return static_cast<float>(vfov_deg);
        }
    }

    // 回退：从 pipeline active profile 获取彩色流
    auto profile = pipe_->get_active_profile();
    for (auto sp : profile.get_streams()) {
        if (sp.stream_type() == RS2_STREAM_COLOR) {
            auto vsp = sp.as<rs2::video_stream_profile>();
            auto intr = vsp.get_intrinsics();
            double vfov_rad = 2.0 * std::atan(static_cast<double>(intr.height) / (2.0 * intr.fy));
            double vfov_deg = vfov_rad * 180.0 / std::acos(-1.0);
            return static_cast<float>(vfov_deg);
        }
    }

    return -1.0f;
}

bool RealSenseCam::SetResolution(STREAM_TYPE streamType, int width, int height)
{
    if (!pipe_) return false;

    // 将 STREAM_TYPE 映射到 rs2_stream
    rs2_stream rs2Stream = RS2_STREAM_ANY;
    rs2_format rs2Fmt = RS2_FORMAT_ANY;
    int streamIndex = 0;

    switch (streamType) {
    case STREAM_TYPE::VIDEO:
        rs2Stream = RS2_STREAM_COLOR;
        rs2Fmt = RS2_FORMAT_RGB8;
        break;
    case STREAM_TYPE::DEPTH:
        rs2Stream = RS2_STREAM_DEPTH;
        rs2Fmt = RS2_FORMAT_Z16;
        break;
    case STREAM_TYPE::IR:
        rs2Stream = RS2_STREAM_INFRARED;
        rs2Fmt = RS2_FORMAT_Y8;
        break;
    case STREAM_TYPE::LEFT_IR:
        rs2Stream = RS2_STREAM_INFRARED;
        rs2Fmt = RS2_FORMAT_Y8;
        streamIndex = 1;
        break;
    case STREAM_TYPE::RIGHT_IR:
        rs2Stream = RS2_STREAM_INFRARED;
        rs2Fmt = RS2_FORMAT_Y8;
        streamIndex = 2;
        break;
    default:
        return false;
    }

    // 记录所有活跃流的当前配置（放在 try 外，catch 恢复时也需要）
    auto activeProfile = pipe_->get_active_profile();
    auto streams = activeProfile.get_streams();

    // 在已有的传感器流中查找匹配的帧率
    int fps = 30;
    for (auto& sp : streams) {
        if (sp.stream_type() == rs2Stream &&
            (streamIndex == 0 || sp.stream_index() == streamIndex)) {
            fps = sp.fps();
            break;
        }
    }

    try {
        pipe_->stop();

        rs2::config cfg;
        for (auto& sp : streams) {
            auto vsp = sp.as<rs2::video_stream_profile>();
            if (sp.stream_type() == rs2Stream &&
                (streamIndex == 0 || sp.stream_index() == streamIndex)) {
                // 目标流：使用新的分辨率
                cfg.enable_stream(rs2Stream, streamIndex, width, height, rs2Fmt, fps);
            } else {
                // 其他流：保持原配置
                cfg.enable_stream(
                    sp.stream_type(), sp.stream_index(),
                    vsp.width(), vsp.height(),
                    sp.format(), sp.fps());
            }
        }

        pipe_->start(cfg);
        return true;
    }
    catch (...) {
        // 启动失败时尝试用之前保存的流配置恢复
        try {
            rs2::config cfg;
            for (auto& sp : streams) {
                auto vsp = sp.as<rs2::video_stream_profile>();
                cfg.enable_stream(
                    sp.stream_type(), sp.stream_index(),
                    vsp.width(), vsp.height(),
                    sp.format(), sp.fps());
            }
            pipe_->start(cfg);
        }
        catch (...) {}
        return false;
    }
}

bool RealSenseCam::SetOption(OPTION_TYPE optionType, float value, int msTimeout)
{
    rs2_stream rs2Stream;
    rs2_option rs2Option;
    switch (optionType)
    {
    case OPTION_TYPE_IR_EXPOSURE:
        rs2Option = RS2_OPTION_EXPOSURE;
        rs2Stream = RS2_STREAM_INFRARED;
        break;
    case OPTION_TYPE_EMITTER:
        rs2Option = RS2_OPTION_EMITTER_ENABLED;
        rs2Stream = RS2_STREAM_DEPTH;
        break;
    default:
        return -1.0f;
    }
    try {
        auto profile = pipe_->get_active_profile();
        auto dev = profile.get_device();
        rs2::sensor sensor;
        for (auto& s : dev.query_sensors())
        {
            for (auto& sp : s.get_stream_profiles())
            {
                if (sp.stream_type() == rs2Stream)
                {
                    sensor = s;
                    break;
                }
            }
            if (sensor)
                break;
        }
        if (!sensor)
            return false;

        if(optionType == OPTION_TYPE_IR_EXPOSURE) {
            if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
                WaitForReturnVal([&] { return sensor.get_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE); }, 0.0f, msTimeout);
            }
		}

        if (!sensor.supports(rs2Option))
            return false;

        auto range = sensor.get_option_range(rs2Option);
        float v = std::max(range.min, std::min(range.max, value));
        sensor.set_option(rs2Option, v);

        return WaitForReturnVal([&] { return sensor.get_option(rs2Option); }, value, msTimeout);
    }
    catch (...)
    {
        return false;
    }
}

float RealSenseCam::GetOption(OPTION_TYPE optionType)
{
	rs2_stream rs2Stream;
    rs2_option rs2Option;
    switch (optionType)
    {
    case OPTION_TYPE_IR_EXPOSURE:
        rs2Option = RS2_OPTION_EXPOSURE;
        rs2Stream = RS2_STREAM_INFRARED;
        break;
    case OPTION_TYPE_EMITTER:
        rs2Option = RS2_OPTION_EMITTER_ENABLED;
		rs2Stream = RS2_STREAM_DEPTH;
        break;
    default:
        return -1.0f;
    }

    auto profile = pipe_->get_active_profile();
    auto dev = profile.get_device();
    rs2::sensor sensor;
    for (auto& s : dev.query_sensors()) {
        for (auto& sp : s.get_stream_profiles()) {
            if (sp.stream_type() == rs2Stream) {
                sensor = s;
                break;
            }
        }
        if (sensor) break;
    }
    if (!sensor) return -1.0f;

    if (!sensor.supports(rs2Option)) 
        return -1.0f;

    auto v = sensor.get_option(rs2Option);
    return (float)v;
}

float RealSenseCam::GetPointCloudExposure()
{
    return 33000.f;
}

float RealSenseCam::GetTrackExposure()
{
    return 2000.0f;
}

bool RealSenseCam::GetCamParam(CameraParameter& camParam) const
{
    if (IDepthCamera::GetCamParam(camParam))
    {
        return true;
    }

    if(!frameSet_)
		return false;

    auto leftIR = frameSet_->get_infrared_frame(1);
    auto rightIR = frameSet_->get_infrared_frame(2);
    auto colorFrame = frameSet_->get_color_frame();

    if (!leftIR || !rightIR || !colorFrame)
        return false;

    auto left_profile = leftIR.get_profile().as<rs2::video_stream_profile>();
    auto right_profile = rightIR.get_profile().as<rs2::video_stream_profile>();
    auto color_profile = colorFrame.get_profile().as<rs2::video_stream_profile>();
    auto left_intrin = left_profile.get_intrinsics();
    auto right_intrin = right_profile.get_intrinsics();
    auto left_to_right = left_profile.get_extrinsics_to(right_profile);
    auto left_to_color = left_profile.get_extrinsics_to(color_profile);

    cv::Mat K_left;
    cv::Mat D_left;
    RsIntrinsicsToCv(left_intrin, K_left, D_left);

    cv::Mat K_right;
    cv::Mat D_right;
    RsIntrinsicsToCv(right_intrin, K_right, D_right);

    cv::Mat T_left_to_right;
    RsExtrinsicsToCv(left_to_right, T_left_to_right);

    cv::Mat T_left_to_color;
    RsExtrinsicsToCv(left_to_color, T_left_to_color);

    auto left_Mode = MapRs2DistortionToModel(left_intrin.model);
    auto right_Mode = MapRs2DistortionToModel(right_intrin.model);

    camParam.distModeLeft = MapRs2DistortionToModel(left_intrin.model);
    camParam.distModeRight = MapRs2DistortionToModel(right_intrin.model);
    camParam.DLeft = D_left;
    camParam.DRight = D_right;
    camParam.KLeft = K_left;
    camParam.KRight = K_right;
    camParam.LeftToColor = T_left_to_color;
    camParam.LeftToRight = T_left_to_right;

	camParam_ = camParam;

    return true;
}