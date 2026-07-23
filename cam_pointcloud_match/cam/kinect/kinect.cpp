#include "kinect.h"

#include <k4a/k4atypes.h>
#include <k4a/k4a.h>

bool KinectV2Cam::Open(const CameConfig & camConfig) {
    IDepthCamera::Open(camConfig);

    uint32_t count = k4a_device_get_installed_count();
    if (!count){
        printf("No k4a devices attached!\n");
        return false;
    }

    // Open the first plugged in Kinect device
    if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &device_))){
        printf("Failed to open k4a device!\n");
        return false;
    }

    // Configure a stream of 4096x3072 BRGA color data at 15 frames per second
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true;

    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device_, 
            config.depth_mode, 
            config.color_resolution, 
            &calibration)){
        printf("Failed to get calibration\n");
        return false;
    }

    transformation_ = k4a_transformation_create(&calibration);

    // Start the camera with the given configuration
    if (K4A_FAILED(k4a_device_start_cameras(device_, &config))){
        printf("Failed to start cameras!\n");
        return false;
    }

    return true;
}

void KinectV2Cam::Close() {
    IDepthCamera::Close();
	if (transformation_)
		k4a_transformation_destroy(transformation_);
	if (depth_image_)
		k4a_image_release(depth_image_);
	if (color_image_)
		k4a_image_release(color_image_);
	if (device_)
		k4a_device_stop_cameras(device_);
	if (device_)
		k4a_device_close(device_);
}

bool KinectV2Cam::Snapshot(int32_t timeout) {
    
    k4a_capture_t capture = nullptr;
    bool ret = false;
    switch (k4a_device_get_capture(device_, &capture, timeout)){
    case K4A_WAIT_RESULT_SUCCEEDED:
        ret = true;
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        printf("Timed out waiting for a capture\n");
        break;
    case K4A_WAIT_RESULT_FAILED:
        printf("Failed to read a capture\n");
        break;
    }

    // Get a depth image
    depth_image_ = k4a_capture_get_depth_image(capture);
    if (!depth_image_){
        printf("Failed to get depth image from capture\n");
        return false;
    }

    // Get a color image
    color_image_ = k4a_capture_get_color_image(capture);
    if (!color_image_){
        printf("Failed to get color image from capture\n");
        return false;
    }

    k4a_capture_release(capture);
    return ret;
}

PointCloudPtr KinectV2Cam::GetPointCloud(
    float maxZ,
    float maxX,
    float maxY) {
    int w = k4a_image_get_width_pixels(color_image_);
    int h = k4a_image_get_height_pixels(color_image_);
    k4a_image_t transformed_depth_image = nullptr;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
        w,
        h,
        w * (int)sizeof(uint16_t),
        &transformed_depth_image)){
        printf("Failed to create transformed depth image\n");
        return nullptr;
    }

    k4a_image_t point_cloud_image = nullptr;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        w,
        h,
        w * 3 * (int)sizeof(int16_t),
        &point_cloud_image)){
        printf("Failed to create point cloud image\n");
        return nullptr;
    }

	if (K4A_RESULT_SUCCEEDED !=
		k4a_transformation_depth_image_to_color_camera
		(transformation_,
			depth_image_,
			transformed_depth_image)) {
		printf("Failed to compute transformed depth image\n");
		return nullptr;
	}

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_point_cloud(
            transformation_,
        transformed_depth_image,
        K4A_CALIBRATION_TYPE_COLOR,
        point_cloud_image)){
        printf("Failed to compute point cloud\n");
        return nullptr;
    }

    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    int16_t* point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);
    uint8_t* color_image_data = k4a_image_get_buffer(color_image_);
    for (int i = 0; i < w * h; i++)
    {
        auto r = color_image_data[4 * i + 0];
        auto g = color_image_data[4 * i + 1];
        auto b = color_image_data[4 * i + 2];
        auto alpha = color_image_data[4 * i + 3];

        auto x = point_cloud_image_data[3 * i + 0];
        auto y = point_cloud_image_data[3 * i + 1];
        auto z = point_cloud_image_data[3 * i + 2];
        if(z == 0 || 
            z > (int16_t)maxZ || 
            std::abs(x) > (int16_t)maxX || 
            std::abs(y) > (int16_t)maxY ||
            (r == 0 && g == 0 && b == 0 && alpha == 0))
            continue;

		cloud->points_.emplace_back(
			Eigen::Vector3d(x, y, z)
		);
		cloud->colors_.emplace_back(
			Eigen::Vector3d(
				r / 255.0,
				g / 255.0,
				b / 255.0)
		);
    }

    k4a_image_release(transformed_depth_image);
    k4a_image_release(point_cloud_image);

    return cloud;
}

bool KinectV2Cam::GetPointAtPixel(
    int x, 
    int y, 
    Point3D* p, 
    STREAM_TYPE stremType)
{
    return false;
}

PointCloudPtr KinectV2Cam::GetPointColudFromPixel(
    const VecPixelCoord&,
    std::vector<size_t >*,
    VecRGB*) {

    return nullptr;
}

int KinectV2Cam::GetWidth() const {
    return 0;
}

int KinectV2Cam::GetHeight() const {

    return 0;
}

void KinectV2Cam::GetFrame(UNMAP_MAT& frames) const
{
}