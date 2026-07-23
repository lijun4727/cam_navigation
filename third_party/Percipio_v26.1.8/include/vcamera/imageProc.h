#include "cameraDefines.h"

#include <vector>

namespace ucv {
class PointCloud;
}

namespace vcamera {
class CAMERA_API_EXPORT ImageProc {
   public:
   /// Enable or disable parallel processing for the image processing functions in this class.
    static CameraApiStatus SetParallelEnabled(bool enabled);
    
    ////////// Coordinate transformation //////////

    /// Convert a depth image into a 3D point cloud.
    /// @param depth Input depth image.
    /// @param status Optional output status code.
    /// @return Converted point cloud.
    static PointCloud DepthImageToPointCloud(const Image& depth, CameraApiStatusCode* status = nullptr);

    /// Project a point cloud back to a depth image with the target depth resolution.
    /// @param point_cloud Input point cloud.
    /// @param depth_intrinsic Intrinsic parameters used for depth projection.
    /// @param depth_width Output depth image width in pixels.
    /// @param depth_height Output depth image height in pixels.
    /// @param depth_scale_unit Depth scale factor applied during projection.
    /// @param status Optional output status code.
    /// @return Reprojected depth image.
    static Image PointCloudToDepthImage(const PointCloud& point_cloud, const CameraIntrinsic& depth_intrinsic,
                                        uint32_t depth_width, uint32_t depth_height, float depth_scale_unit = 1.0f,
                                        CameraApiStatusCode* status = nullptr);

    /// Transform point cloud coordinates by an extrinsic matrix.
    /// @param point_cloud Input point cloud.
    /// @param extrinsic Extrinsic transform matrix.
    /// @param status Optional output status code.
    /// @param inverse Whether to apply the inverse transform.
    /// @return Transformed point cloud.
    static PointCloud TransformPointCloud(const PointCloud& point_cloud, const CameraExtrinsic& extrinsic,
                                          CameraApiStatusCode* status = nullptr, bool inverse = false);

    /// Map a texture image to the depth image coordinate.
    /// @param texture Input texture image.
    /// Note: If input texture image is distorted, this function will do undistortion before
    /// mapping (a temp copy is created and undistorted, and the original input texture image will not be modified).
    /// @param depth Reference depth image that defines the target coordinate system.
    /// @param status Optional output status code.
    /// @return Texture image mapped to depth.
    static Image MapTextureImageToDepth(const Image& texture, const Image& depth,
                                        CameraApiStatusCode* status = nullptr);

    /// Map a gray image to the depth image coordinate.
    /// @param gray Input gray image.
    /// Note: If input gray image is distorted, this function will do undistortion before
    /// mapping (a temp copy is created and undistorted, and the original input gray image will not be modified).
    /// @param depth Reference depth image that defines the target coordinate system.
    /// @param status Optional output status code.
    /// @return Gray image mapped to depth.
    static Image MapGrayImageToDepth(const Image& gray, const Image& depth, CameraApiStatusCode* status = nullptr);

    /// Map a depth image to the texture image coordinate.
    /// Note: If input depth image is distorted, this function will do undistortion before
    /// mapping (a temp copy is created and undistorted, and the original input depth image will not be modified).
    /// @param depth Input depth image.
    /// @param texture Reference texture image that defines the target coordinate system.
    /// @param status Optional output status code.
    /// @return Depth image mapped to texture.
    static Image MapDepthImageToTexture(const Image& depth, const Image& texture,
                                        CameraApiStatusCode* status = nullptr);

    ////////// General image processing //////////

    /// Undistort an image.
    /// @param input_image Input image to undistort.
    /// @param camera_new_intrinsic Optional expected new image intrinsic.
    /// @param status Optional output status code.
    /// @return Undistorted image.
    static Image UndistortImage(const Image& input_image, CameraIntrinsic* camera_new_intrinsic = nullptr,
                                CameraApiStatusCode* status = nullptr);

    ////////// Depth image processing //////////
    /// Fill small holes in a depth image.
    /// @param input_image Input depth image.
    /// @param kernel_size Kernel size used for inpainting.
    /// @param max_internal_hole_to_be_filled Maximum internal hole size to fill.
    /// @param status Optional output status code.
    /// @return Inpainted depth image.
    static Image DepthInpaint(const Image& input_image, int kernel_size, int max_internal_hole_to_be_filled,
                              CameraApiStatusCode* status = nullptr);

    /// Remove speckles from a depth image.
    /// @param input_image Input depth image.
    /// @param difference Maximum depth difference for connected-region grouping.
    /// @param max_speckle_size Maximum speckle size to remove.
    /// @param status Optional output status code.
    /// @return Filtered depth image.
    static Image SpeckleFilter(const Image& input_image, double difference, double max_speckle_size,
                               CameraApiStatusCode* status = nullptr);

    /// Enhance depth quality using multi-frame filtering and optional guide image.
    /// @param depth_images Input depth image sequence.
    /// @param sigma_s Filter param on space.
    /// @param sigma_r Filter param on range.
    /// @param outlier_win_sz Outlier filter windows size.
    /// @param outlier_rate Outiler rate.
    /// @param guide Optional guide image for joint filtering.
    /// @param status Optional output status code.
    /// @return Enhanced depth image.
    static Image DepthEnhenceFilter(const std::vector<Image>& depth_images,
                                    float sigma_s,
                                    float sigma_r,
                                    int outlier_win_sz,
                                    float outlier_rate, Image* guide = nullptr, CameraApiStatusCode* status = nullptr);
};

}  // namespace vcamera
