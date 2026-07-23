#pragma once

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <open3d/geometry/PointCloud.h>
#include <opencv2/core/mat.hpp>
#include <cmath>
#include <limits>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkPolyData.h>

using PolyDataPtr = vtkSmartPointer<vtkPolyData>;
using ActorPtr = vtkSmartPointer<vtkActor>;
using RenderPtr = vtkSmartPointer<vtkRenderer>;
using PointCloud = open3d::geometry::PointCloud;
using PointCloudPtr = std::shared_ptr<PointCloud>;
using TriangleMesh = open3d::geometry::TriangleMesh;
using TriangleMeshPtr = std::shared_ptr<TriangleMesh>;
using Point3D = Eigen::Vector3d;
using PixelCoord = Eigen::Vector2i;  // 像素坐标 (u, v)
using VecPixelCoord = std::vector<PixelCoord>;
using RGB = Eigen::Vector3d;
using VecRGB = std::vector<RGB>;
using Mat = cv::Mat;
using VecInt = std::vector<int>;

/**
 * @brief 精配准结果结构体
 */
struct FineAlignmentResult {
    Eigen::Matrix4d transformation; // 配准得到的4x4刚体变换矩阵（旋转+平移）
    double fitness;                 // 配准“适应度”，表示匹配点对比例，越大越好（0~1）,
    // 如果目标点云本就缺失部分区域，则会降低fitness值
    double inlier_rmse;             // 内点均方根误差（Root Mean Square Error），表示配准
    // 后点对的平均距离，越小越好
    bool converged;                 // 是否收敛，true表示配准成功，false表示失败
};

// 创建/检测无效 Point3D 的辅助函数
inline Point3D InvalidPoint3D() {
    return Point3D::Constant(std::numeric_limits<double>::quiet_NaN());
}

inline bool IsValidPoint3D(const Point3D& p) {
    return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z());
}

struct CameraParameter {
    cv::Mat KColor;                     //彩色相机的内参3x3(OpenCV格式)
    cv::Mat DColor;                     //彩色相机的畸变矩阵(OpenCV格式，1x5、1x4、1x12)

    cv::Mat KLeft;                      //左相机的内参3x3(OpenCV格式)
    cv::Mat DLeft;                      //左相机的畸变矩阵(OpenCV格式，1x5、1x4、1x12)
    int distModeLeft;                   //左相机的畸变类型(参考enum DistortionModel)

    cv::Mat KRight;                     //右相机的内参3x3(OpenCV格式)
    cv::Mat DRight;                     //右相机的畸变矩阵(OpenCV格式，1x5、1x4、1x12)
    int distModeRight;                  //右相机的畸变类型(参考enum DistortionModel)   

    cv::Mat LeftToRight;                //左->右相机外参矩阵 4x4 (OpenCV格式)
    cv::Mat LeftToColor;                //左->彩色相机外参矩阵 4x4 (OpenCV格式)
};

#ifdef DEBUG_LOG
#define DEBUG_WRITE_PLY(filename, pointcloud) open3d::io::WritePointCloud(filename, pointcloud)
#else
#define DEBUG_WRITE_PLY(filename, pointcloud)
#endif

#ifdef _WIN32
#  ifdef CAM_POINTCLOUD_MATCH_EXPORTS
#    define CAM_API __declspec(dllexport)
#  else
#    define CAM_API __declspec(dllimport)
#  endif
#else
#  define CAM_API
#endif

#define MAX_SPOT_NUM 30

