#pragma once
#include "type.h"

#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/TriangleMeshIO.h>  // ← 添加

/**
 * @brief 通过对应点计算刚体变换矩阵
 */
CAM_API Eigen::Matrix4d ComputeTransformationFromCorrespondences(
    const PointCloud& source,
    const PointCloud& target,
    double* mae = nullptr);

/**
 * @brief 自适应 ICP: 估计目标法向量 + 方向校正 (推荐方案)
 *
 * 【核心思想】
 * 不传播源法向量, 而是估计目标点云自身的法向量,
 * 然后用源法向量校正方向, 避免 180° 翻转
 *
 * 【为什么这样更好?】
 *
 * 1. 更符合 ICP 数学原理
 *    Point-to-Plane: E = Σ[(p_s - p_t) · n_t]²
 *    n_t 应该是目标点处的法向量, 不是源点的
 *
 * 2. 避免位置误差带来的法向量误差
 *    粗配准误差 5mm 时:
 *    - 传播方案: n_t = n_s (可能不在同一表面位置)
 *    - 本方案: n_t 通过 PCA 估计, 反映目标点真实几何
 *
 * 3. 适合曲率变化大的区域 (如人脸五官)
 *    5mm 偏差在鼻子区域可能导致法向量差异 10°
 *    估计目标法向量可避免此问题
 *
 * 【算法流程】
 *
 * 1. 输入验证
 * 2. 下采样 (仅目标)
 * 3. 统计滤波 (去噪)
 * 4. 评估粗配准质量
 *
 * 5. 分支处理:
 *
 *    粗配准好 (< 5mm):
 *      a. 估计目标法向量 (PCA, 邻域 50 点)
 *      b. 用源法向量校正目标法向量方向
 *         - 找最近源点, 获取其法向量
 *         - 如果夹角 > 90°, 翻转目标法向量
 *      c. Point-to-Plane ICP
 *
 *    粗配准差 (>= 5mm):
 *      a. 不估计法向量
 *      b. Point-to-Point ICP (更鲁棒)
 *
 * 【法向量方向校正原理】
 *
 * PCA 估计的法向量方向是随机的 (可能指向内或外)
 *
 * 校正方法:
 *   for 每个目标点 p_t:
 *     1. 估计法向量: n_t_raw (PCA, 方向未知)
 *     2. 找最近源点 p_s
 *     3. 获取源法向量: n_s (准确的, 来自 STL)
 *     4. 计算夹角: dot = n_t_raw · n_s
 *     5. 如果 dot < 0 (夹角 > 90°):
 *          n_t = -n_t_raw  (翻转)
 *        否则:
 *          n_t = n_t_raw   (保持)
 *
 * 【为什么校正可行?】
 * - 粗配准后, p_s 和 p_t 接近 (< 5mm)
 * - 它们在同一表面上, 法向量方向应该一致
 * - 用准确的 n_s 来"指导" n_t 的方向
 *
 * 【参数说明】
 * @param source_cloud2 源点云 (50万点, 必须有法向量)
 * @param target_cloud2 目标点云 (单侧扫描, 密集, 无需法向量)
 * @param target_voxel_size 目标下采样体素大小 (mm), 默认 1.0mm
 * @param max_correspondence_distance ICP 搜索距离 (mm), 默认 2.0mm
 * @param max_iteration 最大迭代次数, 默认 50
 * @param rmse_threshold 成功判定 RMSE 阈值 (mm), 默认 2.0mm
 * @param use_point_to_plane 是否使用 Point-to-Plane ICP, 默认 true
 *        true: 估计+校正法向量, 使用 Point-to-Plane
 *        false: 不用法向量, 使用 Point-to-Point
 *
 * @return FineAlignmentResult 包含变换矩阵和质量指标
 * @throws std::runtime_error 如果源点云没有法向量
 */
CAM_API FineAlignmentResult FineAlignment(
    const PointCloudPtr& source_cloud2,
    const PointCloudPtr& target_cloud2,
    double target_voxel_size = 1.0,
    double max_correspondence_distance = 3.0,
    int max_iteration = 50,
    double rmse_threshold = 2.0,
    bool use_point_to_plane = true);

/// <summary>
/// 将Mesh 转换为带法线的点云
/// </summary>
/// <param name="mesh">输入的三角网格</param>
/// <param name="target_density">目标密度级别 (1.0 = 自动, 0.5 = 稀疏, 2.0 = 密集)</param>
/// <returns>带法线的点云</returns>
CAM_API std::shared_ptr<open3d::geometry::PointCloud> MeshToPointCloudWithNormals(
    const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh,
    double target_density = 1.0);

//和上面的对比不做均值采样和法线估计
PointCloudPtr MeshToPointCloud(const TriangleMeshPtr& mesh);

// ============================================================================
// STL 网格文件加载（原样读取，不做任何计算）
// ============================================================================

/// <summary>
/// 从磁盘加载 STL 网格文件（原样读取，不自动计算任何信息）
/// </summary>
/// <param name="filepath">STL 文件路径（支持绝对路径和相对路径）</param>
/// <param name="print_info">是否打印详细加载信息</param>
/// <returns>三角网格智能指针，失败返回 nullptr</returns>
CAM_API TriangleMeshPtr LoadSTLMesh(
    const std::string& filepath,
    bool print_info = false);

/// <summary>
/// 通过球体裁剪点云
/// </summary>
/// <param name="cloud">点云</param>
/// <param name="center">球心</param>
/// <param name="radius">求半径</param>
/// <returns></returns>
CAM_API std::shared_ptr<open3d::geometry::PointCloud> CropPointCloudBySphere(
    const open3d::geometry::PointCloud& cloud,
    const Eigen::Vector3d& center,
	double radius);

/**
 * @brief 将点云写入文件 (根据文件后缀写入,只要是Open3D支持的格式都支持)
 */
CAM_API void Open3D_WritePoint(const std::string& filepath, const Point3D& p);

/**
 * @brief 将完整人头点云 meshCloud 配准到相机采集的单侧点云 cloudInSphere，仅输出变换矩阵
 *
 * 算法流程：
 * 1. 下采样：对两组点云进行体素下采样（2mm），减少计算量，提高鲁棒性。
 * 2. FPFH特征提取：为两组下采样点云计算 FPFH 特征，用于全局粗配准。
 * 3. 全局粗配准（RANSAC）：用特征匹配+RANSAC获得初始变换，解决初始位姿差异大、遮挡、杂点等问题。
 *    - 使用边长约束和距离约束过滤不合理的匹配。
 * 4. 精配准（Point-to-Point ICP）：以RANSAC结果为初始，进行点到面ICP精配准，获得高精度变换。
 *
 * @param meshCloud      完整人头点云（需有法线，点数大，密度约2mm）
 * @param cloudInSphere  相机采集的单侧点云（已去背景，允许有少量杂点，无法线）
 * @return 4x4 配准变换矩阵（meshCloud->Transform(T) 可将meshCloud对齐到cloudInSphere）
 */
CAM_API Eigen::Matrix4d RegisterMeshToScanTransform(
    open3d::geometry::PointCloud& meshCloud,
    open3d::geometry::PointCloud& cloudInSphere,
    FineAlignmentResult* result = nullptr);

/// <summary>
/// 多区域配准，区域顺序要一直
/// </summary>
/// <param name="meshAreas"></param>
/// <param name="camAreas"></param>
/// <returns></returns>
CAM_API Eigen::Matrix4d RegisterMultiAreaPointCloud(
    const std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& meshAreas,
    const std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& camAreas);