#include "open3d_kit.h"

#include <open3d/geometry/BoundingVolume.h> 
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/geometry/KDTreeFlann.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/io/PointCloudIO.h>
#include <opencv2/calib3d.hpp>

#include <map>
#include <iostream>

using namespace open3d::pipelines;

// ============================================================================
// 粗配准
// ============================================================================
Eigen::Matrix4d ComputeTransformationFromCorrespondences(
    const PointCloud& source,
    const PointCloud& target,
    double* mae) {

    if (source.points_.size() != target.points_.size()) {
        throw std::runtime_error("Source and target must have same size");
    }

    registration::CorrespondenceSet corr_set;
    corr_set.reserve(source.points_.size());
    for (size_t i = 0; i < source.points_.size(); ++i) {
        corr_set.emplace_back(static_cast<int>(i), static_cast<int>(i));
    }

    auto estimation = registration::TransformationEstimationPointToPoint();
    Eigen::Matrix4d transformation = estimation.ComputeTransformation(
        source, target, corr_set);
    if (mae) {
        // 这里原先返回的是 RMSE（均方根误差），按需求改为返回平均绝对误差（MAE）：
        // MAE = (1/N) * Σ ||T * p_i - q_i||
        // 先把源点云按估计的变换变换，然后计算对应点的平均绝对距离。
        auto source_transformed = source;
        source_transformed.Transform(transformation);

        double sum_abs = 0.0;
        const size_t n = source_transformed.points_.size();
        for (size_t i = 0; i < n; ++i) {
            sum_abs += (source_transformed.points_[i] - target.points_[i]).norm();
        }
        *mae = n > 0 ? (sum_abs / static_cast<double>(n)) : 0.0;
    }
    return transformation;
}

// ============================================================================
// 两策略自适应精配准 (Point-to-Plane / Point-to-Point)
// ============================================================================
FineAlignmentResult FineAlignment(
    const PointCloudPtr& source_cloud2,
    const PointCloudPtr& target_cloud2,
    double target_voxel_size,
    double max_correspondence_distance,
    int max_iteration,
    double rmse_threshold,
    bool use_point_to_plane) {

    using namespace open3d;
    using namespace open3d::pipelines::registration;

    // ------------------------------------------------------------------------
    // 步骤 1: 输入验证
    // ------------------------------------------------------------------------
    if (!source_cloud2 || !target_cloud2) {
        throw std::invalid_argument("[FineAlignment] Null input");
    }

    if (source_cloud2->IsEmpty() || target_cloud2->IsEmpty()) {
        throw std::runtime_error("[FineAlignment] Empty input");
    }

    std::cout << "\n========== Adaptive ICP (Estimate + Correct Normals) ==========\n";
    std::cout << "[Step 1] Input:\n";
    std::cout << "  Source: " << source_cloud2->points_.size() << " points (3D mesh)";
    if (source_cloud2->HasNormals()) std::cout << " [with normals]";
    if (source_cloud2->HasColors()) std::cout << " [with colors]";
    std::cout << "\n";

    std::cout << "  Target: " << target_cloud2->points_.size() << " points (RGB-D scan)";
    if (target_cloud2->HasColors()) std::cout << " [with colors]";
    std::cout << "\n";

    // ------------------------------------------------------------------------
    // 步骤 2: 下采样
    // ------------------------------------------------------------------------
    std::cout << "\n[Step 2] Downsampling:\n";
    std::cout << "  Source: Keep original\n";

    PointCloudPtr source_for_icp = source_cloud2;

    std::cout << "  Target: Voxel downsampling (voxel=" << target_voxel_size << "mm)...\n";
    auto target_down = target_cloud2->VoxelDownSample(target_voxel_size);
    std::cout << "    Before: " << target_cloud2->points_.size()
        << " -> After: " << target_down->points_.size() << " points\n";

    // ------------------------------------------------------------------------
    // 步骤 3: 统计滤波
    // ------------------------------------------------------------------------
    std::cout << "\n[Step 3] Statistical outlier removal:\n";
    auto [target_clean, inlier_indices] = target_down->RemoveStatisticalOutliers(30, 2.0);

    size_t removed = target_down->points_.size() - target_clean->points_.size();
    std::cout << "  Removed: " << removed << " outliers ("
        << std::fixed << std::setprecision(1)
        << (removed * 100.0 / target_down->points_.size()) << "%)\n";

    target_down = target_clean;

    // ------------------------------------------------------------------------
    // 步骤 4: 评估粗配准质量
    // ------------------------------------------------------------------------
    std::cout << "\n[Step 4] Assessing coarse alignment quality...\n";

    geometry::KDTreeFlann kdtree_assess;
    kdtree_assess.SetGeometry(*target_down);

    double avg_distance = 0.0;
    size_t sample_count = std::min<size_t>(1000, source_for_icp->points_.size());
    size_t sample_step = std::max<size_t>(1, source_for_icp->points_.size() / sample_count);
    size_t actual_samples = 0;
    for (size_t i = 0; i < source_for_icp->points_.size(); i += sample_step) {
        if (i >= source_for_icp->points_.size()) break;

        std::vector<int> indices(1);
        std::vector<double> distances(1);

        if (kdtree_assess.SearchKNN(source_for_icp->points_[i], 1, indices, distances) > 0) {
            avg_distance += std::sqrt(distances[0]);
            actual_samples++;
        }
    }
    if (actual_samples > 0)
        avg_distance /= actual_samples;

    // ------------------------------------------------------------------------
    // 步骤 5: 策略选择
    // ------------------------------------------------------------------------
    registration::RegistrationResult result;
    ICPConvergenceCriteria criteria;
    criteria.max_iteration_ = max_iteration;
    criteria.relative_fitness_ = 1e-6;
    criteria.relative_rmse_ = 1e-6;

    if (use_point_to_plane) {
        // ====================================================================
        // 策略 1: Point-to-Plane ICP (估计+校正法向量)
        // ====================================================================
        std::cout << "  Algorithm: Point-to-Plane ICP\n";
        std::cout << "  Normal handling: Estimate + Correct\n\n";

        // 检查源法向量
        if (!source_for_icp->HasNormals()) {
            source_for_icp->EstimateNormals(
                geometry::KDTreeSearchParamHybrid(target_voxel_size * 3, 50));
        }

        // 步骤 5.1: 估计目标法向量
        std::cout << "[Step 5.1] Estimating target normals via PCA...\n";
        std::cout << "  Search radius: " << (target_voxel_size * 3) << " mm\n";
        std::cout << "  Max neighbors: 50 points\n";

        if (!target_down->HasNormals()) {
            target_down->EstimateNormals(
                geometry::KDTreeSearchParamHybrid(target_voxel_size * 3, 50));
        }

        std::cout << "  [OK] Target normals estimated (direction may be random)\n";

        // 步骤 5.2: 校正法向量方向
        std::cout << "\n[Step 5.2] Correcting normal directions...\n";
        std::cout << "  Strategy: Use nearest source normal to guide direction\n";

        geometry::KDTreeFlann kdtree_correct;
        kdtree_correct.SetGeometry(*source_for_icp);

        size_t flipped_count = 0;
        size_t corrected_count = 0;

        for (size_t i = 0; i < target_down->points_.size(); ++i) {
            std::vector<int> indices(1);
            std::vector<double> distances(1);

            if (kdtree_correct.SearchKNN(target_down->points_[i], 1, indices, distances) > 0) {
                const Eigen::Vector3d& source_normal = source_for_icp->normals_[indices[0]];
                Eigen::Vector3d& target_normal = target_down->normals_[i];

                double dot_product = target_normal.dot(source_normal);

                if (dot_product < 0) {
                    target_normal = -target_normal;
                    flipped_count++;
                }

                corrected_count++;
            }
        }

        std::cout << "  Corrected: " << corrected_count << " / "
            << target_down->points_.size() << " normals\n";
        std::cout << "  Flipped:   " << flipped_count << " normals ("
            << std::fixed << std::setprecision(1)
            << (flipped_count * 100.0 / corrected_count) << "%)\n";
        std::cout << "  [OK] All target normals now consistent with source\n";

        std::cout << "\n  Advantage of this strategy:\n";
        std::cout << "    - Target normals reflect actual local geometry\n";
        std::cout << "    - No position error from direct propagation\n";
        std::cout << "    - Direction corrected using accurate source normals\n";

        // Point-to-Plane ICP
        std::cout << "\n[Step 6] Running Point-to-Plane ICP...\n";
        std::cout << "  Max iterations: " << max_iteration << "\n";
        std::cout << "  Max correspondence distance: " << max_correspondence_distance << " mm\n";

        result = RegistrationICP(
            *source_for_icp,
            *target_down,
            max_correspondence_distance,
            Eigen::Matrix4d::Identity(),
            TransformationEstimationPointToPlane(),
            criteria
        );

    }
    else {
        // ====================================================================
        // 策略 2: Point-to-Point ICP (不使用法向量)
        // ====================================================================
        result = RegistrationICP(
            *source_for_icp,
            *target_down,
            max_correspondence_distance,
            Eigen::Matrix4d::Identity(),
            TransformationEstimationPointToPoint(),
            criteria
        );
    }

    // ------------------------------------------------------------------------
    // 步骤 7: 结果评估
    // ------------------------------------------------------------------------
    bool converged = (result.fitness_ > 0.3) && (result.inlier_rmse_ < rmse_threshold);

    std::cout << "\n========== Result ==========\n";
    std::cout << "  Algorithm: " << (use_point_to_plane ? "Point-to-Plane" : "Point-to-Point") << "\n";
    std::cout << "  Fitness:   " << std::fixed << std::setprecision(4) << result.fitness_;

    if (result.fitness_ > 0.5) std::cout << " [Excellent]\n";
    else if (result.fitness_ > 0.3) std::cout << " [Good]\n";
    else if (result.fitness_ > 0.2) std::cout << " [Acceptable]\n";
    else std::cout << " [Poor]\n";

    std::cout << "  RMSE:      " << std::fixed << std::setprecision(3)
        << result.inlier_rmse_ << " mm";

    if (result.inlier_rmse_ < 1.0) std::cout << " [Excellent - Sub-mm]\n";
    else if (result.inlier_rmse_ < rmse_threshold) std::cout << " [Good]\n";
    else std::cout << " [Failed]\n";

    std::cout << "  Overall:   " << (converged ? "[SUCCESS]" : "[FAILED]") << "\n";

    if (!converged) {
        std::cout << "\n[DIAGNOSTIC]\n";
        if (result.inlier_rmse_ >= rmse_threshold) {
            std::cout << "  High RMSE - Try:\n";
            std::cout << "    1. Re-select initial 3 points\n";
            std::cout << "    2. Increase max_correspondence_distance to "
                << (max_correspondence_distance * 1.5) << " mm\n";
        }
        if (result.fitness_ <= 0.3) {
            std::cout << "  Low fitness - Try:\n";
            std::cout << "    1. Verify initial alignment\n";
            std::cout << "    2. Expand bounding sphere radius\n";
        }
    }

    std::cout << "============================\n\n";

    FineAlignmentResult output;
    output.transformation = result.transformation_;
    output.fitness = result.fitness_;
    output.inlier_rmse = result.inlier_rmse_;
    output.converged = converged;

    return output;
}

std::shared_ptr<open3d::geometry::PointCloud> MeshToPointCloudWithNormals(
    const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh,
    double target_density) {

    if (mesh == nullptr || mesh->IsEmpty()) {
        std::cerr << "错误: 输入的 Mesh 为空或无效" << std::endl;
        return nullptr;
    }

    std::cout << "\n========== Mesh 转点云（带法线，Mesh法线最近邻传递）==========" << std::endl;
    std::cout << "输入 Mesh 信息:" << std::endl;
    std::cout << "  顶点数: " << mesh->vertices_.size() << std::endl;
    std::cout << "  三角形数: " << mesh->triangles_.size() << std::endl;

    // 1. 计算采样点数
    auto bbox = mesh->GetAxisAlignedBoundingBox();
    Eigen::Vector3d extent = bbox.GetExtent();
    double surface_area = 2.0 * (
        extent.x() * extent.y() +
        extent.y() * extent.z() +
        extent.z() * extent.x()
        );
    size_t target_points = static_cast<size_t>(surface_area * 300.0 * target_density);
    target_points = std::max(size_t(10000), std::min(target_points, size_t(500000)));

    std::cout << "\n采样设置:" << std::endl;
    std::cout << "  包围盒尺寸: " << extent.transpose() << std::endl;
    std::cout << "  估算表面积: " << surface_area << " 平方单位" << std::endl;
    std::cout << "  目标采样点数: " << target_points << std::endl;

    // 2. 确保 Mesh 有顶点法线
    if (!mesh->HasVertexNormals()) {
        std::cout << "  正在计算 Mesh 顶点法线..." << std::endl;
        mesh->ComputeVertexNormals();
    }

    // 3. 均匀采样点云
    std::cout << "\n正在均匀采样点云..." << std::endl;
    auto pointcloud = mesh->SamplePointsUniformly(target_points);

    if (!pointcloud || pointcloud->IsEmpty()) {
        std::cerr << "错误: 点云采样失败" << std::endl;
        return nullptr;
    }
    std::cout << "  采样成功: " << pointcloud->points_.size() << " 个点" << std::endl;

    // 4. 用KDTree将Mesh顶点法线最近邻传递到采样点
    std::cout << "\n正在传递Mesh顶点法线到点云..." << std::endl;
    open3d::geometry::KDTreeFlann kdtree(*mesh);
    pointcloud->normals_.resize(pointcloud->points_.size());

    for (size_t i = 0; i < pointcloud->points_.size(); ++i) {
        std::vector<int> indices(1);
        std::vector<double> dists(1);
        if (kdtree.SearchKNN(pointcloud->points_[i], 1, indices, dists) > 0) {
            pointcloud->normals_[i] = mesh->vertex_normals_[indices[0]];
        }
        else {
            pointcloud->normals_[i] = Eigen::Vector3d(0, 0, 1); // fallback
        }
    }
    std::cout << "  法线传递完成" << std::endl;

    // 5. 颜色处理
    if (!pointcloud->HasColors()) {
        pointcloud->PaintUniformColor(Eigen::Vector3d(0.7, 0.7, 0.7));
        std::cout << "  已添加默认颜色" << std::endl;
    }
    else {
        std::cout << "  已保留颜色信息" << std::endl;
    }

    // 6. 验证法线
    int invalid_normals = 0;
    for (const auto& n : pointcloud->normals_) {
        double norm = n.norm();
        if (norm < 0.9 || norm > 1.1) invalid_normals++;
    }
    std::cout << "\n法线验证:" << std::endl;
    std::cout << "  法线总数: " << pointcloud->normals_.size() << std::endl;

    std::cout << "\n转换完成!" << std::endl;
    std::cout << "  最终点数: " << pointcloud->points_.size() << std::endl;
    std::cout << "  法线数: " << pointcloud->normals_.size() << std::endl;
    std::cout << "  颜色数: " << pointcloud->colors_.size() << std::endl;
    std::cout << "======================================\n" << std::endl;

    return pointcloud;
}

PointCloudPtr MeshToPointCloud(const TriangleMeshPtr& mesh)
{
    // 直接从 mesh 顶点构建点云，保证顶点索引与 mesh 一致
    auto meshPointCloud = std::make_shared<open3d::geometry::PointCloud>();
    meshPointCloud->points_ = mesh->vertices_;
    if (mesh->HasVertexNormals()) {
        meshPointCloud->normals_ = mesh->vertex_normals_;
    }
    if (mesh->HasVertexColors()) {
        meshPointCloud->colors_ = mesh->vertex_colors_;
    }
    return meshPointCloud;
}

namespace {  // 匿名命名空间，仅在本文件内可见

    /// <summary>
    /// 手动解析 ASCII STL 文件
    /// </summary>
    std::shared_ptr<open3d::geometry::TriangleMesh> ParseASCIISTL(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            return nullptr;
        }

        auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        std::string line;

        // 临时存储当前三角形的三个顶点
        std::vector<Eigen::Vector3d> temp_vertices;
        temp_vertices.reserve(3);

        // 用于顶点去重的 map (坐标 -> 索引)
        std::map<std::tuple<double, double, double>, size_t> vertex_map;

        std::cout << "  [ASCII 解析] 开始逐行解析..." << std::endl;

        size_t line_number = 0;
        size_t triangle_count = 0;
        bool in_facet = false;

        while (std::getline(file, line)) {
            line_number++;

            // 去除前后空格
            line.erase(0, line.find_first_not_of(" \t\r\n"));
            line.erase(line.find_last_not_of(" \t\r\n") + 1);

            if (line.empty()) continue;

            // 转换为小写以便比较
            std::string line_lower = line;
            std::transform(line_lower.begin(), line_lower.end(), line_lower.begin(), ::tolower);

            // 解析关键字
            if (line_lower.find("solid") == 0) {
                std::cout << "  发现 'solid' 标记 (行 " << line_number << ")" << std::endl;
                continue;
            }
            else if (line_lower.find("facet") == 0) {
                in_facet = true;
                temp_vertices.clear();
                // 可选: 解析法向量 (facet normal nx ny nz)
                continue;
            }
            else if (line_lower.find("vertex") == 0) {
                if (!in_facet) {
                    std::cerr << "  [警告] 行 " << line_number << ": 'vertex' 出现在 'facet' 外" << std::endl;
                    continue;
                }

                // 解析顶点坐标: vertex x y z
                std::istringstream iss(line);
                std::string keyword;
                double x, y, z;

                if (iss >> keyword >> x >> y >> z) {
                    temp_vertices.emplace_back(x, y, z);
                }
                else {
                    std::cerr << "  [错误] 行 " << line_number << ": 无法解析顶点坐标: " << line << std::endl;
                }
                continue;
            }
            else if (line_lower.find("endfacet") == 0) {
                if (!in_facet) {
                    std::cerr << "  [警告] 行 " << line_number << ": 'endfacet' 没有对应的 'facet'" << std::endl;
                    continue;
                }

                // 验证是否有3个顶点
                if (temp_vertices.size() != 3) {
                    std::cerr << "  [错误] 行 " << line_number << ": 三角形顶点数不是3 (实际: "
                        << temp_vertices.size() << ")" << std::endl;
                    in_facet = false;
                    continue;
                }

                // 添加三角形（支持顶点去重）
                Eigen::Vector3i triangle;
                for (size_t i = 0; i < 3; ++i) {
                    const auto& v = temp_vertices[i];

                    // 创建顶点键（用于去重）
                    auto key = std::make_tuple(v.x(), v.y(), v.z());

                    auto it = vertex_map.find(key);
                    if (it != vertex_map.end()) {
                        // 顶点已存在，重用索引
                        triangle[i] = static_cast<int>(it->second);
                    }
                    else {
                        // 新顶点
                        size_t new_index = mesh->vertices_.size();
                        mesh->vertices_.push_back(v);
                        vertex_map[key] = new_index;
                        triangle[i] = static_cast<int>(new_index);
                    }
                }

                mesh->triangles_.push_back(triangle);
                triangle_count++;

                in_facet = false;
                continue;
            }
            else if (line_lower.find("endsolid") == 0) {
                std::cout << "  发现 'endsolid' 标记 (行 " << line_number << ")" << std::endl;
                break;
            }
        }

        file.close();

        std::cout << "  [ASCII 解析] 完成" << std::endl;
        std::cout << "    解析行数: " << line_number << std::endl;
        std::cout << "    三角形数: " << triangle_count << std::endl;
        std::cout << "    顶点数: " << mesh->vertices_.size() << std::endl;

        if (mesh->triangles_.empty() || mesh->vertices_.empty()) {
            std::cerr << "  [错误] 未解析到有效几何数据" << std::endl;
            return nullptr;
        }

        return mesh;
    }

    /// <summary>
    /// 手动解析 Binary STL 文件
    /// </summary>
    std::shared_ptr<open3d::geometry::TriangleMesh> ParseBinarySTL(const std::string& filepath) {
        std::ifstream file(filepath, std::ios::binary);
        if (!file.is_open()) {
            return nullptr;
        }

        auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();

        std::cout << "  [Binary 解析] 开始解析..." << std::endl;

        // 1. 读取 80 字节头（通常是注释，忽略）
        char header[80];
        file.read(header, 80);

        if (!file) {
            std::cerr << "  [错误] 无法读取文件头" << std::endl;
            return nullptr;
        }

        // 2. 读取三角形数量（4 字节无符号整数，小端序）
        uint32_t triangle_count = 0;
        file.read(reinterpret_cast<char*>(&triangle_count), sizeof(uint32_t));

        if (!file) {
            std::cerr << "  [错误] 无法读取三角形数量" << std::endl;
            return nullptr;
        }

        std::cout << "    三角形数量: " << triangle_count << std::endl;

        if (triangle_count == 0) {
            std::cerr << "  [错误] 三角形数量为0" << std::endl;
            return nullptr;
        }

        if (triangle_count > 100000000) {  // 1亿三角形（防止内存溢出）
            std::cerr << "  [错误] 三角形数量异常大: " << triangle_count << std::endl;
            return nullptr;
        }

        // 预分配空间
        mesh->triangles_.reserve(triangle_count);

        // 用于顶点去重
        std::map<std::tuple<float, float, float>, size_t> vertex_map;

        // 3. 逐个读取三角形
        for (uint32_t i = 0; i < triangle_count; ++i) {
            // Binary STL 三角形格式（每个 50 字节）:
            // - 法向量: 3 个 float (12 字节)
            // - 顶点1:  3 个 float (12 字节)
            // - 顶点2:  3 个 float (12 字节)
            // - 顶点3:  3 个 float (12 字节)
            // - 属性:   uint16_t  (2 字节，通常未使用)

            // 读取法向量（暂时忽略，后续可重新计算）
            float normal[3];
            file.read(reinterpret_cast<char*>(normal), 3 * sizeof(float));

            // 读取三个顶点
            float vertices[3][3];
            file.read(reinterpret_cast<char*>(vertices), 9 * sizeof(float));

            // 读取属性字节（忽略）
            uint16_t attribute;
            file.read(reinterpret_cast<char*>(&attribute), sizeof(uint16_t));

            if (!file) {
                std::cerr << "  [错误] 读取三角形 " << i << " 时失败" << std::endl;
                return nullptr;
            }

            // 添加三角形（支持顶点去重）
            Eigen::Vector3i triangle;

            for (int j = 0; j < 3; ++j) {
                float x = vertices[j][0];
                float y = vertices[j][1];
                float z = vertices[j][2];

                // 检查 NaN/Inf
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                    std::cerr << "  [警告] 三角形 " << i << " 顶点 " << j
                        << " 包含无效值 (NaN/Inf)" << std::endl;
                    // 用 0 替代
                    x = y = z = 0.0f;
                }

                // 创建顶点键（用于去重）
                auto key = std::make_tuple(x, y, z);

                auto it = vertex_map.find(key);
                if (it != vertex_map.end()) {
                    // 顶点已存在
                    triangle[j] = static_cast<int>(it->second);
                }
                else {
                    // 新顶点
                    size_t new_index = mesh->vertices_.size();
                    mesh->vertices_.emplace_back(x, y, z);
                    vertex_map[key] = new_index;
                    triangle[j] = static_cast<int>(new_index);
                }
            }

            mesh->triangles_.push_back(triangle);

            // 每解析 10000 个三角形打印一次进度
            if ((i + 1) % 10000 == 0) {
                std::cout << "    进度: " << (i + 1) << " / " << triangle_count << std::endl;
            }
        }

        file.close();

        std::cout << "  [Binary 解析] 完成" << std::endl;
        std::cout << "    三角形数: " << mesh->triangles_.size() << std::endl;
        std::cout << "    顶点数: " << mesh->vertices_.size() << " (去重后)" << std::endl;

        if (mesh->triangles_.empty() || mesh->vertices_.empty()) {
            std::cerr << "  [错误] 未解析到有效几何数据" << std::endl;
            return nullptr;
        }

        return mesh;
    }
}

TriangleMeshPtr LoadSTLMesh(
    const std::string& filepath,
    bool print_info) {

    // ------------------------------------------------------------------------
    // 步骤 1: 文件路径验证
    // ------------------------------------------------------------------------
    std::filesystem::path file_path(filepath);

    if (!std::filesystem::exists(file_path)) {
        std::cerr << "\n[错误] 文件不存在: " << filepath << std::endl;
        std::cerr << "当前工作目录: " << std::filesystem::current_path() << std::endl;
        return nullptr;
    }

    // ------------------------------------------------------------------------
    // 步骤 2: 文件格式验证
    // ------------------------------------------------------------------------
    std::string extension = file_path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

    if (extension != ".stl") {
        std::cerr << "\n[错误] 不支持的文件格式: " << extension << std::endl;
        std::cerr << "支持的格式: .stl (ASCII 或 Binary)" << std::endl;
        return nullptr;
    }

    // ------------------------------------------------------------------------
    // 步骤 3: 文件完整性检查
    // ------------------------------------------------------------------------
    std::ifstream test_file(file_path, std::ios::binary);
    if (!test_file.is_open()) {
        std::cerr << "\n[错误] 无法打开文件（权限不足或文件被占用）: " << filepath << std::endl;
        return nullptr;
    }

    // 检查文件是否为空
    test_file.seekg(0, std::ios::end);
    size_t file_size = test_file.tellg();
    test_file.seekg(0, std::ios::beg);

    if (file_size == 0) {
        std::cerr << "\n[错误] 文件为空: " << filepath << std::endl;
        test_file.close();
        return nullptr;
    }

    // 检测 STL 格式类型
    char header[6] = { 0 };
    test_file.read(header, 5);
    test_file.seekg(0, std::ios::beg);

    bool is_ascii = (std::string(header) == "solid");
    bool format_valid = true;

    if (is_ascii) {
        // ASCII STL 格式验证
        std::string first_line;
        std::getline(test_file, first_line);

        if (first_line.find("solid") == std::string::npos) {
            format_valid = false;
        }
    }
    else {
        // Binary STL 格式验证
        if (file_size < 84) {
            std::cerr << "\n[错误] Binary STL 文件太小 (< 84 字节): " << file_size << " 字节" << std::endl;
            test_file.close();
            return nullptr;
        }

        // 读取三角形数量
        test_file.seekg(80, std::ios::beg);
        uint32_t triangle_count = 0;
        test_file.read(reinterpret_cast<char*>(&triangle_count), sizeof(uint32_t));

        // 计算预期文件大小: 80(头) + 4(数量) + 50*N(三角形)
        size_t expected_size = 84 + triangle_count * 50;

        if (file_size != expected_size) {
            std::cerr << "\n[警告] Binary STL 文件大小不匹配" << std::endl;
            std::cerr << "  实际大小: " << file_size << " 字节" << std::endl;
            std::cerr << "  预期大小: " << expected_size << " 字节" << std::endl;
            std::cerr << "  三角形数: " << triangle_count << std::endl;
        }
    }

    test_file.close();

    // ------------------------------------------------------------------------
    // 步骤 4: 打印基本信息（可选）
    // ------------------------------------------------------------------------
    if (print_info) {
        std::cout << "\n========== 加载 STL 网格文件 ==========" << std::endl;
        std::cout << "文件路径: " << std::filesystem::absolute(file_path) << std::endl;

        // 文件大小
        std::cout << "文件大小: ";
        if (file_size < 1024) {
            std::cout << file_size << " B";
        }
        else if (file_size < 1024 * 1024) {
            std::cout << std::fixed << std::setprecision(2)
                << (file_size / 1024.0) << " KB";
        }
        else {
            std::cout << std::fixed << std::setprecision(2)
                << (file_size / 1024.0 / 1024.0) << " MB";
        }
        std::cout << std::endl;

        std::cout << "STL 格式: " << (is_ascii ? "ASCII" : "Binary") << std::endl;

        if (!format_valid) {
            std::cerr << "[警告] STL 格式可能不规范" << std::endl;
        }

        std::cout << "\n正在读取..." << std::endl;
    }

    // ------------------------------------------------------------------------
    // 步骤 5: 尝试多种方式读取 STL 文件
    // ------------------------------------------------------------------------
    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();

    bool success = false;

    // 方法 1: 使用默认参数读取
    try {
        success = open3d::io::ReadTriangleMesh(filepath, *mesh);
    }
    catch (const std::exception& e) {
        std::cerr << "\n[方法1失败] 读取文件时发生异常: " << e.what() << std::endl;
    }

    // 方法 2: 如果方法1失败，尝试使用 ReadTriangleMeshOptions
    if (!success || mesh->IsEmpty()) {
        if (print_info) {
            std::cout << "\n[尝试方法2] 使用后处理选项..." << std::endl;
        }

        mesh = std::make_shared<open3d::geometry::TriangleMesh>();

        open3d::io::ReadTriangleMeshOptions options;
        options.enable_post_processing = true;
        options.print_progress = print_info;

        try {
            success = open3d::io::ReadTriangleMesh(filepath, *mesh, options);
        }
        catch (const std::exception& e) {
            std::cerr << "\n[方法2失败] 读取文件时发生异常: " << e.what() << std::endl;
        }
    }

    // 方法 3: 手动解析 STL 文件（最后的回退方案）
    if (!success || mesh->IsEmpty()) {
        if (print_info) {
            std::cout << "\n[尝试方法3] 手动解析 STL 文件..." << std::endl;
        }

        if (is_ascii) {
            mesh = ParseASCIISTL(filepath);
        }
        else {
            mesh = ParseBinarySTL(filepath);
        }

        if (mesh && !mesh->IsEmpty()) {
            success = true;
            if (print_info) {
                std::cout << "  [手动解析成功]" << std::endl;
            }
        }
        else {
            std::cerr << "\n[方法3失败] 手动解析失败" << std::endl;
        }
    }

    if (!success || !mesh || mesh->IsEmpty()) {
        std::cerr << "\n[错误] 所有读取方法均失败" << std::endl;
        std::cerr << "可能原因:" << std::endl;
        std::cerr << "  1. STL 文件格式严重损坏" << std::endl;
        std::cerr << "  2. 文件包含非法几何数据" << std::endl;
        std::cerr << "  3. 文件编码问题" << std::endl;
        return nullptr;
    }

    // ------------------------------------------------------------------------
    // 步骤 6: 验证基本数据
    // ------------------------------------------------------------------------
    if (mesh->vertices_.empty()) {
        std::cerr << "\n[错误] 网格没有顶点数据" << std::endl;
        return nullptr;
    }

    if (mesh->triangles_.empty()) {
        std::cerr << "\n[错误] 网格没有三角形数据" << std::endl;
        return nullptr;
    }

    // ------------------------------------------------------------------------
    // 步骤 7: 自动修复网格
    // ------------------------------------------------------------------------
    if (print_info) {
        std::cout << "\n========== 网格修复 ==========" << std::endl;
        std::cout << "修复前状态:" << std::endl;
        std::cout << "  顶点数: " << mesh->vertices_.size() << std::endl;
        std::cout << "  三角形数: " << mesh->triangles_.size() << std::endl;
    }

    // 修复 1: 移除重复顶点
    if (print_info) {
        std::cout << "\n[修复 1/3] 移除重复顶点..." << std::endl;
    }

    size_t original_vertex_count = mesh->vertices_.size();
    mesh->RemoveDuplicatedVertices();
    size_t removed_vertices = original_vertex_count - mesh->vertices_.size();

    if (print_info) {
        std::cout << "  移除了 " << removed_vertices << " 个重复顶点 ("
            << std::fixed << std::setprecision(2)
            << (removed_vertices * 100.0 / original_vertex_count) << "%)" << std::endl;
    }

    // 修复 2: 移除退化三角形
    if (print_info) {
        std::cout << "\n[修复 2/3] 移除零面积三角形..." << std::endl;
    }

    size_t original_triangle_count = mesh->triangles_.size();
    mesh->RemoveDegenerateTriangles();
    size_t removed_degenerate = original_triangle_count - mesh->triangles_.size();

    if (print_info) {
        std::cout << "  移除了 " << removed_degenerate << " 个退化三角形 ("
            << std::fixed << std::setprecision(2)
            << (removed_degenerate * 100.0 / original_triangle_count) << "%)" << std::endl;
    }

    // 修复 3: 移除非流形几何
    if (print_info) {
        std::cout << "\n[修复 3/3] 移除非流形几何..." << std::endl;
    }

    size_t triangle_count_before_manifold = mesh->triangles_.size();
    mesh->RemoveNonManifoldEdges();
    size_t removed_non_manifold_edges = triangle_count_before_manifold - mesh->triangles_.size();

    if (print_info && removed_non_manifold_edges > 0) {
        std::cout << "  移除了 " << removed_non_manifold_edges << " 个非流形边相关三角形" << std::endl;
    }

    // 清理未引用顶点
    if (print_info) {
        std::cout << "\n[清理] 移除未引用顶点..." << std::endl;
    }

    size_t vertex_count_before_cleanup = mesh->vertices_.size();
    mesh->RemoveUnreferencedVertices();
    size_t removed_unreferenced = vertex_count_before_cleanup - mesh->vertices_.size();

    if (print_info) {
        std::cout << "  移除了 " << removed_unreferenced << " 个未引用顶点" << std::endl;
    }

    // ------------------------------------------------------------------------
    // 步骤 8: 数据完整性检查
    // ------------------------------------------------------------------------
    bool has_invalid_data = false;

    for (const auto& v : mesh->vertices_) {
        if (!std::isfinite(v.x()) || !std::isfinite(v.y()) || !std::isfinite(v.z())) {
            has_invalid_data = true;
            break;
        }
    }

    for (const auto& tri : mesh->triangles_) {
        if (tri[0] >= mesh->vertices_.size() ||
            tri[1] >= mesh->vertices_.size() ||
            tri[2] >= mesh->vertices_.size()) {
            has_invalid_data = true;
            break;
        }
    }

    if (has_invalid_data) {
        std::cerr << "\n[警告] 修复后仍检测到无效数据（NaN/Inf 或越界索引）" << std::endl;
    }

    // ------------------------------------------------------------------------
    // 步骤 9: 打印修复后信息
    // ------------------------------------------------------------------------
    if (print_info) {
        std::cout << "\n修复后状态:" << std::endl;
        std::cout << "  顶点数: " << mesh->vertices_.size() << std::endl;
        std::cout << "  三角形数: " << mesh->triangles_.size() << std::endl;

        size_t total_removed_vertices = original_vertex_count - mesh->vertices_.size();
        size_t total_removed_triangles = original_triangle_count - mesh->triangles_.size();

        std::cout << "\n修复统计:" << std::endl;
        std::cout << "  总共移除顶点: " << total_removed_vertices << " ("
            << std::fixed << std::setprecision(2)
            << (total_removed_vertices * 100.0 / original_vertex_count) << "%)" << std::endl;
        std::cout << "  总共移除三角形: " << total_removed_triangles << " ("
            << std::fixed << std::setprecision(2)
            << (total_removed_triangles * 100.0 / original_triangle_count) << "%)" << std::endl;

        std::cout << "\n文件中包含的属性:" << std::endl;
        std::cout << "  顶点法线: " << (mesh->HasVertexNormals() ? "[有]" : "[无]") << std::endl;
        std::cout << "  三角形法线: " << (mesh->HasTriangleNormals() ? "[有]" : "[无]") << std::endl;
        std::cout << "  顶点颜色: " << (mesh->HasVertexColors() ? "[有]" : "[无]") << std::endl;
        std::cout << "  纹理坐标: " << (mesh->HasTriangleUvs() ? "[有]" : "[无]") << std::endl;

        auto bbox = mesh->GetAxisAlignedBoundingBox();
        Eigen::Vector3d min_bound = bbox.min_bound_;
        Eigen::Vector3d max_bound = bbox.max_bound_;
        Eigen::Vector3d extent = bbox.GetExtent();
        Eigen::Vector3d center = bbox.GetCenter();

        std::cout << "\n包围盒信息:" << std::endl;
        std::cout << "  最小点: [" << std::fixed << std::setprecision(2)
            << min_bound.x() << ", " << min_bound.y() << ", " << min_bound.z() << "]" << std::endl;
        std::cout << "  最大点: ["
            << max_bound.x() << ", " << max_bound.y() << ", " << max_bound.z() << "]" << std::endl;
        std::cout << "  中心: ["
            << center.x() << ", " << center.y() << ", " << center.z() << "]" << std::endl;
        std::cout << "  尺寸: ["
            << extent.x() << ", " << extent.y() << ", " << extent.z() << "]" << std::endl;

        std::cout << "\n[成功] 加载并修复完成" << std::endl;
        std::cout << "======================================\n" << std::endl;
    }

    return mesh;
}

std::shared_ptr<open3d::geometry::PointCloud> CropPointCloudBySphere(
    const open3d::geometry::PointCloud& cloud,
    const Eigen::Vector3d& center,
    double radius) {
    std::vector<size_t> indices;
    for (size_t i = 0; i < cloud.points_.size(); ++i) {
        if ((cloud.points_[i] - center).norm() <= radius) {
            indices.push_back(i);
        }
    }
    return cloud.SelectByIndex(indices);
}

void Open3D_WritePoint(const std::string& filepath, const Point3D& p)
{
    open3d::geometry::PointCloud pointCloud({ p });
    open3d::io::WritePointCloud(filepath, pointCloud);
}

Eigen::Matrix4d RegisterMeshToScanTransform(
    open3d::geometry::PointCloud& meshCloud,
    open3d::geometry::PointCloud& cloudInSphere,
    FineAlignmentResult* result)
{
    using namespace open3d;
    using namespace open3d::pipelines::registration;

    double voxel_size = 2.0;
    // 输入检查
    if (meshCloud.IsEmpty() || cloudInSphere.IsEmpty()) {
        throw std::runtime_error("输入点云为空");
    }
    if (!meshCloud.HasNormals()) {
        meshCloud.EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 3, 50));
    }

    // 1. 下采样（2mm分辨率，减少计算量，提升鲁棒性）
    auto meshCloud_ds = meshCloud.VoxelDownSample(voxel_size);
    auto cloudInSphere_ds = cloudInSphere.VoxelDownSample(voxel_size);

    // 2. 估计目标点云法线
    if(!cloudInSphere_ds->HasNormals())
        cloudInSphere_ds->EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 3, 50));

    // 3. FPFH特征提取（为全局粗配准做准备）
    auto mesh_fpfh = pipelines::registration::ComputeFPFHFeature(
        *meshCloud_ds, open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 5, 100));
    auto scan_fpfh = pipelines::registration::ComputeFPFHFeature(
        *cloudInSphere_ds, open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 5, 100));

    // 4. 全局粗配准（RANSAC+特征匹配，解决初始位姿、遮挡、杂点等问题）
    //    - 边长约束：防止大尺度拉伸/缩放
    //    - 距离约束：防止空间误匹配
    std::vector<std::reference_wrapper<const CorrespondenceChecker>> checkers;
    auto checker_edge = CorrespondenceCheckerBasedOnEdgeLength(0.9);
    auto checker_dist = CorrespondenceCheckerBasedOnDistance(voxel_size * 1.5);
    checkers.push_back(checker_edge);
    checkers.push_back(checker_dist);

    auto result_ransac = RegistrationRANSACBasedOnFeatureMatching(
        *meshCloud_ds,                // 源点云（下采样后）
        *cloudInSphere_ds,            // 目标点云（下采样后）
        *mesh_fpfh,                   // 源点云的FPFH特征
        *scan_fpfh,                   // 目标点云的FPFH特征
        true,                         // mutual_filter，是否互为最近邻（True更鲁棒，减少误匹配）
        voxel_size * 1.5,             // max_correspondence_distance，最大对应点距离（单位mm）
        TransformationEstimationPointToPoint(false), // 变换估计方法（点到点，不考虑缩放）
        3,                            // ransac_n，每次RANSAC采样的点对数（通常为3）
        checkers,                     // 检查器列表（如边长约束、距离约束）
        RANSACConvergenceCriteria(100000, 0.999) // 收敛准则（最大迭代次数，目标置信度）
    );

    // 5. 精配准（Point-to-Point ICP，不需要法线）
    //    - 以RANSAC结果为初始变换
    auto result_icp = RegistrationICP(
        *meshCloud_ds, *cloudInSphere_ds,
        voxel_size * 1.0,
        result_ransac.transformation_,
        TransformationEstimationPointToPoint(),
        ICPConvergenceCriteria()
    );

    if (result) {
        result->fitness = result_icp.fitness_;
		result->inlier_rmse = result_icp.inlier_rmse_;
		result->transformation = result_icp.transformation_;
    }

    // 返回最终4x4变换矩阵
    return result_icp.transformation_;
}

Eigen::Matrix4d RegisterMultiAreaPointCloud(
    const std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& meshAreas,
    const std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& camAreas)
{
    using namespace open3d;
    using namespace open3d::pipelines::registration;

    // 合并所有 mesh 区域点云
    auto meshAll = std::make_shared<geometry::PointCloud>();
    for (const auto& area : meshAreas) {
        if (area && !area->IsEmpty())
            *meshAll += *area;
    }

    // 合并所有 cam 区域点云
    auto camAll = std::make_shared<geometry::PointCloud>();
    for (const auto& area : camAreas) {
        if (area && !area->IsEmpty())
            *camAll += *area;
    }

    // 下采样，提升鲁棒性
    double voxel_size = 2.0; // 区域点云密度2mm，建议2.0
    auto meshAll_ds = meshAll->VoxelDownSample(voxel_size);
    auto camAll_ds = camAll->VoxelDownSample(voxel_size);

    // meshAll_ds 已有法线，camAll_ds 需估算法线（仅用于FPFH特征，不影响ICP）
    if (!meshAll_ds->HasNormals()) {
        meshAll_ds->EstimateNormals(geometry::KDTreeSearchParamHybrid(voxel_size * 3, 30));
    }
    if (!camAll_ds->HasNormals()) {
        camAll_ds->EstimateNormals(geometry::KDTreeSearchParamHybrid(voxel_size * 3, 30));
    } 

    // 计算FPFH特征
    auto mesh_fpfh = registration::ComputeFPFHFeature(
        *meshAll_ds, geometry::KDTreeSearchParamHybrid(voxel_size * 5, 100));
    auto cam_fpfh = registration::ComputeFPFHFeature(
        *camAll_ds, geometry::KDTreeSearchParamHybrid(voxel_size * 5, 100));

    // RANSAC 粗配准
    std::vector<std::reference_wrapper<const CorrespondenceChecker>> checkers;
    auto checker_edge = CorrespondenceCheckerBasedOnEdgeLength(0.9);
    auto checker_dist = CorrespondenceCheckerBasedOnDistance(voxel_size * 1.5);
    checkers.push_back(checker_edge);
    checkers.push_back(checker_dist);

    auto result_ransac = RegistrationRANSACBasedOnFeatureMatching(
        *meshAll_ds,                // 1. 源点云（已下采样）
        *camAll_ds,                 // 2. 目标点云（已下采样）
        *mesh_fpfh,                 // 3. 源点云的FPFH特征
        *cam_fpfh,                  // 4. 目标点云的FPFH特征
        true,                      // 5. mutual_filter，是否互为最近邻（True更鲁棒，减少误匹配）
        voxel_size * 1.5,           // 6. max_correspondence_distance，最大对应点距离（单位mm）
        TransformationEstimationPointToPoint(false), // 7. 变换估计方法（点到点，不考虑缩放）
        3,                          // 8. ransac_n，每次RANSAC采样的点对数（通常为3）
        checkers,                   // 9. 检查器列表（如边长约束、距离约束）
        RANSACConvergenceCriteria(150000, 0.999) // 10. 收敛准则（最大迭代次数，目标置信度）
    );

    // ICP 精配准（Point-to-Point，不依赖法线）
    auto result_icp = RegistrationICP(
        *meshAll_ds, *camAll_ds,
        voxel_size * 1.0,
        result_ransac.transformation_,
        TransformationEstimationPointToPoint()
    );

    return result_icp.transformation_;
}