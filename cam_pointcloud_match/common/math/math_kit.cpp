#include "math_kit.h"

#include <Eigen/Eigen>
#include <algorithm>
#include <functional>
#include <set>
#include <string>
#include <unordered_set>

namespace {

struct PlaneCutCandidate {
    VecInt pointIndices;                    // 候选平面拥有的原始点索引
    std::vector<Point3D> polygon;           // 该平面点集形成的凸多边形（按顺序排列）
};

struct PlaneShapeCutCandidate {
    VecInt pointIndices;                    // 候选多边形的原始点索引
    std::vector<Point3D> polygon;           // 候选多边形顶点（已排序）
};

// 将点索引排序后转成字符串键，用于候选平面去重
std::string BuildIndexKey(const VecInt& indices)
{
    std::string key;
    for (size_t i = 0; i < indices.size(); ++i) {
        if (i > 0) key += ",";
        key += std::to_string(indices[i]);
    }
    return key;
}

//将cluster中不满足近邻条件的点剔除掉，如果剔除后点数不足minVertexNum，则返回false
bool CheckEnoughNeighborsInCluster(
    VecInt& cluster,
    const std::vector<std::vector<double>>& distMatrix,
    double minSideDist,
    double maxSideDist,
    int minVertexNum)
{
    if (static_cast<int>(cluster.size()) < minVertexNum) {
        return false;
    }

	std::vector<int> invalidIndices;
    for (size_t i = 0; i < cluster.size(); ++i) {
		auto idx = cluster[i];
        int neighborCount = 0;
        for (int other : cluster) {
            if (idx == other) continue;
            double d = distMatrix[idx][other];
            if (d >= minSideDist && d <= maxSideDist) {
                ++neighborCount;
            }
        }
        if (neighborCount < minVertexNum - 1) {
            invalidIndices.push_back(int(i));
        }
    }

	// 从cluster中剔除invalidIndices对应的索引
    for (auto it = invalidIndices.rbegin(); it != invalidIndices.rend(); ++it) {
		cluster.erase(cluster.begin() + *it);
    }
    
    return cluster.size() >= static_cast<size_t>(minVertexNum);
}

// 判断候选平面和当前切割法中已有平面是否冲突
bool CandidateConflicts(
    const PlaneCutCandidate& candidate,
    const std::vector<PlaneCutCandidate>& current,
    double threshold)
{
    std::set<int> currentPointSet(candidate.pointIndices.begin(), candidate.pointIndices.end());
    for (const auto& exist : current) {
        // 同一个切割法中，一个点只能被一个平面拥有
        for (int idx : exist.pointIndices) {
            if (currentPointSet.find(idx) != currentPointSet.end()) {
                return true;
            }
        }

        // 两个平面的凸包围多边形不能处于“临近状态”
        if (PolygonToPolygonDistance(candidate.polygon, exist.polygon) < threshold) {
            return true;
        }
    }
    return false;
}

// 根据平面法向量构造二维正交基，用于把三维点投影到平面局部坐标系中
void BuildPlaneBasis(
    const Eigen::Vector3d& normal,
    Eigen::Vector3d* u,
    Eigen::Vector3d* v)
{
    Eigen::Vector3d n = normal;
    if (n.squaredNorm() <= 1e-18) {
        *u = Eigen::Vector3d::UnitX();
        *v = Eigen::Vector3d::UnitY();
        return;
    }

    n.normalize();
    Eigen::Vector3d helper = (std::abs(n.z()) < 0.9)
        ? Eigen::Vector3d::UnitZ()
        : Eigen::Vector3d::UnitX();
    *u = helper.cross(n).normalized();
    *v = n.cross(*u).normalized();
}

// 判断按顺序排列的多边形在给定平面投影下是否为凸多边形
bool IsConvexPolygonOnPlane(
    const VecInt& orderedIndices,
    const std::vector<Point3D>& projectedPoints,
    const Plane& fitPlane)
{
    if (orderedIndices.size() < 3) {
        return false;
    }

    Eigen::Vector3d u, v;
    BuildPlaneBasis(fitPlane.normal, &u, &v);

    bool hasPositive = false;
    bool hasNegative = false;
    for (size_t i = 0; i < orderedIndices.size(); ++i) {
        const Point3D& p0 = projectedPoints[orderedIndices[i]];
        const Point3D& p1 = projectedPoints[orderedIndices[(i + 1) % orderedIndices.size()]];
        const Point3D& p2 = projectedPoints[orderedIndices[(i + 2) % orderedIndices.size()]];

        Eigen::Vector2d a(p0.dot(u), p0.dot(v));
        Eigen::Vector2d b(p1.dot(u), p1.dot(v));
        Eigen::Vector2d c(p2.dot(u), p2.dot(v));

        Eigen::Vector2d ab = b - a;
        Eigen::Vector2d bc = c - b;
        double cross = ab.x() * bc.y() - ab.y() * bc.x();
        if (cross > 1e-9) hasPositive = true;
        if (cross < -1e-9) hasNegative = true;
        if (hasPositive && hasNegative) {
            return false;
        }
    }

    return true;
}

// 判断候选多边形和当前切割法中的已有多边形是否冲突
bool ShapeCandidateConflicts(
    const PlaneShapeCutCandidate& candidate,
    const std::vector<PlaneShapeCutCandidate>& current,
    double threshold)
{
    // 预先计算候选点集，使用 unordered_set 以获得 O(1) 平均查找时间
    std::unordered_set<int> pointSet(candidate.pointIndices.begin(), candidate.pointIndices.end());
    
    for (const auto& exist : current) {
        // 同一个切割法中，一个点只能属于一个多边形
        for (int idx : exist.pointIndices) {
            if (pointSet.find(idx) != pointSet.end()) {
                return true;
            }
        }

        // 两个多边形不能处于临近状态
        if (PolygonToPolygonDistance(candidate.polygon, exist.polygon) < threshold) {
            return true;
        }
    }

    return false;
}
} // namespace

// 计算点集合的 PCA
// 返回 PCAResult：质心、按升序排列的特征值和对应的特征向量（每列为一个特征向量）
PCAResult CalPCA(const std::vector<Point3D>& points)
{
    PCAResult res;
    res.eigenvalues.setZero();
    res.eigenvectors.setIdentity();
    res.centroid.setZero();

    if (points.empty()) {
        return res;
    }

    // 计算质心
    Eigen::Vector3d centroid(0.0, 0.0, 0.0);
    for (const auto& p : points) {
        centroid += p;
    }
    centroid /= static_cast<double>(points.size());
    res.centroid = centroid;

    // 计算协方差矩阵（未归一化的二阶矩）
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : points) {
        Eigen::Vector3d v = p - centroid;
        cov += v * v.transpose();
    }

    // 使用 SelfAdjointEigenSolver 求特征值/特征向量（对称矩阵）
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    if (es.info() == Eigen::Success) {
        // eigenvalues 按升序排列（从小到大），对应的特征向量在 eigenvectors 的列上
        res.eigenvalues = es.eigenvalues();
        res.eigenvectors = es.eigenvectors();
    }

    return res;
}

// 根据点集合拟合一条直线（PCA 方法）
// 质心作为线上一点，最大特征值对应的特征向量作为方向
Line FitLineFromPoints(const std::vector<Point3D>& points)
{
    Line result;
    result.point.setZero();
    result.dir.setZero();

    if (points.empty()) return result;

    PCAResult pca = CalPCA(points);
    result.point = pca.centroid;
    // 最大特征值对应最后一列（col(2)）
    result.dir = pca.eigenvectors.col(2);
    return result;
}

// 三点直接确定平面，避免在候选平面枚举时反复走 PCA。
bool BuildPlaneFromTriangle(
    const Point3D& p0,
    const Point3D& p1,
    const Point3D& p2,
    Plane* plane)
{
    if (!plane) {
        return false;
    }

    const Eigen::Vector3d v1 = p1 - p0;
    const Eigen::Vector3d v2 = p2 - p0;
    Eigen::Vector3d normal = v1.cross(v2);
    if (normal.squaredNorm() <= 1e-18) {
        plane->point = p0;
        plane->normal.setZero();
        return false;
    }

    normal.normalize();
    plane->point = p0;
    plane->normal = std::move(normal);
    return true;
}

// 使用 PCA 结果拟合平面：平面法向量为最小特征值对应的特征向量
Plane FitPlaneFromPoints(const std::vector<Point3D>& points)
{
    Plane plane;
    plane.point.setZero();
    plane.normal.setZero();

    if(points.size() < 3) {
        // 少于 3 个点无法确定平面，返回默认平面（法向量为零）
        return plane;
	}else if(points.size() == 3) {
        // 刚好 3 个点，直接用三角形方法构造平面，避免 PCA 的计算开销
        if (BuildPlaneFromTriangle(points[0], points[1], points[2], &plane)) {
            return plane;
        }
	}

    if (points.empty()) return plane;

    PCAResult pca = CalPCA(points);
    plane.point = pca.centroid;
    // 最小特征值对应列 0
    plane.normal = pca.eigenvectors.col(0);
    return plane;
}

std::vector<Point3D> ProjectPointsToPlane(
    const std::vector<Point3D>& points, 
    const Plane& plane)
{
    // 如果法向量为零，直接返回原始点（无法投影）
    Eigen::Vector3d n = plane.normal;
    double n_norm = n.norm();
    if (n_norm <= std::numeric_limits<double>::epsilon()) {
        return points;
    }

    std::vector<Point3D> projected;
    projected.reserve(points.size());
    // 归一化法向量
    n.normalize();

    Eigen::Vector3d p0 = plane.point;
    for (const auto& p : points) {
        Eigen::Vector3d v = p - p0;
        // 沿法向量的分量
        double d = v.dot(n);
        // 投影到平面上的点: p_proj = p - d * n
        Eigen::Vector3d proj = p - d * n;
        projected.push_back(proj);
    }

    return projected;
}

double LineMaxWidth(const std::vector<Point3D>& points)
{
    // 按 math_kit.h 注释的算法实现：
    // 1. 计算 PCA
    // 2. 取第二大特征值对应的特征向量 v (= eigenvectors.col(1))
    // 3. 计算每个点在 v 方向上的投影标量 s = (p - centroid) · v
    // 4. 返回 max(s) - min(s)

    if (points.empty()) return 0.0;

    PCAResult pca = CalPCA(points);

    // 第二大特征向量（Eigen 的 SelfAdjointEigenSolver 返回的特征值按升序）
    Eigen::Vector3d dir = pca.eigenvectors.col(1);

    // 检查方向有效性，避免在退化情况下归一化零向量
    const double tol = 1e-12;
    if (dir.squaredNorm() <= tol * tol) return 0.0;
    dir.normalize();

    // 计算投影标量的最小和最大值（相对于质心或不相对质心都一样，使用质心更稳定）
    Eigen::Vector3d c = pca.centroid;
    double min_s = std::numeric_limits<double>::infinity();
    double max_s = -std::numeric_limits<double>::infinity();
    for (const auto& p : points) {
        double s = (p - c).dot(dir);
        if (s < min_s) min_s = s;
        if (s > max_s) max_s = s;
    }

    double width = max_s - min_s;
    return (width > 0.0) ? width : 0.0;
}

bool IsLinkLine(const std::vector<Point3D>& points, double threshold)
{
    // 空点集视为非线性
    if (points.empty()) return false;

    // 计算线的宽度并与阈值比较
    double width = LineMaxWidth(points);
    return width < threshold;
}

CAM_API void ArrangePointCloud(
    PointCloudPtr& cloud,
    std::vector<int>* newIndexes,
    const Eigen::Vector3d& refAxis)
{
    if (!cloud) {
        return;
    }

    const size_t pointCount = cloud->points_.size();
    if (newIndexes) {
        newIndexes->assign(pointCount, -1);
    }

    if (pointCount <= 1) {
        if (newIndexes && pointCount == 1) {
            (*newIndexes)[0] = 0;
        }
        return;
    }

    // 保存原始数据，排序结束后按新顺序整体重排。
    const std::vector<Point3D> originalPoints = cloud->points_;
    const std::vector<Eigen::Vector3d> originalColors = cloud->colors_;
    const std::vector<Eigen::Vector3d> originalNormals = cloud->normals_;

    auto fillIdentityIndexes = [&]() {
        if (!newIndexes) {
            return;
        }

        for (size_t i = 0; i < pointCount; ++i) {
            (*newIndexes)[i] = static_cast<int>(i);
        }
    };

    // 1. 计算点云的拟合平面，得到平面法向量 n 和平面上的一点 C（质心）。
    Plane fitPlane = FitPlaneFromPoints(originalPoints);
    Eigen::Vector3d n = fitPlane.normal;

    // 法向量无效时不做排序，直接返回原顺序映射。
    const double tol = 1e-12;
    if (n.squaredNorm() <= tol * tol) {
        fillIdentityIndexes();
        return;
    }
    n.normalize();

    Eigen::Vector3d axis = refAxis;
    if (axis.squaredNorm() <= tol * tol) {
        fillIdentityIndexes();
        return;
    }
    axis.normalize();

    const double axisDot = n.dot(axis);
    if (std::abs(axisDot) <= tol) {
        fillIdentityIndexes();
        return;
    }

    if (axisDot < 0.0) {
        n = -n;
    }
    fitPlane.normal = n;

    // 2. 将点投影到拟合平面上，避免输入点不完全共面时影响角度排序。
    std::vector<Point3D> projectedPoints = ProjectPointsToPlane(originalPoints, fitPlane);

    // 3. 沿着平面法向量 n 看过去排序。构造平面内二维坐标基 (u, v)，满足 u × v = n。
    // 后续在这个二维平面中按角度排序。按当前“沿着 n 方向看过去”的定义，
    // 角度升序对应顺时针。
    Eigen::Vector3d helper = (std::abs(n.z()) < 0.9)
        ? Eigen::Vector3d::UnitZ()
        : Eigen::Vector3d::UnitX();
    Eigen::Vector3d u = helper.cross(n);
    if (u.squaredNorm() <= tol * tol) {
        helper = Eigen::Vector3d::UnitY();
        u = helper.cross(n);
        if (u.squaredNorm() <= tol * tol) {
            fillIdentityIndexes();
            return;
        }
    }
    u.normalize();
    Eigen::Vector3d v = n.cross(u).normalized();

    struct ArrangePointInfo {
        int oldIndex = -1;
        double angle = 0.0;
        double dist2 = 0.0;
    };

    // 5. 以拟合平面上的点 C 作为排序中心，计算每个点在二维坐标系中的角度和半径。
    Eigen::Vector2d center2d(fitPlane.point.dot(u), fitPlane.point.dot(v));
    std::vector<ArrangePointInfo> infos(pointCount);
    for (size_t i = 0; i < pointCount; ++i) {
        Eigen::Vector2d p2d(projectedPoints[i].dot(u), projectedPoints[i].dot(v));
        Eigen::Vector2d d = p2d - center2d;

        infos[i].oldIndex = static_cast<int>(i);
        infos[i].angle = std::atan2(d.y(), d.x());
        infos[i].dist2 = d.squaredNorm();
    }

    // 6. 顺时针排序。
    // 若两个点角度相同，则离中心点 C 更近的排前面；若距离也相同，则按原始索引排序。
    const double angleEps = 1e-12;
    const double distEps = 1e-12;
    std::sort(infos.begin(), infos.end(), [&](const ArrangePointInfo& a, const ArrangePointInfo& b) {
        if (std::abs(a.angle - b.angle) > angleEps) {
            return a.angle < b.angle;
        }
        if (std::abs(a.dist2 - b.dist2) > distEps) {
            return a.dist2 < b.dist2;
        }
        return a.oldIndex < b.oldIndex;
    });

    // 7. 调整起始点，使排列后的第一个顶点顺时针方向上的边是最短边。
    // 如果存在多个同样短的边，保留第一次找到的那条边作为起始边。
    size_t startPos = 0;
    double minEdgeLen2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < pointCount; ++i) {
        size_t next = (i + 1) % pointCount;
        const Point3D& p1 = projectedPoints[infos[i].oldIndex];
        const Point3D& p2 = projectedPoints[infos[next].oldIndex];
        double edgeLen2 = (p2 - p1).squaredNorm();
        if (edgeLen2 < minEdgeLen2) {
            minEdgeLen2 = edgeLen2;
            startPos = i;
        }
    }
    std::rotate(infos.begin(), infos.begin() + static_cast<std::ptrdiff_t>(startPos), infos.end());

    // 8. 按新的顺序重排点云，并输出“旧索引 -> 新索引”的映射。
    std::vector<Point3D> reorderedPoints;
    reorderedPoints.reserve(pointCount);
    for (size_t newIndex = 0; newIndex < pointCount; ++newIndex) {
        int oldIndex = infos[newIndex].oldIndex;
        reorderedPoints.push_back(originalPoints[oldIndex]);
        if (newIndexes) {
            (*newIndexes)[oldIndex] = static_cast<int>(newIndex);
        }
    }
    cloud->points_ = std::move(reorderedPoints);

    if (originalColors.size() == pointCount) {
        std::vector<Eigen::Vector3d> reorderedColors;
        reorderedColors.reserve(pointCount);
        for (const auto& info : infos) {
            reorderedColors.push_back(originalColors[info.oldIndex]);
        }
        cloud->colors_ = std::move(reorderedColors);
    }

    if (originalNormals.size() == pointCount) {
        std::vector<Eigen::Vector3d> reorderedNormals;
        reorderedNormals.reserve(pointCount);
        for (const auto& info : infos) {
            reorderedNormals.push_back(originalNormals[info.oldIndex]);
        }
        cloud->normals_ = std::move(reorderedNormals);
    }
}

CAM_API double PointToLineDistance(const Point3D& p, const Line& line)
{
    // 计算点 p 到由 line.point + t * line.dir 表示的无限直线的最短距离。
    // 如果 direction 向量退化为零，则返回点到 line.point 的欧氏距离。
    Eigen::Vector3d p0 = line.point;
    Eigen::Vector3d dir = line.dir;

    double dir_norm2 = dir.squaredNorm();
    if (dir_norm2 <= std::numeric_limits<double>::epsilon()) {
        // 退化为点
        return (p - p0).norm();
    }

    Eigen::Vector3d u = dir / std::sqrt(dir_norm2); // 归一化方向
    Eigen::Vector3d v = p - p0;
    // 垂直分量 = v - (v·u) u
    Eigen::Vector3d perp = v - u * v.dot(u);
    return perp.norm();
}

CAM_API bool PointIsOnLine(
    const Point3D& p, 
    const Line& line, 
    double threshold)
{
    // 使用 PointToLineDistance 判断点是否在直线附近
    double d = PointToLineDistance(p, line);
    return d <= threshold;
}

CAM_API double PointToLineSegmentDistance(const Point3D& p, const LineSeg3D& seg)
{
    // 计算点 p 到线段 seg 的最短距离
    Eigen::Vector3d a = seg.start;
    Eigen::Vector3d b = seg.end;
    Eigen::Vector3d ab = b - a;

    double ab2 = ab.squaredNorm();
    if (ab2 <= std::numeric_limits<double>::epsilon()) {
        // 退化为点，返回到端点的距离
        return (p - a).norm();
    }

    // 投影参数 t = (p-a)·ab / |ab|^2
    double t = (p - a).dot(ab) / ab2;
    if (t <= 0.0) {
        // 最近点为 a
        return (p - a).norm();
    }
    else if (t >= 1.0) {
        // 最近点为 b
        return (p - b).norm();
    }
    else {
        // 投影点在线段内部
        Eigen::Vector3d proj = a + ab * t;
        return (p - proj).norm();
    }
}

CAM_API bool PointIsOnLineSeg(
    const Point3D& p,
    const LineSeg3D& lineSeg,
    double threshold)
{
	auto dist = PointToLineSegmentDistance(p, lineSeg);
    return dist <= threshold;
}

CAM_API double LineToLineDistance(const Line& line1, const Line& line2)
{
    // 两条无限直线之间的最短距离
    Eigen::Vector3d p1 = line1.point;
    Eigen::Vector3d u = line1.dir;
    Eigen::Vector3d p2 = line2.point;
    Eigen::Vector3d v = line2.dir;

    const double eps = 1e-12;
    double u2 = u.squaredNorm();
    double v2 = v.squaredNorm();

    // 处理退化情况：方向向量为零的直线视为点
    if (u2 <= eps && v2 <= eps) {
        return (p1 - p2).norm();
    }
    if (u2 <= eps) {
        return PointToLineDistance(p1, line2);
    }
    if (v2 <= eps) {
        return PointToLineDistance(p2, line1);
    }

    // 计算两方向向量的叉乘，用于判断平行与计算距离
    Eigen::Vector3d cross = u.cross(v);
    double denom = cross.norm();
    Eigen::Vector3d w0 = p2 - p1;

    if (denom <= eps) {
        // 平行或共线：距离为 w0 在 u 正交方向上的分量长度
        Eigen::Vector3d u_hat = u / std::sqrt(u2);
        Eigen::Vector3d perp = w0 - u_hat * w0.dot(u_hat);
        return perp.norm();
    }

    // 非平行：距离等于绝对值 |(p2-p1)·(u×v)| / |u×v|
    double distance = std::abs(w0.dot(cross)) / denom;
    return distance;
}   



CAM_API bool LinesIsIntersected(
    const Line& line1, 
    const Line& line2, 
    double threshold)
{
	auto dist = LineToLineDistance(line1, line2);
    return dist <= threshold;
}

CAM_API double LineSegToLineSegDistance(const LineSeg3D& seg1, const LineSeg3D& seg2)
{
    // 计算两条线段 seg1(seg1.start->seg1.end) 与 seg2(seg2.start->seg2.end) 的最小距离
    // 采用标准参数化求解并对参数进行截断到区间 [0,1] 的方法。
    // p, q 为两段起点，d1,d2 为方向向量，r = p - q
    Eigen::Vector3d p = seg1.start;
    Eigen::Vector3d q = seg2.start;
    Eigen::Vector3d d1 = seg1.end - seg1.start; // 段1方向
    Eigen::Vector3d d2 = seg2.end - seg2.start; // 段2方向
    Eigen::Vector3d r = p - q;

    double a = d1.dot(d1); // 段1长度的平方
    double e = d2.dot(d2); // 段2长度的平方
    double f = d2.dot(r);

    const double EPS = 1e-12;

    // 若两段都退化为点，直接返回两点距离
    if (a <= EPS && e <= EPS) {
        return (p - q).norm();
    }

    // 若段1退化为点，返回点到段2的距离
    if (a <= EPS) {
        return PointToLineSegmentDistance(p, LineSeg3D{seg2.start, seg2.end});
    }

    // 若段2退化为点，返回点到段1的距离
    if (e <= EPS) {
        return PointToLineSegmentDistance(q, LineSeg3D{seg1.start, seg1.end});
    }

    // 常用系数
    double b = d1.dot(d2);
    double c = d1.dot(r);
    double denom = a * e - b * b; // 判别式，若为0则平行

    double s = 0.0; // 段1的参数
    double t = 0.0; // 段2的参数

    if (denom != 0.0) {
        // 非平行情况，计算未截断的 s 和 t
        double sN = (b * f - c * e);
        double tN = (a * f - b * c);
        double sD = denom;
        double tD = denom;

        // 对 s 进行初步截断判断
        if (sN < 0.0) {
            s = 0.0;
            t = f / e; // 稍后会对 t 截断到 [0,1]
        } else if (sN > sD) {
            s = 1.0;
            t = (f + b) / e;
        } else {
            s = sN / sD;
            t = tN / tD;
        }
    } else {
        // 平行情况，令 s = 0，计算 t
        s = 0.0;
        t = f / e;
    }

    // 将 t 截断到 [0,1]，并在截断后重新计算 s
    if (t < 0.0) {
        t = 0.0;
        double sTmp = -c / a;
        if (sTmp < 0.0) s = 0.0;
        else if (sTmp > 1.0) s = 1.0;
        else s = sTmp;
    } else if (t > 1.0) {
        t = 1.0;
        double sTmp = (b - c) / a;
        if (sTmp < 0.0) s = 0.0;
        else if (sTmp > 1.0) s = 1.0;
        else s = sTmp;
    }

    // 确保 s 在 [0,1] 内
    if (s < 0.0) s = 0.0;
    else if (s > 1.0) s = 1.0;

    // 根据参数计算两段上的最近点并返回距离
    Eigen::Vector3d c1 = p + d1 * s;
    Eigen::Vector3d c2 = q + d2 * t;
    return (c1 - c2).norm();
}

CAM_API bool LineSegsIsIntersected(
    const LineSeg3D& lineSeg1,
    const LineSeg3D& lineSeg2,
    double threshold)
{
	auto dist = LineSegToLineSegDistance(lineSeg1, lineSeg2);
	return dist <= threshold;
}

CAM_API double PointToPlaneDistance(
    const Point3D& p,
    const Plane& plane)
{
    // 计算点 p 到平面 plane 的距离
    // plane 给出一个位于平面上的点 plane.point 和法向量 plane.normal
    Eigen::Vector3d n = plane.normal;

    // 如果法向量退化为零，则退化为点到点的距离（无法定义平面）
    const double eps = 1e-12;
    if (n.squaredNorm() <= eps * eps) {
        return (p - plane.point).norm();
    }

    // 归一化法向量，计算投影距离：|(p - p0) · n_hat|
    n.normalize();
    double dist = std::abs((p - plane.point).dot(n));
    return dist;
}

CAM_API bool PointIsOnPlane(
    const Point3D& p, 
    const Plane& plane, 
    double threshold)
{
	auto dist = PointToPlaneDistance(p, plane);
	return dist <= threshold;
}

CAM_API double PolygonToPolygonDistance(
    const std::vector<Point3D>& poly1,
    const std::vector<Point3D>& poly2)
{
    // 输入点数不足以形成多边形时，退化处理为点集间最小距离。
    if (poly1.empty() || poly2.empty()) {
        return 0.0;
    }

    const double eps = 1e-9;

    // 构造与法向量正交的二维坐标基，用于角度排序和点在多边形内测试。
    auto BuildPlaneBasis = [](const Eigen::Vector3d& normal, Eigen::Vector3d* u, Eigen::Vector3d* v) {
        Eigen::Vector3d n = normal;
        if (n.squaredNorm() <= 1e-18) {
            *u = Eigen::Vector3d::UnitX();
            *v = Eigen::Vector3d::UnitY();
            return;
        }
        n.normalize();
        Eigen::Vector3d helper = (std::abs(n.z()) < 0.9)
            ? Eigen::Vector3d::UnitZ()
            : Eigen::Vector3d::UnitX();
        *u = helper.cross(n).normalized();
        *v = n.cross(*u).normalized();
    };

    // 将一个凸多边形顶点按绕质心的角度排序，便于后续边遍历和点在多边形内判断。
    auto ArrangePolygon = [&](const std::vector<Point3D>& points, const Plane& plane) {
        struct Node {
            int index = -1;
            double angle = 0.0;
            double dist2 = 0.0;
        };

        Eigen::Vector3d u, v;
        BuildPlaneBasis(plane.normal, &u, &v);

        Eigen::Vector2d center(plane.point.dot(u), plane.point.dot(v));
        std::vector<Node> nodes(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            Eigen::Vector2d p2(points[i].dot(u), points[i].dot(v));
            Eigen::Vector2d d = p2 - center;
            nodes[i].index = static_cast<int>(i);
            nodes[i].angle = std::atan2(d.y(), d.x());
            nodes[i].dist2 = d.squaredNorm();
        }

        std::sort(nodes.begin(), nodes.end(), [&](const Node& a, const Node& b) {
            if (std::abs(a.angle - b.angle) > 1e-12) {
                return a.angle > b.angle;
            }
            if (std::abs(a.dist2 - b.dist2) > 1e-12) {
                return a.dist2 < b.dist2;
            }
            return a.index < b.index;
        });

        std::vector<Point3D> arranged;
        arranged.reserve(points.size());
        for (const auto& node : nodes) {
            arranged.push_back(points[node.index]);
        }
        return arranged;
    };

    // 判断点 p 在 plane 上的投影是否落在凸多边形 polygon 内部或边界上。
    auto IsPointInConvexPolygon = [&](const Point3D& p, const std::vector<Point3D>& polygon, const Plane& plane) {
        if (polygon.size() < 3) {
            return false;
        }

        Eigen::Vector3d u, v;
        BuildPlaneBasis(plane.normal, &u, &v);

        Eigen::Vector2d p2(p.dot(u), p.dot(v));
        bool hasPositive = false;
        bool hasNegative = false;

        for (size_t i = 0; i < polygon.size(); i++) {
            const Point3D& a3 = polygon[i];
            const Point3D& b3 = polygon[(i + 1) % polygon.size()];
            Eigen::Vector2d a(a3.dot(u), a3.dot(v));
            Eigen::Vector2d b(b3.dot(u), b3.dot(v));
            Eigen::Vector2d ab = b - a;
            Eigen::Vector2d ap = p2 - a;

            double cross = ab.x() * ap.y() - ab.y() * ap.x();
            if (cross > eps) hasPositive = true;
            if (cross < -eps) hasNegative = true;
            if (hasPositive && hasNegative) {
                return false;
            }
        }
        return true;
    };

    // 计算点到凸多边形区域的距离：
    // 若点投影落在多边形区域内，则距离为点到平面的距离；
    // 否则距离为点到各边线段距离的最小值。
    auto PointToPolygonDistanceImpl = [&](const Point3D& p, const std::vector<Point3D>& polygon, const Plane& plane) {
        if (polygon.empty()) {
            return 0.0;
        }

        Eigen::Vector3d n = plane.normal;
        if (n.squaredNorm() <= 1e-18) {
            double minDist = std::numeric_limits<double>::infinity();
            for (const auto& pt : polygon) {
                minDist = std::min(minDist, (p - pt).norm());
            }
            return minDist;
        }

        n.normalize();
        Point3D proj = p - ((p - plane.point).dot(n)) * n;
        if (IsPointInConvexPolygon(proj, polygon, plane)) {
            return PointToPlaneDistance(p, plane);
        }

        double minDist = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < polygon.size(); ++i) {
            LineSeg3D seg{ polygon[i], polygon[(i + 1) % polygon.size()] };
            minDist = std::min(minDist, PointToLineSegmentDistance(p, seg));
        }
        return minDist;
    };

    // 判断一条线段是否与凸多边形区域相交。
    // 若线段穿过对方平面且交点落在多边形内部，则认为相交。
    auto SegmentIntersectsPolygon = [&](const LineSeg3D& seg, const std::vector<Point3D>& polygon, const Plane& plane) {
        Eigen::Vector3d n = plane.normal;
        if (n.squaredNorm() <= 1e-18 || polygon.size() < 3) {
            return false;
        }
        n.normalize();

        double d0 = (seg.start - plane.point).dot(n);
        double d1 = (seg.end - plane.point).dot(n);

        // 两端都在平面同侧且不接近平面，不可能与平面内区域相交。
        if ((d0 > eps && d1 > eps) || (d0 < -eps && d1 < -eps)) {
            return false;
        }

        // 整条线段近似落在平面上时，检查端点是否在多边形内，
        // 或者与多边形边的最小距离是否为 0。
        if (std::abs(d0) <= eps && std::abs(d1) <= eps) {
            if (IsPointInConvexPolygon(seg.start, polygon, plane) ||
                IsPointInConvexPolygon(seg.end, polygon, plane)) {
                return true;
            }
            for (size_t i = 0; i < polygon.size(); ++i) {
                LineSeg3D edge{ polygon[i], polygon[(i + 1) % polygon.size()] };
                if (LineSegToLineSegDistance(seg, edge) <= eps) {
                    return true;
                }
            }
            return false;
        }

        // 计算线段与平面的交点参数 t。
        double denom = d0 - d1;
        if (std::abs(denom) <= eps) {
            return false;
        }
        double t = d0 / (d0 - d1);
        if (t < -eps || t > 1.0 + eps) {
            return false;
        }

        Point3D intersectPoint = seg.start + t * (seg.end - seg.start);
        return IsPointInConvexPolygon(intersectPoint, polygon, plane);
    };

    // 1. 分别拟合两个多边形的平面。
    Plane fitPlane1 = FitPlaneFromPoints(poly1);
    Plane fitPlane2 = FitPlaneFromPoints(poly2);

    // 2. 将顶点分别投影到各自的拟合平面上。
    std::vector<Point3D> projPoly1 = ProjectPointsToPlane(poly1, fitPlane1);
    std::vector<Point3D> projPoly2 = ProjectPointsToPlane(poly2, fitPlane2);

    // 3. 对投影后的凸多边形顶点做环绕排序，保证边的连接顺序正确。
    projPoly1 = ArrangePolygon(projPoly1, fitPlane1);
    projPoly2 = ArrangePolygon(projPoly2, fitPlane2);

    if (projPoly1.size() < 2 || projPoly2.size() < 2) {
        double minDist = std::numeric_limits<double>::infinity();
        for (const auto& p1 : projPoly1) {
            for (const auto& p2 : projPoly2) {
                minDist = std::min(minDist, (p1 - p2).norm());
            }
        }
        return std::isfinite(minDist) ? minDist : 0.0;
    }

    // 4. 若一条多边形的边穿过另一条多边形区域，则两区域有交，距离为 0。
    for (size_t i = 0; i < projPoly1.size(); ++i) {
        LineSeg3D seg{ projPoly1[i], projPoly1[(i + 1) % projPoly1.size()] };
        if (SegmentIntersectsPolygon(seg, projPoly2, fitPlane2)) {
            return 0.0;
        }
    }
    for (size_t i = 0; i < projPoly2.size(); ++i) {
        LineSeg3D seg{ projPoly2[i], projPoly2[(i + 1) % projPoly2.size()] };
        if (SegmentIntersectsPolygon(seg, projPoly1, fitPlane1)) {
            return 0.0;
        }
    }

    // 5. 若一个多边形顶点落在另一个多边形区域内，则两区域重叠，距离为 0。
    for (const auto& p : projPoly1) {
        if (PointToPolygonDistanceImpl(p, projPoly2, fitPlane2) <= eps) {
            return 0.0;
        }
    }
    for (const auto& p : projPoly2) {
        if (PointToPolygonDistanceImpl(p, projPoly1, fitPlane1) <= eps) {
            return 0.0;
        }
    }

    // 6. 分离情况下，最小距离只可能出现在：
    //    - poly1 顶点到 poly2 区域
    //    - poly2 顶点到 poly1 区域
    //    - 两条边之间
    double minDist = std::numeric_limits<double>::infinity();

    for (const auto& p : projPoly1) {
        minDist = std::min(minDist, PointToPolygonDistanceImpl(p, projPoly2, fitPlane2));
    }
    for (const auto& p : projPoly2) {
        minDist = std::min(minDist, PointToPolygonDistanceImpl(p, projPoly1, fitPlane1));
    }

    for (size_t i = 0; i < projPoly1.size(); ++i) {
        LineSeg3D seg1{ projPoly1[i], projPoly1[(i + 1) % projPoly1.size()] };
        for (size_t j = 0; j < projPoly2.size(); ++j) {
            LineSeg3D seg2{ projPoly2[j], projPoly2[(j + 1) % projPoly2.size()] };
            minDist = std::min(minDist, LineSegToLineSegDistance(seg1, seg2));
        }
    }

    return std::isfinite(minDist) ? minDist : 0.0;
}

CAM_API bool PolygonsIsIntersected(
    const std::vector<Point3D>& poly1, 
    const std::vector<Point3D>& poly2, 
    double threshold)
{
	auto dist = PolygonToPolygonDistance(poly1, poly2);
	return dist <= threshold;
}

CAM_API double PointToPolygonDistance(
    const Point3D& p,
    const std::vector<Point3D>& poly)
{
    // 空多边形时，约定距离为 0
    if (poly.empty()) {
        return 0.0;
    }

    // 只有一个点时，退化为点到点距离
    if (poly.size() == 1) {
        return (p - poly.front()).norm();
    }

    const double eps = 1e-9;

    // 1. 计算多边形的拟合平面
    Plane fitPlane = FitPlaneFromPoints(poly);

    // 2. 将多边形顶点投影到拟合平面上
    std::vector<Point3D> projPoly = ProjectPointsToPlane(poly, fitPlane);

    // 若退化成线段，则返回点到各线段的最小距离
    if (projPoly.size() == 2) {
        return PointToLineSegmentDistance(p, LineSeg3D{ projPoly[0], projPoly[1] });
    }

    // 3. 构造拟合平面的二维坐标基，用于顶点排序和点在凸多边形内判断
    Eigen::Vector3d n = fitPlane.normal;
    if (n.squaredNorm() <= 1e-18) {
        // 平面法向量退化时，退化为点到点集的最小距离
        double minDist = std::numeric_limits<double>::infinity();
        for (const auto& pt : projPoly) {
            minDist = std::min(minDist, (p - pt).norm());
        }
        return std::isfinite(minDist) ? minDist : 0.0;
    }
    n.normalize();

    Eigen::Vector3d helper = (std::abs(n.z()) < 0.9)
        ? Eigen::Vector3d::UnitZ()
        : Eigen::Vector3d::UnitX();
    Eigen::Vector3d u = helper.cross(n).normalized();
    Eigen::Vector3d v = n.cross(u).normalized();

    // 4. 按绕质心的角度对投影顶点排序，保证边的连接顺序正确
    struct PolygonNode {
        int index = -1;
        double angle = 0.0;
        double dist2 = 0.0;
    };

    Eigen::Vector2d center2d(fitPlane.point.dot(u), fitPlane.point.dot(v));
    std::vector<PolygonNode> nodes(projPoly.size());
    for (size_t i = 0; i < projPoly.size(); ++i) {
        Eigen::Vector2d p2(projPoly[i].dot(u), projPoly[i].dot(v));
        Eigen::Vector2d d = p2 - center2d;
        nodes[i].index = static_cast<int>(i);
        nodes[i].angle = std::atan2(d.y(), d.x());
        nodes[i].dist2 = d.squaredNorm();
    }

    std::sort(nodes.begin(), nodes.end(), [&](const PolygonNode& a, const PolygonNode& b) {
        if (std::abs(a.angle - b.angle) > 1e-12) {
            return a.angle > b.angle;
        }
        if (std::abs(a.dist2 - b.dist2) > 1e-12) {
            return a.dist2 < b.dist2;
        }
        return a.index < b.index;
    });

    std::vector<Point3D> orderedPoly;
    orderedPoly.reserve(projPoly.size());
    for (const auto& node : nodes) {
        orderedPoly.push_back(projPoly[node.index]);
    }

    // 5. 将点 p 投影到拟合平面上，如果投影点落在凸多边形区域内，则距离就是点到平面的距离
    Point3D projPoint = p - ((p - fitPlane.point).dot(n)) * n;
    Eigen::Vector2d projPoint2d(projPoint.dot(u), projPoint.dot(v));

    bool hasPositive = false;
    bool hasNegative = false;
    for (size_t i = 0; i < orderedPoly.size(); ++i) {
        Eigen::Vector2d a(orderedPoly[i].dot(u), orderedPoly[i].dot(v));
        Eigen::Vector2d b(orderedPoly[(i + 1) % orderedPoly.size()].dot(u), orderedPoly[(i + 1) % orderedPoly.size()].dot(v));
        Eigen::Vector2d ab = b - a;
        Eigen::Vector2d ap = projPoint2d - a;

        double cross = ab.x() * ap.y() - ab.y() * ap.x();
        if (cross > eps) hasPositive = true;
        if (cross < -eps) hasNegative = true;
        if (hasPositive && hasNegative) {
            break;
        }
    }

    if (!(hasPositive && hasNegative)) {
        return PointToPlaneDistance(p, fitPlane);
    }

    // 6. 若投影点不在多边形区域内，则最小距离为点到各边线段距离的最小值
    double minDist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < orderedPoly.size(); ++i) {
        LineSeg3D seg{ orderedPoly[i], orderedPoly[(i + 1) % orderedPoly.size()] };
        minDist = std::min(minDist, PointToLineSegmentDistance(p, seg));
    }

    return std::isfinite(minDist) ? minDist : 0.0;
}

CAM_API bool PointIsOnPolygon(
    const Point3D& p, 
    const std::vector<Point3D>& poly, 
    double threshold){
	auto dist = PointToPolygonDistance(p, poly);
    return dist <= threshold;
}

CAM_API void DeleteOuterPoints(PointCloudPtr pointCloud, double maxSideDist)
{
    // 收集有效点及其原始索引
    std::vector<Point3D> validPoints;
    std::vector<int> validIndices;
    validPoints.reserve(pointCloud->points_.size());
    validIndices.reserve(pointCloud->points_.size());
    for (size_t i = 0; i < pointCloud->points_.size(); ++i) {
        const auto& p = pointCloud->points_[i];
        if (IsValidPoint3D(p)) {
            validPoints.emplace_back(p);
            validIndices.emplace_back(static_cast<int>(i));
        }
    }

    if (validPoints.empty()) {
        // 没有有效点，清空所有数组并返回
        pointCloud->points_.clear();
        pointCloud->colors_.clear();
        pointCloud->normals_.clear();
        return;
    }

    const size_t n = validPoints.size();
    std::vector<double> nearestDist(n, std::numeric_limits<double>::infinity());
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            const double d = (validPoints[i] - validPoints[j]).norm();
            if (d < nearestDist[i])
                nearestDist[i] = d;
            if (d < nearestDist[j])
                nearestDist[j] = d;
        }
    }

    // 构建保留点的掩码（保留那些 nearestDist <= maxSideDist）
    std::vector<bool> keepMask(n, true);
    for (size_t i = 0; i < n; ++i) {
        if (nearestDist[i] > maxSideDist) {
            keepMask[i] = false;
        }
    }

    // 若全部保留或全部删除，则直接做相应处理
    bool anyKept = false;
    for (bool k : keepMask) { if (k) { anyKept = true; break; } }
    if (!anyKept) {
        pointCloud->points_.clear();
        pointCloud->colors_.clear();
        pointCloud->normals_.clear();
        return;
    }

    // 重建点、颜色、法线数组，保持顺序为原始点云的相对顺序
    std::vector<Point3D> newPoints;
    std::vector<RGB> newColors;
    std::vector<Point3D> newNormals;
    newPoints.reserve(n);
    const bool hasColors = pointCloud->HasColors();
    const bool hasNormals = pointCloud->HasNormals();
    if (hasColors) newColors.reserve(n);
    if (hasNormals) newNormals.reserve(n);

    for (size_t vi = 0; vi < n; ++vi) {
        if (!keepMask[vi]) continue;
        const int origIdx = validIndices[vi];
        newPoints.emplace_back(pointCloud->points_[origIdx]);
        if (hasColors && origIdx < static_cast<int>(pointCloud->colors_.size())) {
            newColors.emplace_back(pointCloud->colors_[origIdx]);
        }
        if (hasNormals && origIdx < static_cast<int>(pointCloud->normals_.size())) {
            newNormals.emplace_back(pointCloud->normals_[origIdx]);
        }
    }

    pointCloud->points_.swap(newPoints);
    if (hasColors) pointCloud->colors_.swap(newColors);
    else pointCloud->colors_.clear();
    if (hasNormals) pointCloud->normals_.swap(newNormals);
    else pointCloud->normals_.clear();
}

CAM_API PLANE_CUT_VEC PlaneCut(
    const PointCloudPtr& pointCloud,
    int minVertexNum,
    int maxPlaneNum,
    double minSideDist,
    double maxSideDist,
    double threshold,
    std::vector<int>* unclusteredIndices)
{
    PLANE_CUT_VEC result;

    if (unclusteredIndices) {
        unclusteredIndices->clear();
    }

    if (!pointCloud || minVertexNum < 3 || maxPlaneNum <= 0) {
        return result;
    }

    const auto& points = pointCloud->points_;
    const int n = static_cast<int>(points.size());
    if (n < minVertexNum) {
        if (unclusteredIndices) {
            for (int i = 0; i < n; ++i) {
                unclusteredIndices->push_back(i);
            }
        }
        return result;
    }

    // 预计算点对距离矩阵，后续用于近邻约束判断。
    std::vector<std::vector<double>> distMatrix(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double d = (points[i] - points[j]).norm();
            distMatrix[i][j] = d;
            distMatrix[j][i] = d;
        }
    }

    // 枚举三点生成候选平面。
    std::vector<PlaneCutCandidate> candidates;
    std::set<std::string> candidateKeys;
    std::set<int> clusterablePointSet;

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            auto d = (points[i] - points[j]).squaredNorm();
            if (d < minSideDist * minSideDist || d > maxSideDist * maxSideDist) 
                continue;
            for (int k = j + 1; k < n; ++k) {
				auto d1 = (points[i] - points[k]).squaredNorm();
				auto d2 = (points[j] - points[k]).squaredNorm();
                if((d1 < minSideDist * minSideDist || d1 > maxSideDist * maxSideDist) && 
                    (d2 < minSideDist * minSideDist || d2 > maxSideDist * maxSideDist))
					continue;

                std::vector<Point3D> seedPoints{ points[i], points[j], points[k] };
                Plane plane = FitPlaneFromPoints(seedPoints);

                // 三点近共线时无法稳定定义平面，跳过。
                if (plane.normal.squaredNorm() <= 1e-18) {
                    continue;
                }

                // 收集所有落在该平面附近的点。
                VecInt cluster;
                for (int idx = 0; idx < n; ++idx) {
                    if (PointToPlaneDistance(points[idx], plane) <= threshold) {
                        cluster.push_back(idx);
                    }
                }

                if (!CheckEnoughNeighborsInCluster(cluster, distMatrix, minSideDist, maxSideDist, minVertexNum)) {
                    continue;
                }

                std::sort(cluster.begin(), cluster.end());
                std::string key = BuildIndexKey(cluster);
                if (!candidateKeys.insert(key).second) {
                    continue;
                }

                std::vector<Point3D> polygon;
                polygon.reserve(cluster.size());
                for (int idx : cluster) {
                    polygon.push_back(points[idx]);
                    clusterablePointSet.insert(idx);
                }

                // 复用现有排序函数，使多边形顶点顺序稳定。
                PointCloudPtr polyCloud = std::make_shared<PointCloud>();
                polyCloud->points_ = polygon;
                ArrangePointCloud(polyCloud);

                PlaneCutCandidate candidate;
                candidate.pointIndices = cluster;
                candidate.polygon = polyCloud->points_;
                candidates.push_back(std::move(candidate));
            }
        }
    }

    // 输出无法参与任何候选平面切割的点。
    if (unclusteredIndices) {
        for (int idx = 0; idx < n; ++idx) {
            if (clusterablePointSet.find(idx) == clusterablePointSet.end()) {
                unclusteredIndices->push_back(idx);
            }
        }
    }

    if (candidates.empty()) {
        return result;
    }

    // 优先尝试点数更多的候选平面，便于更快得到较完整的切割法。
    std::sort(candidates.begin(), candidates.end(), [](const PlaneCutCandidate& a, const PlaneCutCandidate& b) {
        if (a.pointIndices.size() != b.pointIndices.size()) {
            return a.pointIndices.size() > b.pointIndices.size();
        }
        return a.pointIndices < b.pointIndices;
    });

    std::set<std::string> solutionKeys;
    std::vector<PlaneCutCandidate> current;
    std::vector<int> selectedCandidateIndices;

    // 使用显式状态的迭代回溯生成所有合法切割法，避免递归。
    auto appendCurrentSolution = [&]() {
        if (current.empty()) {
            return;
        }

        // 将当前切割法转成结果类型，并做一次去重。
        std::vector<VecInt> oneCut;
        std::vector<std::string> keys;
        for (const auto& plane : current) {
            oneCut.push_back(plane.pointIndices);
            keys.push_back(BuildIndexKey(plane.pointIndices));
        }
        std::sort(keys.begin(), keys.end());

        std::string solutionKey;
        for (size_t i = 0; i < keys.size(); i++) {
            if (i > 0) solutionKey += "|";
            solutionKey += keys[i];
        }
        if (solutionKeys.insert(solutionKey).second) {
            result.push_back(std::move(oneCut));
        }
    };

    int nextIndex = 0;
    while (true) {
        bool advanced = false;

        if (static_cast<int>(current.size()) < maxPlaneNum) {
            for (int i = nextIndex; i < static_cast<int>(candidates.size()); ++i) {
                const auto& candidate = candidates[static_cast<size_t>(i)];
                if (CandidateConflicts(candidate, current, threshold)) {
                    continue;
                }

                current.push_back(candidate);
                selectedCandidateIndices.push_back(i);
                appendCurrentSolution();

                nextIndex = i + 1;
                advanced = true;
                break;
            }
        }

        if (advanced) {
            continue;
        }

        if (selectedCandidateIndices.empty()) {
            break;
        }

        nextIndex = selectedCandidateIndices.back() + 1;
        selectedCandidateIndices.pop_back();
        current.pop_back();
    }

    return result;
}

CAM_API PLANE_SHAPE_CUT_VEC PlaneShapeCut(
    const PointCloudPtr& pointCloud,
    int minVertexNum,
    int maxVertexNum,
    double minSideDist,
    double maxSideDist,
    double threshold)
{
    PLANE_SHAPE_CUT_VEC result;

    if (!pointCloud || minVertexNum < 3 || maxVertexNum < minVertexNum) {
        return result;
    }

    const auto& points = pointCloud->points_;
    const int pointCount = static_cast<int>(points.size());
    if (pointCount < minVertexNum) {
        return result;
    }

    // 1. 拟合整个点云的平面，并把所有点投影到该平面上。
    // 后续凸性判断基于这个统一的投影平面进行。
    Plane fitPlane = FitPlaneFromPoints(points);
    std::vector<Point3D> projectedPoints = ProjectPointsToPlane(points, fitPlane);

    // 2. 预计算点对距离矩阵，便于快速检查多边形边长是否满足约束。
    std::vector<std::vector<double>> distMatrix(pointCount, std::vector<double>(pointCount, 0.0));
    for (int i = 0; i < pointCount; ++i) {
        for (int j = i + 1; j < pointCount; ++j) {
            double d = (points[i] - points[j]).norm();
            distMatrix[i][j] = d;
            distMatrix[j][i] = d;
        }
    }

    // 3. 枚举所有候选多边形。
    // 每个候选多边形都要：
    // - 顶点数在 [minVertexNum, maxVertexNum] 范围内
    // - 顶点顺序通过 ArrangePointCloud 排序
    // - 投影到 fitPlane 上后是凸多边形
    // - 每条边的长度都满足 [minSideDist, maxSideDist]
    std::vector<PlaneShapeCutCandidate> candidates;
    std::set<std::string> candidateKeys;

    // 使用迭代方式枚举所有给定大小的点组合，替代递归组合枚举。
    auto appendCandidateFromIndices = [&](const VecInt& selectedIndices) {
        // 使用 ArrangePointCloud 的逻辑对候选点集进行排序。
        PointCloudPtr subCloud = std::make_shared<PointCloud>();
        subCloud->points_.reserve(selectedIndices.size());
        for (int idx : selectedIndices) {
            subCloud->points_.push_back(points[idx]);
        }

        std::vector<int> newIndexes;
        ArrangePointCloud(subCloud, &newIndexes);

        // 将"局部旧索引 -> 局部新索引"的映射还原成"全局有序索引"。
        VecInt orderedGlobalIndices(selectedIndices.size(), -1);
        for (size_t localOldIndex = 0; localOldIndex < selectedIndices.size(); ++localOldIndex) {
            int localNewIndex = newIndexes[localOldIndex];
            orderedGlobalIndices[static_cast<size_t>(localNewIndex)] = selectedIndices[localOldIndex];
        }

        // 要求在整体拟合平面上的投影是凸多边形。
        if (!IsConvexPolygonOnPlane(orderedGlobalIndices, projectedPoints, fitPlane)) {
            return;
        }

        // 检查边长是否满足约束。
        for (size_t i = 0; i < orderedGlobalIndices.size(); ++i) {
            int a = orderedGlobalIndices[i];
            int b = orderedGlobalIndices[(i + 1) % orderedGlobalIndices.size()];
            double side = distMatrix[a][b];
            if (side < minSideDist || side > maxSideDist) {
                return;
            }
        }

        // 用排序后的索引集合去重，避免重复候选。
        VecInt sortedKeyIndices = orderedGlobalIndices;
        std::sort(sortedKeyIndices.begin(), sortedKeyIndices.end());
        std::string key = BuildIndexKey(sortedKeyIndices);
        if (!candidateKeys.insert(key).second) {
            return;
        }

        PlaneShapeCutCandidate candidate;
        candidate.pointIndices = std::move(orderedGlobalIndices);
        candidate.polygon = subCloud->points_;
        candidates.push_back(std::move(candidate));
    };

    for (int polygonSize = minVertexNum; polygonSize <= std::min(maxVertexNum, pointCount); ++polygonSize) {
        VecInt selectedIndices(static_cast<size_t>(polygonSize));
        for (int i = 0; i < polygonSize; ++i) {
            selectedIndices[static_cast<size_t>(i)] = i;
        }

        while (true) {
            appendCandidateFromIndices(selectedIndices);

            int pos = polygonSize - 1;
            while (pos >= 0 && selectedIndices[static_cast<size_t>(pos)] == pointCount - polygonSize + pos) {
                --pos;
            }
            if (pos < 0) {
                break;
            }

            ++selectedIndices[static_cast<size_t>(pos)];
            for (int i = pos + 1; i < polygonSize; ++i) {
                selectedIndices[static_cast<size_t>(i)] = selectedIndices[static_cast<size_t>(i - 1)] + 1;
            }
        }
    }

    if (candidates.empty()) {
        return result;
    }

    // 4. 优先尝试顶点数更多的多边形，尽量先构造更完整的切割法。
    std::sort(candidates.begin(), candidates.end(), [](const PlaneShapeCutCandidate& a, const PlaneShapeCutCandidate& b) {
        if (a.pointIndices.size() != b.pointIndices.size()) {
            return a.pointIndices.size() > b.pointIndices.size();
        }
        return a.pointIndices < b.pointIndices;
    });

    // 5. 使用显式栈/指针的迭代回溯方式生成所有合法切割法，替代递归 DFS。
    std::vector<PlaneShapeCutCandidate> currentShapes;
    std::vector<int> selectedCandidateIndices;
    std::set<std::string> solutionKeys;
    
    // 使用点使用标记位图来快速检查点冲突，避免重复构建 set
    std::vector<bool> pointUsed(pointCount, false);

    auto appendCurrentSolution = [&]() {
        if (currentShapes.empty()) {
            return;
        }

        std::vector<VecInt> oneCut;
        std::vector<std::string> keys;
        for (const auto& shape : currentShapes) {
            oneCut.push_back(shape.pointIndices);

            VecInt sortedIndices = shape.pointIndices;
            std::sort(sortedIndices.begin(), sortedIndices.end());
            keys.push_back(BuildIndexKey(sortedIndices));
        }

        std::sort(keys.begin(), keys.end());
        std::string solutionKey;
        for (size_t i = 0; i < keys.size(); ++i) {
            if (i > 0) solutionKey += "|";
            solutionKey += keys[i];
        }

        if (solutionKeys.insert(solutionKey).second) {
            result.push_back(std::move(oneCut));
        }
    };

    int nextIndex = 0;
    while (true) {
        bool advanced = false;
        for (int i = nextIndex; i < static_cast<int>(candidates.size()); ++i) {
            const auto& candidate = candidates[static_cast<size_t>(i)];
            
            // 快速点冲突检查：使用位图直接检查
            bool hasPointConflict = false;
            for (int idx : candidate.pointIndices) {
                if (pointUsed[idx]) {
                    hasPointConflict = true;
                    break;
                }
            }
            
            // 只有在没有点冲突时才进行更昂贵的多边形距离检查
            if (hasPointConflict) {
                continue;
            }
            
            // 检查多边形距离冲突
            bool hasDistanceConflict = false;
            for (const auto& exist : currentShapes) {
                if (PolygonToPolygonDistance(candidate.polygon, exist.polygon) < threshold) {
                    hasDistanceConflict = true;
                    break;
                }
            }
            
            if (hasDistanceConflict) {
                continue;
            }

            // 标记使用的点
            for (int idx : candidate.pointIndices) {
                pointUsed[idx] = true;
            }
            
            currentShapes.push_back(candidate);
            selectedCandidateIndices.push_back(i);
            appendCurrentSolution();

            nextIndex = i + 1;
            advanced = true;
            break;
        }

        if (advanced) {
            continue;
        }

        if (selectedCandidateIndices.empty()) {
            break;
        }

        nextIndex = selectedCandidateIndices.back() + 1;
        selectedCandidateIndices.pop_back();
        
        // 回溯时清除点使用标记
        const auto& lastShape = currentShapes.back();
        for (int idx : lastShape.pointIndices) {
            pointUsed[idx] = false;
        }
        
        currentShapes.pop_back();
    }

    return result;
}

bool FitEllipse(
    const std::vector<Point3D>& points,
    Point3D* center,
    float* shortRadio,
    float* longRadio,
    std::vector<Point3D>* ellipse,
    std::vector<Point3D>* incompEllipse)
{
    // 1. 基础防崩检查
    if (points.size() < 6 || !center || !shortRadio || !longRadio) {
        return false;
    }

    Eigen::Index N = static_cast<Eigen::Index>(points.size());
    PCAResult pca = CalPCA(points);

    // 提取投影平面的局部基底
    Eigen::Vector3d u_axis = pca.eigenvectors.col(2);
    Eigen::Vector3d v_axis = pca.eigenvectors.col(1);

    // 投影到二维
    Eigen::MatrixXd points_2d(N, 2);
    for (Eigen::Index i = 0; i < N; ++i) {
        Eigen::Vector3d diff = points[i] - pca.centroid;
        points_2d(i, 0) = diff.dot(u_axis);
        points_2d(i, 1) = diff.dot(v_axis);
    }

    // 构建最小二乘设计矩阵
    Eigen::MatrixXd D_mat(N, 6);
    for (Eigen::Index i = 0; i < N; ++i) {
        double x = points_2d(i, 0);
        double y = points_2d(i, 1);
        D_mat(i, 0) = x * x;
        D_mat(i, 1) = x * y;
        D_mat(i, 2) = y * y;
        D_mat(i, 3) = x;
        D_mat(i, 4) = y;
        D_mat(i, 5) = 1.0;
    }

    Eigen::BDCSVD<Eigen::MatrixXd> svd_ellipse(D_mat, Eigen::ComputeFullV);
    Eigen::VectorXd coeff = svd_ellipse.matrixV().col(5);

    if (coeff(0) < 0) {
        coeff = -coeff;
    }

    double A = coeff(0); double B = coeff(1); double C = coeff(2);
    double D = coeff(3); double E = coeff(4); double F = coeff(5);

    double delta = 4.0 * A * C - B * B;
    if (delta <= 1e-6) {
        return false;
    }

    double x0 = (B * E - 2.0 * C * D) / delta;
    double y0 = (B * D - 2.0 * A * E) / delta;

    // 计算三维中心点
    *center = pca.centroid + x0 * u_axis + y0 * v_axis;

    double H = A * x0 * x0 + B * x0 * y0 + C * y0 * y0 - F;

    Eigen::Matrix2d semi_axis_mat;
    semi_axis_mat << A, B / 2.0,
        B / 2.0, C;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(semi_axis_mat);
    if (eigensolver.info() != Eigen::Success) {
        return false;
    }

    Eigen::Vector2d eigenvalues_2d = eigensolver.eigenvalues();
    Eigen::Matrix2d eigenvectors_2d = eigensolver.eigenvectors();

    double r1_sq = H / eigenvalues_2d(0);
    double r2_sq = H / eigenvalues_2d(1);

    if (r1_sq <= 0.0 || r2_sq <= 0.0) {
        return false;
    }

    double r1 = std::sqrt(r1_sq);
    double r2 = std::sqrt(r2_sq);

    *longRadio = static_cast<float>(r1);
    *shortRadio = static_cast<float>(r2);

    // 计算长短轴在三维空间中的方向向量
    Eigen::Vector2d e_long = eigenvectors_2d.col(0);
    Eigen::Vector2d e_short = eigenvectors_2d.col(1);

    Eigen::Vector3d long_axis_dir = (e_long(0) * u_axis + e_long(1) * v_axis).normalized();
    Eigen::Vector3d short_axis_dir = (e_short(0) * u_axis + e_short(1) * v_axis).normalized();

    const double kPI = 3.14159265358979323846;

    // =======================================================
    // 1️⃣ 输出：填充【全部（闭合）椭圆顶点】
    // =======================================================
    if (ellipse && ellipse->size() > 2) {
        size_t sample_count = ellipse->size();
        for (size_t i = 0; i < sample_count; ++i) {
            double theta = 2.0 * kPI * static_cast<double>(i) / static_cast<double>(sample_count);
            Point3D pt = *center + (r1 * std::cos(theta)) * long_axis_dir
                + (r2 * std::sin(theta)) * short_axis_dir;
            (*ellipse)[i] = pt;
        }
    }

    // =======================================================
    // 2️⃣ 输出：填充【仅覆盖输入 points 的残缺椭圆弧顶点】
    // =======================================================
    if (incompEllipse && incompEllipse->size() > 1) {
        size_t sample_count_incomp = incompEllipse->size();

        // Step A: 把所有输入点投影到椭圆上，计算它们对应的参数方程角度 theta
        std::vector<double> thetas;
        thetas.reserve(points.size());
        for (const auto& p : points) {
            Eigen::Vector3d diff = p - *center;
            double x_local = diff.dot(long_axis_dir);
            double y_local = diff.dot(short_axis_dir);
            // 考虑长短轴缩放后，求出准确的椭圆参数方程角度（范围 -PI 到 PI）
            double theta = std::atan2(y_local / r2, x_local / r1);
            thetas.push_back(theta);
        }

        // Step B: 对角度进行升序排列，寻找最大的空白间隙（Max Gap）
        std::sort(thetas.begin(), thetas.end());

        double max_gap = 0.0;
        size_t max_gap_idx = 0;
        size_t n_thetas = thetas.size();

        for (size_t i = 0; i < n_thetas; ++i) {
            // 下一个角度，如果到了尾部则绕回到开头并加上 2*PI
            double next_theta = (i == n_thetas - 1) ? (thetas[0] + 2.0 * kPI) : thetas[i + 1];
            double gap = next_theta - thetas[i];
            if (gap > max_gap) {
                max_gap = gap;
                max_gap_idx = i; // 记录最大缺口的起点索引
            }
        }

        // Step C: 确定残缺椭圆弧的有效起始角度与结束角度
        // 最大缺口的“下一站”就是残缺弧的【起点】，最大缺口的“本身”就是残缺弧的【终点】
        double start_theta = (max_gap_idx == n_thetas - 1) ? thetas[0] : thetas[max_gap_idx + 1];
        double end_theta = thetas[max_gap_idx];

        // 如果终点角度小于起点角度，说明跨越了 -PI/PI 边界，将其自增 2*PI 便于线性插值
        if (end_theta < start_theta) {
            end_theta += 2.0 * kPI;
        }

        // Step D: 在有效弧度区间内，根据 incompEllipse 的大小进行均匀采样覆盖
        for (size_t i = 0; i < sample_count_incomp; ++i) {
            double factor = static_cast<double>(i) / static_cast<double>(sample_count_incomp - 1);
            double theta = start_theta + factor * (end_theta - start_theta);

            Point3D pt = *center + (r1 * std::cos(theta)) * long_axis_dir
                + (r2 * std::sin(theta)) * short_axis_dir;
            (*incompEllipse)[i] = pt;
        }
    }

    return true;
}