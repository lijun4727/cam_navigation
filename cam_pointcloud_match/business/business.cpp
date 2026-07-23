#include "business.h"
#include "common/math/open3d_kit.h"
#include "common/math/math_kit.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <bit>
#include <functional>
#include <numeric>
#include <unordered_map>
#include <open3d/io/PointCloudIO.h>
#include <open3d/geometry/BoundingVolume.h>
#include <iostream>
#include <vtkPolyData.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>

using namespace Eigen;

Eigen::Matrix4d RegisterPointCloudFromPoints(
	const std::vector<int>& sourceIndexes,
	const std::vector<int>& targetIndexes,
	const PointCloud& source,
	const PointCloud& target,
    FineAlignmentResult* result) {
    open3d::geometry::PointCloud sourcePoints;
    open3d::geometry::PointCloud targetPoints;
    for(int i = 0; i < sourceIndexes.size(); i++) {
        int si = sourceIndexes[i];
        int ti = targetIndexes[i];
        sourcePoints.points_.emplace_back(source.points_[si]);
        targetPoints.points_.emplace_back(target.points_[ti]);
	}
    DEBUG_WRITE_PLY("D:/sourcePoints.ply", sourcePoints);
    DEBUG_WRITE_PLY("D:/targetPoints.ply", targetPoints);

    auto T1 = ComputeTransformationFromCorrespondences(sourcePoints, targetPoints);
    sourcePoints.Transform(T1);
    DEBUG_WRITE_PLY("D:/sourcePoints.ply", sourcePoints);

	auto sourceTemp = source;
    sourceTemp.Transform(T1);
    DEBUG_WRITE_PLY("D:/source.ply", sourceTemp);

    auto box = sourceTemp.GetOrientedBoundingBox();
    auto targetInBox = target.Crop(box);
    DEBUG_WRITE_PLY("D:/targetInBox.ply", *targetInBox);

    auto res = FineAlignment(
        std::make_shared<open3d::geometry::PointCloud>(sourceTemp),
        targetInBox);

	auto T2 = res.transformation;
    sourceTemp.Transform(T2);
    DEBUG_WRITE_PLY("D:/source.ply", sourceTemp);

    if (result)
        *result = res;

    return T2 * T1;
}

// 将 Open3D TriangleMesh 转为 vtkPolyData
PolyDataPtr Open3DMeshToVtkPolyData(const TriangleMeshPtr& mesh)
{
    if (!mesh || mesh->IsEmpty()) return nullptr;

    auto pts = vtkSmartPointer<vtkPoints>::New();
    pts->SetNumberOfPoints(static_cast<vtkIdType>(mesh->vertices_.size()));
    for (size_t i = 0; i < mesh->vertices_.size(); ++i) {
        const auto& v = mesh->vertices_[i];
        pts->SetPoint(static_cast<vtkIdType>(i), v.x(), v.y(), v.z());
    }

    auto polys = vtkSmartPointer<vtkCellArray>::New();
    for (const auto& tri : mesh->triangles_) {
        vtkIdType ids[3];
        ids[0] = tri[0];
        ids[1] = tri[1];
        ids[2] = tri[2];
        polys->InsertNextCell(3, ids);
    }

    auto poly = vtkSmartPointer<vtkPolyData>::New();
    poly->SetPoints(pts);
    poly->SetPolys(polys);

    // 顶点法线（如果有）
    if (mesh->HasVertexNormals()) {
        auto normals = vtkSmartPointer<vtkFloatArray>::New();
        normals->SetNumberOfComponents(3);
        normals->SetName("Normals");
        for (const auto& n : mesh->vertex_normals_) {
            normals->InsertNextTuple3(static_cast<float>(n.x()),
                static_cast<float>(n.y()),
                static_cast<float>(n.z()));
        }
        poly->GetPointData()->SetNormals(normals);
    }

    // 顶点颜色（如果有），转换为 unsigned char [0,255]
    if (mesh->HasVertexColors()) {
        auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);
        colors->SetName("Colors");
        for (const auto& c : mesh->vertex_colors_) {
            unsigned char r = static_cast<unsigned char>(std::clamp<int>(static_cast<int>(c.x() * 255.0 + 0.5), 0, 255));
            unsigned char g = static_cast<unsigned char>(std::clamp<int>(static_cast<int>(c.y() * 255.0 + 0.5), 0, 255));
            unsigned char b = static_cast<unsigned char>(std::clamp<int>(static_cast<int>(c.z() * 255.0 + 0.5), 0, 255));
            unsigned char tuple[3] = { r, g, b };
            colors->InsertNextTypedTuple(tuple);
        }
        poly->GetPointData()->SetScalars(colors);
    }

    return poly;
}

CAM_API PolyDataPtr Open3DPointCloudToVtkPolyData(const PointCloud& cloud)
{
    auto poly = vtkSmartPointer<vtkPolyData>::New();

    if (cloud.IsEmpty()) {
        return poly;
    }

    auto pts = vtkSmartPointer<vtkPoints>::New();
    pts->SetNumberOfPoints(static_cast<vtkIdType>(cloud.points_.size()));
    for (size_t i = 0; i < cloud.points_.size(); ++i) {
        const auto& p = cloud.points_[i];
        pts->SetPoint(static_cast<vtkIdType>(i), p.x(), p.y(), p.z());
    }
    poly->SetPoints(pts);

    // 构建顶点单元，确保按点云方式渲染
    auto verts = vtkSmartPointer<vtkCellArray>::New();
    for (vtkIdType i = 0; i < static_cast<vtkIdType>(cloud.points_.size()); ++i) {
        verts->InsertNextCell(1);
        verts->InsertCellPoint(i);
    }
    poly->SetVerts(verts);

    // 法线（如果有）
    if (cloud.HasNormals()) {
        auto normals = vtkSmartPointer<vtkFloatArray>::New();
        normals->SetNumberOfComponents(3);
        normals->SetName("Normals");
        for (const auto& n : cloud.normals_) {
            normals->InsertNextTuple3(static_cast<float>(n.x()),
                static_cast<float>(n.y()),
                static_cast<float>(n.z()));
        }
        poly->GetPointData()->SetNormals(normals);
    }

    // 颜色（如果有），转换到 unsigned char [0,255]
    if (cloud.HasColors()) {
        auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);
        colors->SetName("Colors");
        for (const auto& c : cloud.colors_) {
            unsigned char r = static_cast<unsigned char>(std::clamp<int>(static_cast<int>(c.x() * 255.0 + 0.5), 0, 255));
            unsigned char g = static_cast<unsigned char>(std::clamp<int>(static_cast<int>(c.y() * 255.0 + 0.5), 0, 255));
            unsigned char b = static_cast<unsigned char>(std::clamp<int>(static_cast<int>(c.z() * 255.0 + 0.5), 0, 255));
            unsigned char tuple[3] = { r, g, b };
            colors->InsertNextTypedTuple(tuple);
        }
        poly->GetPointData()->SetScalars(colors);
    }

    return poly;
}

CAM_API ActorPtr VtkPolyDataToActor(const PolyDataPtr& poly)
{
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(poly);
    mapper->ScalarVisibilityOn();

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    actor->PickableOn();

    return actor;
}

CAM_API ActorPtr MeshToActor(const TriangleMeshPtr& mesh)
{
    auto poly = Open3DMeshToVtkPolyData(mesh);
    return VtkPolyDataToActor(poly);
}

CAM_API ActorPtr PointCloudToActor(const PointCloudPtr& pointCloud)
{
    auto ploy = Open3DPointCloudToVtkPolyData(*pointCloud);
    return VtkPolyDataToActor(ploy);
}

CAM_API PointCloudPtr ActorToPointCloud(const ActorPtr& actor)
{
    if (!actor) return nullptr;

    auto* mapper = actor->GetMapper();
    if (!mapper) return nullptr;

    // Try to get input polydata from mapper
    vtkPolyData* poly = vtkPolyData::SafeDownCast(mapper->GetInput());
    if (!poly) return nullptr;

    auto cloud = std::make_shared<PointCloud>();

    vtkPoints* pts = poly->GetPoints();
    if (!pts) return cloud;

    const vtkIdType n = pts->GetNumberOfPoints();
    cloud->points_.reserve(static_cast<size_t>(n));

    for (vtkIdType i = 0; i < n; ++i) {
        double p[3] = { 0.0, 0.0, 0.0 };
        pts->GetPoint(i, p);
        cloud->points_.emplace_back(p[0], p[1], p[2]);
    }

    // Normals (if available)
    vtkDataArray* normals = poly->GetPointData() ? poly->GetPointData()->GetNormals() : nullptr;
    if (normals && normals->GetNumberOfComponents() == 3) {
        cloud->normals_.resize(static_cast<size_t>(n));
        for (vtkIdType i = 0; i < n; ++i) {
            double nv[3] = { 0.0, 0.0, 0.0 };
            normals->GetTuple(i, nv);
            cloud->normals_[static_cast<size_t>(i)] = Point3D(nv[0], nv[1], nv[2]);
        }
    }

    // Colors (if available)
    vtkDataArray* scalars = poly->GetPointData() ? poly->GetPointData()->GetScalars() : nullptr;
    if (scalars && scalars->GetNumberOfComponents() >= 3) {
        cloud->colors_.resize(static_cast<size_t>(n));
        // Use GetTuple which will convert underlying types to double
        std::vector<double> tuple(scalars->GetNumberOfComponents());
        for (vtkIdType i = 0; i < n; ++i) {
            scalars->GetTuple(i, tuple.data());
            double r = tuple[0];
            double g = tuple[1];
            double b = tuple[2];
            // If colors appear to be in 0-255 range, normalize to 0-1
            if (r > 1.1 || g > 1.1 || b > 1.1) {
                r /= 255.0; g /= 255.0; b /= 255.0;
            }
            cloud->colors_[static_cast<size_t>(i)] = Eigen::Vector3d(r, g, b);
        }
    }

    return cloud;
}

CAM_API void ShowActor(
    const RenderPtr& render, 
    const ActorPtr& actor,
    bool resetCamRange)
{
	render->AddActor(actor);
    if(resetCamRange)
        render->ResetCameraClippingRange();
}

CAM_API Point3D MaxPointFromVec(const PointCloudPtr& cloud, const Eigen::Vector3d& vec)
{
    if (!cloud || cloud->IsEmpty()) return InvalidPoint3D();
    if (vec.squaredNorm() <= 0.0) return InvalidPoint3D();

    Eigen::Vector3d dir = vec.normalized();
    double bestProj = -std::numeric_limits<double>::infinity();
    Point3D bestP = InvalidPoint3D();    
    for (const auto& p : cloud->points_) {
        if (!IsValidPoint3D(p)) continue;
        double proj = p.dot(dir);
        if (proj > bestProj) {
            bestProj = proj;
            bestP = p;
        }
    }

    if (!IsValidPoint3D(bestP)) return InvalidPoint3D();
    return bestP;
}

CAM_API CalibrationInfo CalibrateMeshFromReference(
    const std::vector<PointCloudPtr>& sources,
    const std::vector<PointCloudPtr>& targets,
    std::vector<TriangleMeshPtr>& meshes,
    std::vector<PointCloudPtr>* pointClouds) {
	auto calAVGPoint = [](const std::vector<PointCloudPtr>& clouds)
		{
			PointCloud avgCloud;
			int i = 0;
			bool allDone = false;
			while (true) {
				PointCloud points;
				for (const auto& cloud : clouds) {
					if (cloud->points_.size() <= i) {
						allDone = true;
						break;
					}
					points.points_.emplace_back(cloud->points_[i]);
				}
                if(allDone)
					break;
				avgCloud.points_.emplace_back(points.GetCenter());
				i++;
			}
            avgCloud.colors_.emplace_back(1, 0, 0);
            avgCloud.colors_.emplace_back(0, 1, 0);
            avgCloud.colors_.emplace_back(0, 0, 1);
            avgCloud.colors_.emplace_back(1, 1, 0);
			return avgCloud;
		};

	auto sourceFeatures = calAVGPoint(sources);
	auto targetFeatures = calAVGPoint(targets);
    if(sourceFeatures.points_.size() != targetFeatures.points_.size()) {
        return {};
    }

    MakesSmallestFirstPoint(sourceFeatures);
    MakesSmallestFirstPoint(targetFeatures);

	auto T = ComputeTransformationFromCorrespondences(
		sourceFeatures, targetFeatures);
	for (auto& mesh : meshes) {
		if (mesh) {
			mesh->Transform(T);
		}
	}
	if (pointClouds) {
		for (auto& cloud : *pointClouds) {
			if (cloud) {
				cloud->Transform(T);
			}
		}
	}

	return { sourceFeatures, targetFeatures, T };
}

CAM_API void MakesSmallestFirstPoint(PointCloud& pointCloud)
{
    const size_t pointCount = pointCloud.points_.size();
    if (pointCount < 2) {
        return;
    }

    double minDistance2 = std::numeric_limits<double>::infinity();
    size_t minIndex = 0;
    for (size_t i = 0; i < pointCount; ++i) {
        const auto& current = pointCloud.points_[i];
        const auto& next = pointCloud.points_[(i + 1) % pointCount];
        if (!IsValidPoint3D(current) || !IsValidPoint3D(next)) {
            continue;
        }

        const double distance2 = (current - next).squaredNorm();
        if (distance2 < minDistance2) {
            minDistance2 = distance2;
            minIndex = i;
        }
    }

    if (!std::isfinite(minDistance2)) {
        return;
    }

    auto rotateVector = [minIndex, pointCount](auto& values) {
        if (values.size() != pointCount) {
            return;
        }

        using ValueType = typename std::decay_t<decltype(values)>::value_type;
        std::vector<ValueType> reordered;
        reordered.reserve(pointCount);
        for (size_t i = 0; i < pointCount; ++i) {
            reordered.emplace_back(values[(minIndex + i) % pointCount]);
        }
        values.swap(reordered);
    };

    rotateVector(pointCloud.points_);
    rotateVector(pointCloud.normals_);
    rotateVector(pointCloud.colors_);
}

CAM_API void MakaGradient(
    PointCloudPtr pointCloud,
    Eigen::Vector3d startColor,
    Eigen::Vector3d endColor)
{
    // 空指针或空点云时没有任何颜色可写，直接返回。
    if (!pointCloud || pointCloud->points_.empty()) {
        return;
    }

    // Open3D 的颜色通常使用 [0, 1] 范围的 RGB，因此这里做一次钳制，
    // 避免调用方传入越界颜色后在后续显示或写文件时产生异常结果。
    startColor = startColor.cwiseMax(0.0).cwiseMin(1.0);
    endColor = endColor.cwiseMax(0.0).cwiseMin(1.0);

    const size_t pointCount = pointCloud->points_.size();

    // 颜色数组需要与点数组一一对应，因此直接按点数重建 colors_。
    pointCloud->colors_.resize(pointCount);

    // 只有一个点时，“第一个点”和“最后一个点”是同一个点。
    // 按注释语义，让它使用 startColor 即可。
    if (pointCount == 1) {
        pointCloud->colors_[0] = startColor;
        return;
    }

    // 对每一个点按索引位置做线性插值：
    // t = 0   -> 第一个点，颜色为 startColor
    // t = 1   -> 最后一个点，颜色为 endColor
    // 0<t<1   -> 中间点，颜色位于 startColor 和 endColor 之间
    //
    // 这里用索引比值而不是空间距离，是因为头文件注释明确写的是“根据索引的距离”。
    const double denominator = static_cast<double>(pointCount - 1);
    for (size_t i = 0; i < pointCount; ++i) {
        const double t = static_cast<double>(i) / denominator;
        pointCloud->colors_[i] = startColor * (1.0 - t) + endColor * t;
    }
}

CAM_API OBJECT_VEC CalculateObject(
    const PointCloudPtr& pointCloud,
    const OBJECT_MODEL_VEC& objectModels,
    double threshold,
    double minSideDist,
    double maxSideDist)
{
    OBJECT_VEC objects;

    // 输入为空时，无法进行任何分类，直接返回空结果。
    if (!pointCloud || pointCloud->IsEmpty() || objectModels.empty()) {
        return objects;
    }

    // 先复制一份输入点云，后续会在这份点云上删除离群点，避免修改调用方传入的数据。
    auto validPointCloud = std::make_shared<PointCloud>(*pointCloud);

    // 删除离群点：如果某个点到其最近点的距离都超过 maxSideDist，则视为离群点。
    DeleteOuterPoints(validPointCloud, maxSideDist);
    if (!validPointCloud || validPointCloud->IsEmpty()) {
        return objects;
    }

    // 统计模型信息：
    // 1. minVertexNum: 每个形状至少需要多少个顶点；
    // 2. maxVertexNum: 每个形状至多允许多少个顶点；
    // 3. maxPlaneNum : 平面切割时，一个切割法中最多允许的平面数量。
    // 同时，把每个模型点云先按照 ArrangePointCloud 的规则排序，
    // 这样后续按对应点做刚体对齐时，点序具有一致的几何意义。
    int minVertexNum = std::numeric_limits<int>::max();
    int maxVertexNum = 0;
    int maxPlaneNum = 0;
    OBJECT_MODEL_VEC arrangedObjectModels;
    arrangedObjectModels.reserve(objectModels.size());
    for (const auto& info : objectModels) {
        if (!info.cloud || info.cloud->points_.empty()) {
            continue;
        }

        const int vertexNum = static_cast<int>(info.cloud->points_.size());
        minVertexNum = std::min(minVertexNum, vertexNum);
        maxVertexNum = std::max(maxVertexNum, vertexNum);
        ++maxPlaneNum;

        auto arrangedCloud = std::make_shared<PointCloud>(*info.cloud);
        ArrangePointCloud(arrangedCloud);
        arrangedObjectModels.push_back({ info.type, arrangedCloud });
    }

    // 没有任何合法模型时，无法完成分类。
    if (arrangedObjectModels.empty() ||
        minVertexNum == std::numeric_limits<int>::max() ||
        maxVertexNum <= 0) {
        return objects;
    }

    // 第一步：先对整个点云做“平面切割”。
    // 每一种切割法，表示把点云分成若干个近似共面的平面集合。
    auto cuts = PlaneCut(
        validPointCloud,
        minVertexNum,
        maxPlaneNum,
        minSideDist,
        maxSideDist,
        threshold);
    if (cuts.empty()) {
        return objects;
    }

    // 根据索引从点云中提取子点云，并尽可能保留颜色与法线数据。
    auto extractedPointCloud = [](const PointCloudPtr& sourcePointCloud, const VecInt& pointIndexes) {
        auto extractedCloud = std::make_shared<PointCloud>();
        for (int index : pointIndexes) {
            if (index < 0 || index >= static_cast<int>(sourcePointCloud->points_.size())) {
                continue;
            }

            extractedCloud->points_.emplace_back(sourcePointCloud->points_[index]);
            if (sourcePointCloud->HasColors() && index < static_cast<int>(sourcePointCloud->colors_.size())) {
                extractedCloud->colors_.emplace_back(sourcePointCloud->colors_[index]);
            }
            if (sourcePointCloud->HasNormals() && index < static_cast<int>(sourcePointCloud->normals_.size())) {
                extractedCloud->normals_.emplace_back(sourcePointCloud->normals_[index]);
            }
        }
        return extractedCloud;
    };

    // SHAPE_CUT 表示“一种完整的形状切割法”：
    // 它由多个形状组成，每个形状用一个索引数组表示（索引基于 validPointCloud）。
    using SHAPE_CUT = std::vector<VecInt>;
    using SHAPE_CUT_VEC = std::vector<SHAPE_CUT>;

    // 记录某个形状最终匹配到的模型类型、点索引、点云以及 MAE，
    // 以便在找到最佳方案后，直接组装为返回值。
    struct MatchResult {
        ObjectType type = OT_UNKNOWN;
        VecInt indices;
        PointCloudPtr cloud;
        double mae = std::numeric_limits<double>::infinity();
    };

    // 把“每个平面的多种形状切割法”组合成“完整的形状切割法”。
    // 这里本质上是在做笛卡尔积：
    // 假设平面 A 有 2 种切法，平面 B 有 3 种切法，则总共有 2*3=6 种完整切法。
    // 为了提高可读性，使用迭代的“里程表进位”方式，而不是递归 DFS。
    auto buildShapeCuts = [](const std::vector<PLANE_SHAPE_CUT_VEC>& planeShapeCutVec) {
        SHAPE_CUT_VEC shapeCutVec;
        if (planeShapeCutVec.empty()) {
            return shapeCutVec;
        }

        int shapeCutNum = 1;
        for (const auto& cutsOfPlane : planeShapeCutVec) {
            if (cutsOfPlane.empty()) {
                return SHAPE_CUT_VEC();
            }
            shapeCutNum *= static_cast<int>(cutsOfPlane.size());
        }

        shapeCutVec.reserve(shapeCutNum);

        const int planeCount = static_cast<int>(planeShapeCutVec.size());
        std::vector<int> indices(planeCount, 0);

        while (true) {
            SHAPE_CUT current;
            for (int planeIndex = 0; planeIndex < planeCount; ++planeIndex) {
                const auto& onePlaneShapeCut =
                    planeShapeCutVec[static_cast<size_t>(planeIndex)][static_cast<size_t>(indices[planeIndex])];
                current.insert(current.end(), onePlaneShapeCut.begin(), onePlaneShapeCut.end());
            }
            shapeCutVec.push_back(std::move(current));

            // 里程表进位：从最后一个平面开始递增，如果溢出则清零并向前进位。
            int pos = planeCount - 1;
            for (; pos >= 0; --pos) {
                ++indices[pos];
                if (indices[pos] < static_cast<int>(planeShapeCutVec[static_cast<size_t>(pos)].size())) {
                    break;
                }
                indices[pos] = 0;
            }

            // 所有位都进位完成，说明所有组合都已遍历完毕。
            if (pos < 0) {
                break;
            }
        }

        return shapeCutVec;
    };

    // 对一种完整的形状切割法进行打分。
    // 打分规则严格按照 business.h 注释：
    // 1. 每个形状先提取 objectPC；
    // 2. 对 objectPC 排序；
    // 3. 与所有“点数相同”的模型点云做刚体对齐，计算 MAE；
    // 4. 选择 MAE 最小的模型；
    // 5. 如果 minMAE > threshold * 2，则该完整切割法无效；
    // 6. 否则每个成功匹配的物体得分为 objectThreshold - minMAE。
    auto evaluateShapeCut = [&](const SHAPE_CUT& shapeCut, std::vector<MatchResult>* outMatches) {
        double score = 0.0;
        std::vector<MatchResult> localMatches;
        const double objectThreshold = threshold * 2.0;

        for (const auto& shapeIndices : shapeCut) {
            // 从场景点云中提取当前形状对应的子点云。
            auto objectPC = extractedPointCloud(validPointCloud, shapeIndices);
            if (!objectPC || objectPC->points_.empty()) {
                return -std::numeric_limits<double>::infinity();
            }

            // 对待识别形状点云排序，确保点序与模型排序规则一致。
            ArrangePointCloud(objectPC);

            double minMae = std::numeric_limits<double>::infinity();
            ObjectType bestType = OT_UNKNOWN;

            // 只与“点数相同”的模型比较，因为 ComputeTransformationFromCorrespondences
            // 要求 source 和 target 的点数必须一致，且这里是按点对一一对应求刚体变换。
            for (const auto& model : arrangedObjectModels) {
                if (!model.cloud || model.cloud->points_.size() != objectPC->points_.size()) {
                    continue;
                }

                // 通过对应点计算 objectPC 到模型点云的刚体变换，
                // 同时由函数内部返回对齐后的平均绝对误差 MAE。
                double mae = std::numeric_limits<double>::infinity();
                ComputeTransformationFromCorrespondences(*objectPC, *model.cloud, &mae);

                if (mae < minMae) {
                    minMae = mae;
                    bestType = model.type;
                }
            }

            // 没找到同点数模型，或者最小 MAE 超过阈值，则该完整切割法无效。
            if (bestType == OT_UNKNOWN || minMae > objectThreshold) {
                return -std::numeric_limits<double>::infinity();
            }

            score += objectThreshold - minMae;
            localMatches.push_back({ bestType, shapeIndices, objectPC, minMae });
        }

        if (outMatches) {
            *outMatches = std::move(localMatches);
        }
        return score;
    };

    // 遍历每一种“平面切割法”，在其中寻找得分最高的“完整形状切割法”。
    // 再在所有平面切割法之间，选出全局得分最高的一种，作为最终分类结果。
    double bestScore = -std::numeric_limits<double>::infinity();
    std::vector<MatchResult> bestMatches;

    for (auto itCut = cuts.begin(); itCut != cuts.end(); ++itCut) {
        // 每个元素对应当前平面切割法中，每个平面的所有形状切割法。
        std::vector<PLANE_SHAPE_CUT_VEC> planeShapeCutVec(itCut->size());
        bool isValidPlaneCut = true;

        for (size_t i = 0; i < itCut->size(); ++i) {
            const auto& planeIndices = (*itCut)[i];

            // 提取当前平面的点云，然后对该平面做形状切割。
            auto planeCloud = extractedPointCloud(validPointCloud, planeIndices);
            planeShapeCutVec[i] = PlaneShapeCut(
                planeCloud,
                minVertexNum,
                maxVertexNum,
                minSideDist,
                maxSideDist,
                threshold);

            // PlaneShapeCut 返回的索引是基于 planeCloud 的局部索引，
            // 这里需要把它映射回 validPointCloud 的全局索引。
            for (auto& oneCut : planeShapeCutVec[i]) {
                for (auto& shapeIndices : oneCut) {
                    for (auto& index : shapeIndices) {
                        index = planeIndices[static_cast<size_t>(index)];
                    }
                }
            }

            // 某个平面没有任何合法形状切割法，则整种平面切割法都无效。
            if (planeShapeCutVec[i].empty()) {
                isValidPlaneCut = false;
                break;
            }
        }

        if (!isValidPlaneCut) {
            continue;
        }

        // 将每个平面的形状切割法做笛卡尔积组合，得到完整的形状切割法集合。
        SHAPE_CUT_VEC shapeCutVec = buildShapeCuts(planeShapeCutVec);
        for (const auto& shapeCut : shapeCutVec) {
            std::vector<MatchResult> matches;
            const double score = evaluateShapeCut(shapeCut, &matches);
            if (score > bestScore) {
                bestScore = score;
                bestMatches = std::move(matches);
            }
        }
    }

    // 把最佳匹配结果转换成最终的 OBJECT_VEC 输出结构。
    for (const auto& match : bestMatches) {
        if (match.type == OT_UNKNOWN) {
            continue;
        }
        objects[match.type].push_back({ match.indices, match.cloud });
    }

    return objects;
}

CAM_API OBJECT_UNMAP CalculateObject2(
    const PointCloudPtr& pointCloud,
    const OBJECT_UNMAP& objectModels,
    double threshold,
    double minSideDist,
    double maxSideDist)
{
    OBJECT_UNMAP bestObjects;

    if (!pointCloud || pointCloud->IsEmpty() || objectModels.empty()) {
        return bestObjects;
    }

    auto validPointCloud = std::make_shared<PointCloud>(*pointCloud);
    DeleteOuterPoints(validPointCloud, maxSideDist);
    if (!validPointCloud || validPointCloud->IsEmpty()) {
        return bestObjects;
    }

    struct ModelEntry {
        ObjectType type = OT_UNKNOWN;
        PointCloudPtr cloud;
        int pointNum = 0;
    };

    std::vector<ModelEntry> modelEntries;
    modelEntries.reserve(objectModels.size());
    for (const auto& [type, cloud] : objectModels) {
        if (!cloud || cloud->points_.empty()) {
            continue;
        }

        auto arrangedCloud = std::make_shared<PointCloud>(*cloud);
        ArrangePointCloud(arrangedCloud);

        modelEntries.push_back({
            type,
            arrangedCloud,
            static_cast<int>(arrangedCloud->points_.size())
            });
    }

    if (modelEntries.empty()) {
        return bestObjects;
    }

    // 点数多的模型优先枚举，可稍微减少搜索树分支。
    std::sort(modelEntries.begin(), modelEntries.end(),
        [](const ModelEntry& a, const ModelEntry& b) {
            if (a.pointNum != b.pointNum) {
                return a.pointNum > b.pointNum;
            }
            return static_cast<int>(a.type) < static_cast<int>(b.type);
        });

    auto extractPointCloud = [](const PointCloudPtr& sourcePointCloud, const VecInt& pointIndexes) {
        auto extractedCloud = std::make_shared<PointCloud>();
        extractedCloud->points_.reserve(pointIndexes.size());

        const bool hasColors = sourcePointCloud->HasColors();
        const bool hasNormals = sourcePointCloud->HasNormals();
        if (hasColors) {
            extractedCloud->colors_.reserve(pointIndexes.size());
        }
        if (hasNormals) {
            extractedCloud->normals_.reserve(pointIndexes.size());
        }

        for (int index : pointIndexes) {
            if (index < 0 || index >= static_cast<int>(sourcePointCloud->points_.size())) {
                continue;
            }

            extractedCloud->points_.emplace_back(sourcePointCloud->points_[index]);

            if (hasColors && index < static_cast<int>(sourcePointCloud->colors_.size())) {
                extractedCloud->colors_.emplace_back(sourcePointCloud->colors_[index]);
            }
            if (hasNormals && index < static_cast<int>(sourcePointCloud->normals_.size())) {
                extractedCloud->normals_.emplace_back(sourcePointCloud->normals_[index]);
            }
        }

        return extractedCloud;
        };

    auto subtractIndices = [](const VecInt& source, const VecInt& selected) {
        VecInt remain;
        remain.reserve(source.size() - selected.size());

        size_t j = 0;
        for (int value : source) {
            if (j < selected.size() && value == selected[j]) {
                ++j;
            }
            else {
                remain.emplace_back(value);
            }
        }

        return remain;
        };

    auto validateObjectCloud = [&](PointCloudPtr objectCloud) -> bool {
        if (!objectCloud) {
            return false;
        }

        const size_t pointCount = objectCloud->points_.size();
        if (pointCount < 3) {
            return false;
        }

        Plane fitPlane = FitPlaneFromPoints(objectCloud->points_);
        if (fitPlane.normal.squaredNorm() <= 1e-12) {
            return false;
        }

        for (const auto& point : objectCloud->points_) {
            if (PointToPlaneDistance(point, fitPlane) > threshold) {
                return false;
            }
        }

        ArrangePointCloud(objectCloud);

        fitPlane = FitPlaneFromPoints(objectCloud->points_);
        if (fitPlane.normal.squaredNorm() <= 1e-12) {
            return false;
        }

        std::vector<Point3D> projectedPoints =
            ProjectPointsToPlane(objectCloud->points_, fitPlane);
        if (projectedPoints.size() != pointCount) {
            return false;
        }

        constexpr double kEps = 1e-8;
        double turnSign = 0.0;

        for (size_t i = 0; i < pointCount; ++i) {
            const Point3D& p0 = projectedPoints[i];
            const Point3D& p1 = projectedPoints[(i + 1) % pointCount];
            const Point3D& p2 = projectedPoints[(i + 2) % pointCount];

            const double sideLength = (p1 - p0).norm();
            if (sideLength < minSideDist || sideLength > maxSideDist) {
                return false;
            }

            const double turn =
                (p1 - p0).cross(p2 - p1).dot(fitPlane.normal);
            if (std::abs(turn) <= kEps) {
                continue;
            }

            const double sign = turn > 0.0 ? 1.0 : -1.0;
            if (turnSign == 0.0) {
                turnSign = sign;
            }
            else if (turnSign != sign) {
                return false;
            }
        }

        return turnSign != 0.0;
        };

    struct SelectedObject {
        ObjectType type = OT_UNKNOWN;
        VecInt indices;
        PointCloudPtr cloud;
        double mae = std::numeric_limits<double>::infinity();
    };

    auto isCompatibleWithSelected =
        [](const PointCloudPtr& candidateCloud,
            const std::vector<SelectedObject>& selectedObjects) {
                constexpr double kIntersectEps = 1e-6;

                for (const auto& selected : selectedObjects) {
                    if (!selected.cloud) {
                        continue;
                    }

                    const double polyDistance =
                        PolygonToPolygonDistance(
                            candidateCloud->points_,
                            selected.cloud->points_);

                    if (polyDistance <= kIntersectEps) {
                        return false;
                    }
                }

                return true;
        };

    auto enumerateCombinations =
        [](const VecInt& source,
            int chooseNum,
            const std::function<void(const VecInt&)>& onCombination) {
                if (chooseNum < 0 || chooseNum > static_cast<int>(source.size())) {
                    return;
                }

                VecInt current;
                current.reserve(static_cast<size_t>(chooseNum));

                std::function<void(int, int)> dfs =
                    [&](int start, int need) {
                    if (need == 0) {
                        onCombination(current);
                        return;
                    }

                    const int maxStart =
                        static_cast<int>(source.size()) - need;
                    for (int i = start; i <= maxStart; ++i) {
                        current.emplace_back(source[static_cast<size_t>(i)]);
                        dfs(i + 1, need - 1);
                        current.pop_back();
                    }
                    };

                dfs(0, chooseNum);
        };

    VecInt allIndices(validPointCloud->points_.size());
    std::iota(allIndices.begin(), allIndices.end(), 0);

    double bestSumMae = std::numeric_limits<double>::infinity();
    int bestMatchedPointNum = -1;
    std::vector<SelectedObject> currentSelected;
    std::vector<SelectedObject> bestSelected;

    auto search = [&](auto&& self,
        size_t modelIndex,
        const VecInt& remainingIndices,
        double currentSumMae) -> void {
            if (modelIndex >= modelEntries.size()) {
                const int matchedPointNum =
                    static_cast<int>(validPointCloud->points_.size() - remainingIndices.size());

                if (!currentSelected.empty()) {
                    if (matchedPointNum > bestMatchedPointNum ||
                        (matchedPointNum == bestMatchedPointNum && currentSumMae < bestSumMae)) {
                        bestMatchedPointNum = matchedPointNum;
                        bestSumMae = currentSumMae;
                        bestSelected = currentSelected;
                    }
                }
                return;
            }

            // 分支1：当前物体类型不参与本次组合
            self(self, modelIndex + 1, remainingIndices, currentSumMae);

            const auto& model = modelEntries[modelIndex];
            if (model.pointNum <= 0 ||
                remainingIndices.size() < static_cast<size_t>(model.pointNum)) {
                return;
            }

            // 分支2：当前物体类型参与组合，从剩余点中选取同点数的组合
            enumerateCombinations(
                remainingIndices,
                model.pointNum,
                [&](const VecInt& combination) {
                    auto objectCloud = extractPointCloud(validPointCloud, combination);
                    if (!validateObjectCloud(objectCloud)) {
                        return;
                    }

                    if (!isCompatibleWithSelected(objectCloud, currentSelected)) {
                        return;
                    }

                    double mae = std::numeric_limits<double>::infinity();
                    ComputeTransformationFromCorrespondences(
                        *objectCloud,
                        *model.cloud,
                        &mae);

                    if (mae == std::numeric_limits<double>::infinity()) {
                        return;
                    }

                    currentSelected.push_back({
                        model.type,
                        combination,
                        objectCloud,
                        mae
                        });

                    VecInt nextRemaining = subtractIndices(remainingIndices, combination);
                    self(self, modelIndex + 1, nextRemaining, currentSumMae + mae);

                    currentSelected.pop_back();
                });
        };

    search(search, 0, allIndices, 0.0);

    for (const auto& selected : bestSelected) {
        if (selected.type == OT_UNKNOWN || !selected.cloud) {
            continue;
        }
        bestObjects[selected.type] = selected.cloud;
    }

    return bestObjects;
}

CAM_API OBJECT_UNMAP CalculateObject3(
    const PointCloudPtr& pointCloud,
    const OBJECT_UNMAP& objectModels,
    double threshold,
    double minSideDist,
    double maxSideDist)
{
    OBJECT_UNMAP bestObjects;

    if (!pointCloud || pointCloud->IsEmpty() || objectModels.empty()) {
        return bestObjects;
    }

    auto validPointCloud = std::make_shared<PointCloud>(*pointCloud);
    DeleteOuterPoints(validPointCloud, maxSideDist);
    if (!validPointCloud || validPointCloud->IsEmpty()) {
        return bestObjects;
    }

    struct ModelEntry {
        ObjectType type = OT_UNKNOWN;
        PointCloudPtr cloud;
        int pointNum = 0;
        std::vector<double> pairDistances;
        uint32_t typeBit = 0;
    };

    auto buildPairDistances = [](const std::vector<Point3D>& points) {
        std::vector<double> distances;
        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t j = i + 1; j < points.size(); ++j) {
                distances.push_back((points[i] - points[j]).norm());
            }
        }
        std::sort(distances.begin(), distances.end());
        return distances;
    };

    auto reorderPointCloud = [](const PointCloud& source, const std::vector<int>& indices) {
        PointCloud reordered;
        reordered.points_.reserve(indices.size());
        for (int index : indices) {
            reordered.points_.push_back(source.points_[static_cast<size_t>(index)]);
        }

        if (source.colors_.size() == source.points_.size()) {
            reordered.colors_.reserve(indices.size());
            for (int index : indices) {
                reordered.colors_.push_back(source.colors_[static_cast<size_t>(index)]);
            }
        }

        if (source.normals_.size() == source.points_.size()) {
            reordered.normals_.reserve(indices.size());
            for (int index : indices) {
                reordered.normals_.push_back(source.normals_[static_cast<size_t>(index)]);
            }
        }

        return reordered;
    };

    auto getAllOrderings = [](size_t pointCount) -> const std::vector<std::vector<int>>& {
        static std::unordered_map<size_t, std::vector<std::vector<int>>> cache;

        auto it = cache.find(pointCount);
        if (it != cache.end()) {
            return it->second;
        }

        std::vector<std::vector<int>> orderings;
        std::vector<int> baseIndices(pointCount);
        std::iota(baseIndices.begin(), baseIndices.end(), 0);

        for (size_t shift = 0; shift < pointCount; ++shift) {
            std::vector<int> ordered(pointCount);
            for (size_t i = 0; i < pointCount; ++i) {
                ordered[i] = baseIndices[(shift + i) % pointCount];
            }
            orderings.push_back(ordered);

            std::reverse(ordered.begin(), ordered.end());
            orderings.push_back(ordered);
        }

        return cache.emplace(pointCount, std::move(orderings)).first->second;
    };

    struct OrderedMatchResult {
        double mae = std::numeric_limits<double>::infinity();
        PointCloud orderedCloud;
    };

    auto calcMinMaeByAllOrders = [&](const PointCloud& candidateCloud, const PointCloud& modelCloud) {
        const size_t pointCount = candidateCloud.points_.size();
        if (pointCount == 0 || pointCount != modelCloud.points_.size()) {
            return OrderedMatchResult{};
        }

        OrderedMatchResult result;
        const auto& orderings = getAllOrderings(pointCount);
        for (const auto& ordered : orderings) {
            double mae = std::numeric_limits<double>::infinity();
            PointCloud reordered = reorderPointCloud(candidateCloud, ordered);
            ComputeTransformationFromCorrespondences(reordered, modelCloud, &mae);
            if (mae < result.mae) {
                result.mae = mae;
                result.orderedCloud = std::move(reordered);
            }
        }

        return result;
    };

    auto calcSignatureError = [](const std::vector<double>& a, const std::vector<double>& b) {
        if (a.size() != b.size() || a.empty()) {
            return std::numeric_limits<double>::infinity();
        }

        double sum = 0.0;
        for (size_t i = 0; i < a.size(); ++i) {
            sum += std::abs(a[i] - b[i]);
        }
        return sum / static_cast<double>(a.size());
    };

    std::vector<ModelEntry> modelEntries;
    modelEntries.reserve(objectModels.size());
    std::unordered_map<int, std::vector<size_t>> modelIndexesByPointNum;
    unsigned int maxTypeValue = 0;

    for (const auto& [type, cloud] : objectModels) {
        if (type == OT_UNKNOWN || !cloud || cloud->points_.size() < 3) {
            continue;
        }

        auto arrangedCloud = std::make_shared<PointCloud>(*cloud);

        ModelEntry entry;
        entry.type = type;
        entry.cloud = arrangedCloud;
        entry.pointNum = static_cast<int>(arrangedCloud->points_.size());
        entry.pairDistances = buildPairDistances(arrangedCloud->points_);
        entry.typeBit = 1u << static_cast<unsigned int>(type);

        modelIndexesByPointNum[entry.pointNum].push_back(modelEntries.size());
        modelEntries.push_back(std::move(entry));
        maxTypeValue = std::max(maxTypeValue, static_cast<unsigned int>(type));
    }

    if (modelEntries.empty()) {
        return bestObjects;
    }

    const size_t pointCount = validPointCloud->points_.size();
    if (pointCount > 31) {
        return bestObjects;
    }

    std::vector<std::vector<double>> distanceMatrix(
        pointCount,
        std::vector<double>(pointCount, 0.0));
    std::vector<uint32_t> adjacencyMasks(pointCount, 0);

    for (size_t i = 0; i < pointCount; ++i) {
        for (size_t j = i + 1; j < pointCount; ++j) {
            const double distance =
                (validPointCloud->points_[i] - validPointCloud->points_[j]).norm();
            distanceMatrix[i][j] = distance;
            distanceMatrix[j][i] = distance;

            const bool linked = distance >= minSideDist && distance <= maxSideDist;
            if (linked) {
                adjacencyMasks[i] |= (1u << static_cast<uint32_t>(j));
                adjacencyMasks[j] |= (1u << static_cast<uint32_t>(i));
            }
        }
    }

    auto buildPairDistancesFromIndices = [&](const VecInt& pointIndexes) {
        std::vector<double> distances;
        distances.reserve(pointIndexes.size() * (pointIndexes.size() - 1) / 2);
        for (size_t i = 0; i < pointIndexes.size(); ++i) {
            for (size_t j = i + 1; j < pointIndexes.size(); ++j) {
                distances.push_back(distanceMatrix[static_cast<size_t>(pointIndexes[i])]
                                                [static_cast<size_t>(pointIndexes[j])]);
            }
        }
        std::sort(distances.begin(), distances.end());
        return distances;
    };

    auto extractPointCloud = [](const PointCloudPtr& sourcePointCloud, const VecInt& pointIndexes) {
        auto extractedCloud = std::make_shared<PointCloud>();
        extractedCloud->points_.reserve(pointIndexes.size());

        const bool hasColors = sourcePointCloud->HasColors();
        const bool hasNormals = sourcePointCloud->HasNormals();
        if (hasColors) {
            extractedCloud->colors_.reserve(pointIndexes.size());
        }
        if (hasNormals) {
            extractedCloud->normals_.reserve(pointIndexes.size());
        }

        for (int index : pointIndexes) {
            extractedCloud->points_.push_back(sourcePointCloud->points_[static_cast<size_t>(index)]);
            if (hasColors && static_cast<size_t>(index) < sourcePointCloud->colors_.size()) {
                extractedCloud->colors_.push_back(sourcePointCloud->colors_[static_cast<size_t>(index)]);
            }
            if (hasNormals && static_cast<size_t>(index) < sourcePointCloud->normals_.size()) {
                extractedCloud->normals_.push_back(sourcePointCloud->normals_[static_cast<size_t>(index)]);
            }
        }

        return extractedCloud;
    };

    auto buildPointMask = [](const VecInt& indices) {
        uint32_t mask = 0;
        for (int index : indices) {
            mask |= (1u << static_cast<uint32_t>(index));
        }
        return mask;
    };

    auto isConnectedSubset = [&](const VecInt& indices, uint32_t pointMask) {
        if (indices.empty() || pointMask == 0) {
            return false;
        }

        uint32_t visitedMask = 0;
        uint32_t frontierMask = (1u << static_cast<uint32_t>(indices.front()));

        while (frontierMask != 0) {
            const uint32_t currentBit = frontierMask & (~frontierMask + 1u);
            frontierMask ^= currentBit;
            visitedMask |= currentBit;

            const int currentIndex = std::countr_zero(currentBit);
            const uint32_t neighbors = adjacencyMasks[static_cast<size_t>(currentIndex)] & pointMask;
            frontierMask |= (neighbors & ~visitedMask);
        }

        return visitedMask == pointMask;
    };

    auto hasMinimumCycleDegree = [&](const VecInt& indices, uint32_t pointMask) {
        if (indices.size() < 3 || pointMask == 0) {
            return false;
        }

        for (int index : indices) {
            const uint32_t neighborMask =
                adjacencyMasks[static_cast<size_t>(index)] & (pointMask & ~(1u << static_cast<uint32_t>(index)));
            const int neighborCount = std::popcount(neighborMask);
            if (neighborCount < 2) {
                return false;
            }
        }

        return true;
    };

    auto validateObjectCloud = [&](PointCloudPtr objectCloud) {
        if (!objectCloud || objectCloud->points_.size() < 3) {
            return false;
        }

        Plane fitPlane = FitPlaneFromPoints(objectCloud->points_);
        if (fitPlane.normal.squaredNorm() <= 1e-12) {
            return false;
        }

        for (const auto& point : objectCloud->points_) {
            if (PointToPlaneDistance(point, fitPlane) > threshold) {
                return false;
            }
        }

        ArrangePointCloud(objectCloud, nullptr, fitPlane.normal);
        fitPlane = FitPlaneFromPoints(objectCloud->points_);
        if (fitPlane.normal.squaredNorm() <= 1e-12) {
            return false;
        }

        std::vector<Point3D> projectedPoints =
            ProjectPointsToPlane(objectCloud->points_, fitPlane);
        if (projectedPoints.size() != objectCloud->points_.size()) {
            return false;
        }

        constexpr double kEps = 1e-8;
        double turnSign = 0.0;
        for (size_t i = 0; i < projectedPoints.size(); ++i) {
            const Point3D& p0 = projectedPoints[i];
            const Point3D& p1 = projectedPoints[(i + 1) % projectedPoints.size()];
            const Point3D& p2 = projectedPoints[(i + 2) % projectedPoints.size()];

            const double sideLength = (p1 - p0).norm();
            if (sideLength < minSideDist || sideLength > maxSideDist) {
                return false;
            }

            const double turn = (p1 - p0).cross(p2 - p1).dot(fitPlane.normal);
            if (std::abs(turn) <= kEps) {
                continue;
            }

            const double sign = turn > 0.0 ? 1.0 : -1.0;
            if (turnSign == 0.0) {
                turnSign = sign;
            }
            else if (turnSign != sign) {
                return false;
            }
        }

        return turnSign != 0.0;
    };

    struct Candidate {
        ObjectType type = OT_UNKNOWN;
        PointCloudPtr cloud;
        uint32_t pointMask = 0;
        uint32_t typeBit = 0;
        int matchedPoints = 0;
        double mae = std::numeric_limits<double>::infinity();
    };

    std::vector<Candidate> candidates;
    const double objectThreshold = threshold * 2.0;

    auto enumerateCombinations = [&](int chooseNum, const std::function<void(const VecInt&)>& onCombination) {
        if (chooseNum <= 0 || chooseNum > static_cast<int>(pointCount)) {
            return;
        }

        VecInt current;
        current.reserve(static_cast<size_t>(chooseNum));

        std::function<void(int, int)> dfs = [&](int start, int need) {
            if (need == 0) {
                onCombination(current);
                return;
            }

            for (int i = start; i <= static_cast<int>(pointCount) - need; ++i) {
                current.push_back(i);
                dfs(i + 1, need - 1);
                current.pop_back();
            }
        };

        dfs(0, chooseNum);
    };

    for (const auto& [candidatePointNum, modelIndexes] : modelIndexesByPointNum) {
        enumerateCombinations(candidatePointNum, [&](const VecInt& combination) {
            const uint32_t pointMask = buildPointMask(combination);
            if (!hasMinimumCycleDegree(combination, pointMask) ||
                !isConnectedSubset(combination, pointMask)) {
                return;
            }

            std::vector<size_t> matchedModelIndexes;
            matchedModelIndexes.reserve(modelIndexes.size());
            const std::vector<double> candidateSignature =
                buildPairDistancesFromIndices(combination);

            for (size_t modelIndex : modelIndexes) {
                const auto& model = modelEntries[modelIndex];
                const double signatureError =
                    calcSignatureError(candidateSignature, model.pairDistances);
                if (signatureError <= threshold) {
                    matchedModelIndexes.push_back(modelIndex);
                }
            }

            if (matchedModelIndexes.empty()) {
                return;
            }

            auto candidateCloud = extractPointCloud(validPointCloud, combination);
            if (!validateObjectCloud(candidateCloud)) {
                return;
            }

            const auto& orderings = getAllOrderings(candidateCloud->points_.size());
            std::vector<PointCloud> orderedCandidateClouds;
            orderedCandidateClouds.reserve(orderings.size());
            for (const auto& ordered : orderings) {
                orderedCandidateClouds.push_back(reorderPointCloud(*candidateCloud, ordered));
            }

            ObjectType bestType = OT_UNKNOWN;
            uint32_t bestTypeBit = 0;
            double bestMae = std::numeric_limits<double>::infinity();
            int bestOrderedIndex = -1;

            for (size_t modelIndex : matchedModelIndexes) {
                const auto& model = modelEntries[modelIndex];

                for (size_t orderedIndex = 0; orderedIndex < orderedCandidateClouds.size(); ++orderedIndex) {
                    double mae = std::numeric_limits<double>::infinity();
                    ComputeTransformationFromCorrespondences(
                        orderedCandidateClouds[orderedIndex],
                        *model.cloud,
                        &mae);

                    if (mae < bestMae) {
                        bestMae = mae;
                        bestType = model.type;
                        bestTypeBit = model.typeBit;
                        bestOrderedIndex = static_cast<int>(orderedIndex);
                    }
                }
            }

            if (bestType == OT_UNKNOWN || bestMae > objectThreshold) {
                return;
            }

            if (bestOrderedIndex >= 0) {
                candidateCloud = std::make_shared<PointCloud>(
                    orderedCandidateClouds[static_cast<size_t>(bestOrderedIndex)]);
            }

            candidates.push_back({
                bestType,
                candidateCloud,
                pointMask,
                bestTypeBit,
                static_cast<int>(combination.size()),
                bestMae });
        });
    }

    if (candidates.empty()) {
        return bestObjects;
    }

    const uint32_t pointStateCount = 1u << static_cast<uint32_t>(pointCount);
    const uint32_t typeStateCount = 1u << (maxTypeValue + 1u);

    struct DpState {
        bool valid = false;
        int matchedPoints = -1;
        int objectCount = -1;
        double mae = std::numeric_limits<double>::infinity();
        int prevStateIndex = -1;
        int candidateIndex = -1;
    };

    auto isBetterState = [](int matchedPoints, int objectCount, double mae, const DpState& current) {
        if (!current.valid) {
            return true;
        }
        if (matchedPoints != current.matchedPoints) {
            return matchedPoints > current.matchedPoints;
        }
        if (objectCount != current.objectCount) {
            return objectCount > current.objectCount;
        }
        return mae < current.mae;
    };

    auto stateIndex = [pointStateCount](uint32_t typeMask, uint32_t pointMask) {
        return static_cast<int>(typeMask * pointStateCount + pointMask);
    };

    std::vector<DpState> dp(static_cast<size_t>(typeStateCount) * pointStateCount);
    dp[0] = { true, 0, 0, 0.0, -1, -1 };

    for (int candidateIndex = 0; candidateIndex < static_cast<int>(candidates.size()); ++candidateIndex) {
        const auto& candidate = candidates[static_cast<size_t>(candidateIndex)];
        for (int typeMask = static_cast<int>(typeStateCount) - 1; typeMask >= 0; --typeMask) {
            if ((static_cast<uint32_t>(typeMask) & candidate.typeBit) != 0) {
                continue;
            }

            for (int pointMask = static_cast<int>(pointStateCount) - 1; pointMask >= 0; --pointMask) {
                const int currentIndex = stateIndex(static_cast<uint32_t>(typeMask), static_cast<uint32_t>(pointMask));
                const auto& current = dp[static_cast<size_t>(currentIndex)];
                if (!current.valid) {
                    continue;
                }
                if ((static_cast<uint32_t>(pointMask) & candidate.pointMask) != 0) {
                    continue;
                }

                const uint32_t nextTypeMask = static_cast<uint32_t>(typeMask) | candidate.typeBit;
                const uint32_t nextPointMask = static_cast<uint32_t>(pointMask) | candidate.pointMask;
                const int nextIndex = stateIndex(nextTypeMask, nextPointMask);

                const int nextMatchedPoints = current.matchedPoints + candidate.matchedPoints;
                const int nextObjectCount = current.objectCount + 1;
                const double nextMae = current.mae + candidate.mae;

                if (isBetterState(nextMatchedPoints, nextObjectCount, nextMae, dp[static_cast<size_t>(nextIndex)])) {
                    dp[static_cast<size_t>(nextIndex)] = {
                        true,
                        nextMatchedPoints,
                        nextObjectCount,
                        nextMae,
                        currentIndex,
                        candidateIndex };
                }
            }
        }
    }

    int bestStateIndex = -1;
    for (int i = 0; i < static_cast<int>(dp.size()); ++i) {
        const auto& state = dp[static_cast<size_t>(i)];
        if (!state.valid || state.objectCount <= 0) {
            continue;
        }
        if (bestStateIndex < 0 ||
            isBetterState(state.matchedPoints, state.objectCount, state.mae, dp[static_cast<size_t>(bestStateIndex)])) {
            bestStateIndex = i;
        }
    }

    while (bestStateIndex >= 0) {
        const auto& state = dp[static_cast<size_t>(bestStateIndex)];
        if (state.candidateIndex < 0) {
            break;
        }

        auto& candidate = candidates[static_cast<size_t>(state.candidateIndex)];
        bestObjects[candidate.type] = candidate.cloud;
        bestStateIndex = state.prevStateIndex;
    }

    return bestObjects;
}

CAM_API BASE_CHECK_RESULT CheckBaseValid(
    const PointCloudPtr& pointCloud, 
    const OBJECT_UNMAP& objectModels, 
    int minSideDiff, 
    double minSideDist, 
    double maxSideDist)
{
    if (!pointCloud) return BCR_POINT_NUM_ERROR;
    const size_t n = pointCloud->points_.size();
    if (n < 3) return BCR_POINT_NUM_ERROR;

    // 计算多边形边长（相邻点距离），pointCloud 已按顺时针排列
    std::vector<double> dists;
    dists.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n; // 与下一个点（最后一个点与第一个点相连）
        const double dist = (pointCloud->points_[i] - pointCloud->points_[j]).norm();
        dists.push_back(dist);
    }
    if (dists.empty()) return BCR_POINT_NUM_ERROR;
    std::sort(dists.begin(), dists.end());

    const double minDist = dists.front();
    const double maxDist = dists.back();

    if (minDist < minSideDist) return BCR_SIDE_TOO_SHORT;
    if (maxDist > maxSideDist) return BCR_SIDE_TOO_LONG;

    if (dists.size() >= 2) {
        const double second = dists[1];
        if ((second - minDist) < static_cast<double>(minSideDiff)) {
            return BCR_MINSIDE_INVALID;
        }
    }

    OBJECT_UNMAP allModels =  objectModels;
    allModels[OT_BASE] = pointCloud;

    // 使用 CalculateObject3 进行识别，要求识别出 OT_BASE 且点数量不变
    auto result = CalculateObject3(pointCloud, allModels, /*threshold=*/50.0, minSideDist, maxSideDist);
    auto it = result.find(OT_BASE);
    if (it == result.end()) {
        return BCR_POINT_NUM_ERROR;
    }
    const PointCloudPtr& detected = it->second;
    if (!detected) return BCR_POINT_NUM_ERROR;
    if (detected->points_.size() != pointCloud->points_.size()) return BCR_POINT_NUM_ERROR;

    return BCR_SUCESS;
}
