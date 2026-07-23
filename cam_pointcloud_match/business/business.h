#pragma once
#pragma once

#include "type.h"

#include <vtkSmartPointer.h>
class vtkPolyData;

/// <summary>
/// 按点对计算 Mesh 到相机点云的刚体变换矩阵,源和目标点的索引按顺序一一对应
/// </summary>
/// <param name="sourceIndexes">源点云点索引</param>
/// <param name="targetIndexes">目标点云点索引</param>
/// <param name="source">源点云</param>
/// <param name="target">目标点云</param>
/// <returns></returns>
CAM_API Eigen::Matrix4d RegisterPointCloudFromPoints(
	const std::vector<int>& sourceIndexes,
	const std::vector<int>& targetIndexes,
	const PointCloud& source,
	const PointCloud& target,
	FineAlignmentResult* result = nullptr);

CAM_API PolyDataPtr Open3DMeshToVtkPolyData(const TriangleMeshPtr& mesh);

CAM_API PolyDataPtr Open3DPointCloudToVtkPolyData(const PointCloud& cloud);

CAM_API ActorPtr VtkPolyDataToActor(const PolyDataPtr& poly);

CAM_API ActorPtr MeshToActor(const TriangleMeshPtr& mesh);

CAM_API ActorPtr PointCloudToActor(const PointCloudPtr& pointCloud);

CAM_API PointCloudPtr ActorToPointCloud(const ActorPtr& actor);

CAM_API void ShowActor(
	const RenderPtr& render, 
	const ActorPtr& actor, 
	bool resetCamRange = true);

// 计算沿方向 vec 在点云 cloud 上投影值最大的点（即该方向的最远点）
CAM_API Point3D MaxPointFromVec(const PointCloudPtr& cloud, const Eigen::Vector3d& vec);

/// <summary>
/// 采样几组source和target，计算出他们的平均值，然后再计算配准
/// 矩阵，对meshes和pointClouds进行变换
/// </summary>
struct CalibrationInfo {
	PointCloud sourceFeatures;
	PointCloud targetFeatures;
	Eigen::Matrix4d T;
};
CAM_API CalibrationInfo CalibrateMeshFromReference(
    const std::vector<PointCloudPtr>& sources,
    const std::vector<PointCloudPtr>& targets,
    std::vector<TriangleMeshPtr>& meshes,
    std::vector<PointCloudPtr>* pointClouds = nullptr);

/// <summary>
/// 找到pointCloud中的第一个距离下一个顶点距离最小的顶点，然后重新调整pointCloud
/// 中的顶点顺序,使得这个顶点成为pointCloud中的第一个顶点，并且原来pointCloud中
/// 后续的顶点顺序保持不变，然后这个顶点原来在pointCloud中前面的索引位置上的顶点
/// 保持顺序不变，追加到pointCloud的末尾
/// </summary>
/// <param name="pointCloud"></param>
/// <returns></returns>
CAM_API void MakesSmallestFirstPoint(PointCloud& pointCloud);

/// <summary>
/// 使点云第一个点的颜色为startColor，最后一个点的颜色为endColor，点云中其他点的颜色
/// 根据索引的距离，在startColor和endColor之间进行线性插值计算出来，形成一个渐变色的
/// 效果
/// </summary>
/// <param name="pointCloud"></param>
/// <param name="startColor"></param>
/// <param name="endColor"></param>
/// <returns></returns>
CAM_API void MakaGradient(
	PointCloudPtr pointCloud, 
	Eigen::Vector3d startColor,
	Eigen::Vector3d endColor);

enum ObjectType {
	OT_UNKNOWN = -1,	//未知类型
	OT_BASE,			//基座
	OT_PROBE,			//探针
	OT_GUIDE,			//引导器
	OT_MEASURE_PROBE,   //测量器探针
	OT_MEASURE_COORD	//测量器坐标
};

struct OBJECT_VEC_ENTRY {
	std::vector<int> indices;
	PointCloudPtr cloud;
};
using OBJECT_VEC = std::unordered_map<ObjectType, std::vector<OBJECT_VEC_ENTRY>>;

/// <summary>
/// 物体模型信息
/// </summary>
struct OBJECT_MODEL_INFO {
	ObjectType type;			//物体类型
	PointCloudPtr cloud;		//物体模型的点云
};;
using OBJECT_MODEL_VEC = std::vector<OBJECT_MODEL_INFO>;
/// <summary>
/// pointCloud是1到3个物体的发光点捕获的点云，每个物体最多5个顶点且近似共面，因此点
/// 云中顶点数量最多为15个，根据objectModels中指定物体模型的信息，对pointCloud中的顶
/// 点进行分类,返回一个 OBJECT_VEC 结构
/// 具体的实现算法如下：
/// ·先清除掉pointCloud中离群点的索引，离群点的定义是：点云中某个点距离它最近的点的距离
///	  超过maxSideDist，视为离群点；
/// ·将近似共面的顶点进行切割PlaneCut
/// ·遍历每个切割法，进行物体模型匹配，算法如下：
/// ··代码片段如下：
///		PLANE_CUT_VEC cuts;
///		//使用函数PlaneCut对pointCloud进行切割，得到cuts
///		//对objectModels的点云用ArrangePointCloud进行排序
///		for (auto itCut = cuts.begin(); itCut != cuts.end(); ++itCut) {	
///			std::vector<PLANE_SHAPE_CUT_VEC> planeShapeCutVec;	//每个元素对应itCut每个平面的形状切割法
///			planeShapeCutVec.resize(itCut->size());
///			for(size_t i = 0; i < itCut->size(); ++i){
///				auto itPlane = itCut[i];
///				//对itPlane中的点云用函数PlaneShapeCut进行形状切割
///				//填充planeShapeCutVec[i];
///			}
///			//该“平面切割法”中，"形状的切割法"数量=planeShapeCutVec每个元素的形状切割法数量的累乘
///			//"形状的切割法"类型SHAPE_CUT_VEC的定义:
///			//·最外层vector表示是不同的形状切割法
///			//·最内层vector表示该切割法中每个形状的顶点索引(pointCloud中索引)
///			using SHAPE_CUT_VEC = std::vector<std::vector<VecInt>>;	
///			SHAPE_CUT_VEC shapeCutVec;
///			int shapeCutNum = 1;
///			for(size_t i = 0; i < planeShapeCutVec.size(); ++i){
///				shapeCutNum *= planeShapeCutVec[i].size();	
///			}
///			shapeCutVec.resize(shapeCutNum);
///			//·通过planeShapeCutVec填充shapeCutVec；
///			//·遍历shapeCutVec，根据每个形状的索引，从pointCloud获取点云objectPC
///			//·对objectPC的点用函数ArrangePointCloud进行排序
///			//·对objectPC和objectModels中每个物体模型的点云使用ComputeTransformationFromCorrespondences
///				(必须相同点数)进行对齐，计算MAE
///			//	找到MAE最小的物体模型，设objecThreshold=threshold * 2，如果minMAE > objecThreshold,
///			//	则这个shap不合法	
///			//·计算shapeCutVecIt得分shapeCutVecItScore:匹配一个物体得objecThreshold分,减去该物体模型
///			//	和objectPC的MAE
///			//·遍历shapeCutVec，找到得分最高的shapeCutVecIt，设为该次“平面切割法”中最佳形状切割法
///		}
///		//最终求出所有“平面切割法”中最佳形状切割法中得分最高的shapeCutVecIt，设为最终的物体分类；
/// </summary>
/// <param name="pointCloud">实景中物体的发光点捕获的点云</param>
/// <param name="objectModels">硬编码的物体模型顶点组成的点云，必须是凸多边形</param>
/// <param name="threshold">临近阈值，用于计算下面的临近状态

/// </param>
/// <param name="maxSideDist">同一个物体的最大边长距离阈值，默认 200 毫米</param>
/// <param name="minSideDist">同一个物体的最小边长距离阈值，默认 10 毫米</param>
/// <returns>返回pointCloud中最接近objectModels中指定的物体模型的物体分类</returns>
CAM_API OBJECT_VEC CalculateObject(
	const PointCloudPtr& pointCloud,
	const OBJECT_MODEL_VEC& objectModels,
	double threshold = 50.0,
	double minSideDist = 10,
	double maxSideDist = 200);

/// <summary>
/// 从pointCloud中，按照objectModels分类出物体，为了解决CalculateObject效率低下的问题，objectModels采用unordered_map存储，确保每个物体类型的model只会
/// 有一种点云，返回的类型也为unordered_map存储，确保每个物体类型只会有一个点云
/// 算法如下：
/// ·先清除掉pointCloud中离群点，离群点的定义是：点云中某个点距离它最近的点的距离超过maxSideDist，视为离群点
/// ·用排列组合的方式，按照objectModels中每个物体模型的点数，从pointCloud中选取相同数量的点进行组合，
///   排列组合公式中A或者C后的括号中，第一个数字表示排列组合的上标数字，第二个表示下标数字
///   算法如下：
///   >因为pointCloud中的顶点数可能小于objectModels所有物体类型顶点总和，所以先组合物体类型selectObjectModel,
///    必须满足的条件:每个组合的所有物体类型顶点数量总和小于等于pointCloud中的顶点数，
///    selectObjectModel每个组合中每个物体类型的数量可能不一样，
///   >选择selectObjectModel中一个物体类型的组合selectObjectModelIt
///    从pointCloud中选取相同数量的点进行组合，
///    例如：
///    情形1：selectObjectModelIt中有3种物体类型,每个物体类型的点数依次为：3、4、5，pointCloud有12个点，因此公式为C(3,12) * C(4, 9) * C(5, 5)
///    情形2: selectObjectModelIt中有3种物体类型,每个物体类型的点数依次为：4、4、4，pointCloud有12个点，因此公式为C(4,8) * C(4,4)* C(4,4)
///    情形3: selectObjectModelIt中有2种物体类型,每个物体类型的点数依次为：4、4，pointCloud有8个点，因此公式为C(4,8) * C(4,4)
///    情形4: selectObjectModelIt中有1种物体类型,物体类型的点数为：4，pointCloud有4个点，因此公式为C(4,4)
///    情形5: selectObjectModelIt中有2种物体类型,每个物体类型的点数依次为：4、4，pointCloud有9个点，因此公式为C(4,9) * C(4,5)
///    因此，假如selectObjectModelIt挑选的物体类型的数量为n，物体类型的点数以此为P1、P2、......Pn-1、Pn,(Pn-1是个变量，不是表达式)，
///    则物体类型的顶点数总和为PointNum = P1+P2、......、Pn-1+Pn,
///    每个物体类型组合的排列组合的公式为：
///      COMBI=C(P1, PointNum) * C(P2, PointNum - P1) * ...  C(Pn-1, PointNum - P1 - P2... - Pn-2) * C(Pn, PointNum - P1 - P2... - Pn-1)  
///    总的排列组合公式为：selectObjectModel中所有COMBI的排列组合的并集  
/// ·对objectModels中每个物体类型的模型的点云各自调用函数ArrangePointCloud进行排序
/// ·对排列组合形成的每个物体类型的顶点组合各自调用函数ArrangePointCloud进行排序
/// ·排序之后，排列组合形成的每个物体类型的顶点，必须满足以下条件，否则排除该排列组合:
///   >每个物体类型的顶点各自拟合一个平面，物体类型的每个顶点距离自己拟合的平面小于阈值threshold
///   >每个物体类型的顶点各自拟合一个平面，每个物体类型的顶点投影到各自的拟合平面，每个物体类型投影的点各自组成的多边形必须是凸多边形
///   >每个物体类型的顶点区域不能互相穿插，互相穿插可以这么计算：把每个物体类型的顶点投影到各自拟合平面，投影后的顶点所在凸多边形区域不能在空间中
///    出现穿插
///   >每个物体类型的顶点组合形成的多边形的边长满足大于等于minSideDist，小于等于maxSideDist
///  ·计算排列组合中最优的顶点组合，最优算法如下：
///    >设某个物体类型的Model点云为modelPT，某个排列组合为该物体类型挑选的顶点组合的为objectPT，
///     则调用ComputeTransformationFromCorrespondences(objectPT, modelPT, MAE),
///    >如果选择了多个物体类型来组合顶点，则每个物体类型的mae的总和为sumMAE
///    >计算sumMAE最小的排列组合作为最优顶点组合
/// </summary>
/// <param name="pointCloud">点云</param>
/// <param name="objectModels">每个物体类型的DEMO点云</param>
/// <param name="threshold">阈值</param>
/// <param name="minSideDist">最小边长距离阈值，默认 10 毫米</param>
/// <param name="maxSideDist">最大边长距离阈值，默认 200 毫米</param>
/// <returns></returns>
using OBJECT_UNMAP = std::unordered_map<ObjectType, PointCloudPtr>;
CAM_API OBJECT_UNMAP CalculateObject2(
	const PointCloudPtr& pointCloud,
	const OBJECT_UNMAP& objectModels,
	double threshold = 50.0,
	double minSideDist = 10,
	double maxSideDist = 200);

CAM_API OBJECT_UNMAP CalculateObject3(
	const PointCloudPtr& pointCloud,
	const OBJECT_UNMAP& objectModels,
	double threshold = 50.0,
	double minSideDist = 10,
	double maxSideDist = 200);

/// <summary>
/// 基座是否合法
/// 合法算法：
/// ·最小边长大于minSideDist
/// ·最大边长小于maxSideDist
/// ·基座最短的边，比第二短的边要小至少10个单位
/// ·带入函数CalculateObject3(objectModels带入这个函数)后,能被正确的识别为类型OT_BASE,且识别后点的数量没有变化
/// </summary>
/// <param name="pointCloud">基座的点云</param>
/// <returns>如果基座合法，返回true；否则返回false</returns>
enum BASE_CHECK_RESULT {
	BCR_SUCESS,
	BCR_SIDE_TOO_SHORT,			//最小边长小于minSideDist
	BCR_SIDE_TOO_LONG,			//最大边长大于maxSideDist
	BCR_MINSIDE_INVALID,		//最小边长与第二短边长的差小于minSideDiff
	BCR_POINT_NUM_ERROR			//点的数量错误
};
CAM_API BASE_CHECK_RESULT CheckBaseValid(
	const PointCloudPtr& pointCloud,
	const OBJECT_UNMAP& objectModels,
	int minSideDiff = 10,
	double minSideDist = 10,
	double maxSideDist = 200);