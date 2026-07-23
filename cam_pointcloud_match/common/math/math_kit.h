#pragma once

#include "type.h"

struct LineSeg3D {
	Point3D start;		// 线段的起点
	Point3D end;		// 线段的终点
};

struct Line {
	Point3D point;		// 线上的一个点
	Point3D dir;		// 线的方向向量 (不需要单位化)
};

/// <summary>
/// 根据点集合拟合一条直线
/// </summary>
/// <param name="points"></param>
/// <returns></returns>
Line FitLineFromPoints(const std::vector<Point3D>& points);

 /// <summary>
 //  PCA 计算结果
 // eigenvalues: 特征值，按升序排列（eigenvalues[0] 最小，eigenvalues[2] 最大）
 // eigenvectors : 对应的特征向量矩阵，每一列为一个特征向量，列索引与 eigenvalues 对应
 // centroid : 点集的质心
 /// </summary>
struct PCAResult {
	Eigen::Vector3d eigenvalues;
	Eigen::Matrix3d eigenvectors;
	Point3D centroid;
};

/// <summary>
/// 对点集合执行 PCA，返回特征值、特征向量和质心
/// </summary>
/// <param name="points">点集合（Point3D 为 Eigen::Vector3d 的别名）</param>
/// <returns>PCAResult（特征值按升序排列）</returns>
PCAResult CalPCA(const std::vector<Point3D>& points);

/// <summary>
/// 平面表示：point 在平面上
/// normal 为平面法向量
/// </summary>
struct Plane {
	Point3D point;
	Point3D normal;
};

// 三点直接确定平面，避免在候选平面枚举时反复走 PCA。
bool BuildPlaneFromTriangle(
	const Point3D& p0,
	const Point3D& p1,
	const Point3D& p2,
	Plane* plane);

/// <summary>
/// 根据点集合拟合平面（使用 PCA），返回一个点和平面法向量
/// </summary>
/// <param name="points">点集合（Point3D 为 Eigen::Vector3d 的别名）</param>
/// <returns>拟合的平面</returns>
/// </summary>
/// <param name="points"></param>
/// <returns></returns>
Plane FitPlaneFromPoints(const std::vector<Point3D>& points);

/// <summary>
/// 计算点集合在平面上的投影点的集合
/// </summary>
/// <param name="points">点集合</param>
/// <param name="plane">投影平面</param>
/// <returns>投影后的点集合</returns>
std::vector<Point3D> ProjectPointsToPlane(
	const std::vector<Point3D>& points,
	const Plane& plane);

/// <summary>
/// 计算由点集构成的线的宽度。计算PCA，然后找到第二大特征值对应的特征向量,
/// 找出点集合在该向量上的相距最远的两个点，返回它们之间的距离作为线的宽度。
/// </summary>
/// <param name="points">组成线的三维点的向量。</param>
/// <returns>线的最大宽度值。</returns>
double LineMaxWidth(const std::vector<Point3D>& points);

/// <summary>
/// 查看点集合是否看起来想一条直线：计算直线的宽度，如果宽度小于给定的阈值，
/// 则认为这些点近似落在一条线上。
/// </summary>
/// <param name="points"></param>
/// <param name="threshold"></param>
/// <returns></returns>
bool IsLinkLine(const std::vector<Point3D>& points, double threshold = 3.0);

/// <summary>
///	排列cloud的顶点顺序
/// 算法步骤：
/// ·计算cloud的拟合平面fitPlane，得到平面法向量n和平面上的一个点C(点云的质心)
/// ·使用refAxis统一法向量方向，保证n·refAxis > 0；若cloud所在平面与refAxis平行
///   (即n·refAxis近似等于0)，则不做排序，保持原顺序
/// ·沿着平面法向量n看过去，使点云顺时针旋转(满足右手法则)
/// ·让排列后的第一个顶点顺时针方向的边是最短的边，如果有两个边长相同，且都是最短的，
///   那么让找到的第一个边长最短的边作为起始边
/// ·排序的过程中，如果二个点的角度一样，则距离点C(近的排在前面，如果距离中心点
///   也一样，则按原始索引排序
/// </summary>
/// <param name="cloud">点云</param>
/// <param name="newIndexes">排序之后，原来的点的索引在新的点云的索引中的映射</param>
/// <param name="refAxis">参考轴，用于统一拟合平面法向量方向</param>
/// <returns></returns>
CAM_API void ArrangePointCloud(
	PointCloudPtr& cloud,
	std::vector<int>* newIndexes = nullptr, 
	const Eigen::Vector3d& refAxis = Eigen::Vector3d::UnitZ());

/// <summary>
/// 计算：点p到由line表示的直线的距离
/// </summary>
/// <param name="p"></param>
/// <param name="line"></param>
/// <returns></returns>
CAM_API double PointToLineDistance(const Point3D& p, const Line& line);

/// <summary>
/// 判断点p是否在由line表示的直线上，允许一定的距离误差threshold
/// </summary>
/// <param name="p"></param>
/// <param name="line"></param>
/// <param name="threshold"></param>
/// <returns></returns>
CAM_API bool PointIsOnLine(const Point3D& p, const Line& line, double threshold = 3.0);

/// <summary>
/// 计算点p到由seg表示的线段的距离
/// ·如果点p投影到seg所在的直线上时，投影点在线段范围内，则返回点p到投影点的距离
/// ·如果点p投影到seg所在的直线上时，投影点不在线段范围内，否则返回点p到线段两个
///	  端点的距离中的较小值。
/// </summary>
/// <param name="p"></param>
/// <param name="seg"></param>
/// <returns></returns>
CAM_API double PointToLineSegmentDistance(const Point3D& p, const LineSeg3D& seg);

/// <summary>
/// 判断点p是否在由line表示的线段上，允许一定的距离误差threshold
/// </summary>
/// <param name="p"></param>
/// <param name="line"></param>
/// <param name="threshold"></param>
/// <returns></returns>
CAM_API bool PointIsOnLineSeg(
	const Point3D& p, 
	const LineSeg3D& line, 
	double threshold = 3.0);

/// <summary>
/// 计算两条直线的距离
/// </summary>
/// <param name="line1"></param>
/// <param name="line2"></param>
/// <returns></returns>
CAM_API double LineToLineDistance(const Line& line1, const Line& line2);

/// <summary>
/// 两条直线是否相交，允许一定的距离误差threshold
/// </summary>
/// <param name="line1"></param>
/// <param name="line2"></param>
/// <param name="threshold"></param>
/// <returns></returns>
CAM_API bool LinesIsIntersected(
	const Line& line1, 
	const Line& line2, 
	double threshold = 3.0);

/// <summary>
/// 两条线段的最近距离
/// </summary>
/// <param name="lineSeg1"></param>
/// <param name="lineSeg2"></param>
/// <returns></returns>
CAM_API double LineSegToLineSegDistance(const LineSeg3D& lineSeg1, const LineSeg3D& lineSeg2);

/// <summary>
/// 两条线段是否相交，允许一定的距离误差threshold
/// </summary>
/// <param name="lineSeg1"></param>
/// <param name="lineSeg2"></param>
/// <param name="threshold"></param>
/// <returns></returns>
CAM_API bool LineSegsIsIntersected(
	const LineSeg3D& lineSeg1,
	const LineSeg3D& lineSeg2,
	double threshold = 3.0);

/// <summary>
/// 点到平面的距离
/// </summary>
/// <param name="p"></param>
/// <param name="plane"></param>
/// <returns></returns>
CAM_API double PointToPlaneDistance(
	const Point3D& p,
	const Plane& plane);

/// <summary>
/// 点是否在平面上，允许一定的距离误差threshold
/// </summary>
/// <param name="p"></param>
/// <param name="plane"></param>
/// <param name="threshold"></param>
/// <returns></returns>
CAM_API bool PointIsOnPlane(
	const Point3D& p,
	const Plane& plane,
	double threshold = 3.0);

/// <summary>
/// 计算多边形poly1的区域(包括点和边)和多边形poly2的区域(包括点和边)之间的
/// 最小距离。
/// poly1的顶点不一定在同一个平面，但是外界可以保证，它的顶点在
/// 自己拟合平面上的投影的多边形是一个凸多边形，poly2同样是这样的情形。
/// poly1和poly2之间的最小距离,是基于他们各自在自己的拟合平面上的投影的多边
/// 形区域之间的最小距离来计算的
///	距离的计算算法如下:
/// ·计算poly1和poly2的拟合平面fitPlane1和fitPlane2
/// ·计算poly1在fitPlane1上的投影多边形的projPoly1
/// ·计算poly2在fitPlane2上的投影多边形的projPoly2
/// ·如果projPoly1和projPoly2是分离的，那么距离为projPoly1和projPoly2区域之
///	  间的最小距离
/// ·projPoly1和projPoly2之间如果出现穿插，那么距离为0
/// ·如果projPoly1中的点、边、部分区域或者全部区域，在projPoly2区域之中，那
///	  么距离为0
/// </summary>
/// <param name="poly1"></param>
/// <param name="poly2"></param>
/// <returns></returns>
CAM_API double PolygonToPolygonDistance(
	const std::vector<Point3D>& poly1,
	const std::vector<Point3D>& poly2);

/// <summary>
/// 两个多边形是否相交，允许一定的距离误差threshold
/// </summary>
/// <param name="poly1"></param>
/// <param name="poly2"></param>
/// <param name="threshold"></param>
/// <returns></returns>
CAM_API bool PolygonsIsIntersected(
	const std::vector<Point3D>& poly1,
	const std::vector<Point3D>& poly2,
	double threshold = 3.0);

/// <summary>
/// 点到多边形的距离：点p到多边形poly的距离是点p到poly区域（包括点和边）的最小距离，
/// 这个poly的顶点不一定在同一个平面，但是外界可以保证，它的顶点在自己拟合平面上的
/// 投影的多边形是一个凸多边形，所以可以通过投影后的二维凸多边形来计算距离。
/// </summary>
/// <param name="p"></param>
/// <param name="poly"></param>
/// <returns></returns>
CAM_API double PointToPolygonDistance(
	const Point3D& p,
	const std::vector<Point3D>& poly);

/// <summary>
/// 点是否在多边形上：点p是否在多边形poly的区域（包括点和边）上，允许一定的距离误差
/// threshold，
/// </summary>
/// <param name="p"></param>
/// <param name="poly"></param>
/// <param name="threshold"></param>
/// <returns></returns>
CAM_API bool PointIsOnPolygon(
	const Point3D& p,
	const std::vector<Point3D>& poly,
	double threshold = 3.0);

/// <summary>
/// 删除离群点，离群点的定义是：点云中某个点距离它最近的点的距离超过maxSideDist，视为离群点
/// </summary>
/// <param name=""></param>
/// <returns></returns>
CAM_API void DeleteOuterPoints(PointCloudPtr pointCloud, double maxSideDist);

//“平面切割法”类型的定义:
//·最外层vector表示是不同平面切割法
//·最内层vector表示该切割法中每个平面内部点云索引数组(pointCloud中索引)
using PLANE_CUT_VEC = std::vector<std::vector<VecInt>>;
/// <summary>
/// 对点云pointCloud中近似共面的顶点进行切割,顶点近似共面指的是几个顶点形成平面时，所有的
/// 点距离他们拟合的平面的距离<=threshold
/// 每个切割法中必须满足以下条件：
/// ·参数minVertexNum为每个平面中至少拥有的顶点数量；
/// ·每种切割法中，一个顶点只能被一个平面拥有；
/// ·对每个切割法中的每个平面建立凸包围多边形，这个凸多边形包围该平面所有顶点，这个凸
///	  凸多边形的顶点是基于该平面中的顶点建立起来的，刚好包围该平面所有顶点，一个平面中所有
///	  顶点的凸包围多边形，不能和该切割法中的其他平面中的顶点的凸包围多边形出现临近状态:
/// 临近状态(相对于同一个切割法中的平面集合来说的):
/// ··两个平面的凸包围多边形近似碰撞,两个多边形内部区域(包括顶点和边)的最近距离小于
///	    threshold，都可以视为近似碰撞。
///		包括以下几种情况:
///		1、两个多边形出现平面上或者空间中的交叉
///		2、一个多边形近似的包含另外一个多边形的顶点、边、或者全部的顶点
/// ·对每个切割法中平面内的顶点寻找其符合规则的近邻点(距离<=maxSideDist且>=minSideDist)的
///   minVertexNum-1个顶点，如果没有任何顶点能寻找到minVertexNum-1个顶点，则这个切割法不合
///	  法，丢弃掉这个切割法；
/// ·如果一个切割法中平面的数量>=maxPlaneNum，则这个切割法不合法，丢弃掉这个切割法；
/// </summary>
/// <param name="pointCloud"></param>
/// <param name="minVertexNum">每个平面中至少拥有的顶点数量</param>
/// <param name="maxPlaneNum">每个切割法中最多拥有的平面数量</param>
/// <param name="minSideDist">同一个平面中顶点的最小边长距离阈值，默认10毫米</param>
/// <param name="maxSideDist">同一个平面中顶点的最大边长距离阈值，默认200毫米</param>
/// <param name="threshold">临近阈值，用于计算临近状态(参考函数中threshold参数的临近状态说明)</param>
/// <param name="unclusteredIndices">无法参与切割的顶点索引</param>
/// <returns></returns>
CAM_API PLANE_CUT_VEC PlaneCut(
	const PointCloudPtr& pointCloud,
	int minVertexNum,
	int maxPlaneNum,
	double minSideDist = 10,
	double maxSideDist = 200,
	double threshold = 15.0,
	std::vector<int>* unclusteredIndices = nullptr);

//“平面多边形切割法”类型PLANE_SHAPE_CUT_VEC的定义:
//·最外层vector的it表示一个形状切割法
//·最内层vector的it表示该切割法中每个形状的顶点索引(pointCloud中索引),因为一个平面中
//	可能有多个形状，所以是一个vector；
using PLANE_SHAPE_CUT_VEC = std::vector<std::vector<VecInt>>;
/// <summary>
/// 对近似共面的点云进行多边形切割,近似共面指的是几个顶点形成平面时，所有的点距离他们拟合的平
/// 面的距离<=threshold
/// 算法如下：
/// ·拟合pointCloud的平面fitPlane
/// ·然后基于pointCloud进行多边形切割，切割的多边形的规则如下：
/// ··点的数量必须大于等于minVertexNum，小于等于maxVertexNum
/// ··边长side必须满足side<=maxSideDist和side>=minSideDist的要求
/// ··多边形在fitPlane的投影是凸多边形，如果同一个切割法中pointCloud
///		可以切割多个多边形时，则这些多边形之间不能是临近状态
/// ··临近状态(相对于同一个切割法中多个多边形来说的):
///   1、两个多边形的区域之间的距离小于threshold，都可以视为临近。
///	  2、两个多边形出现交叉
///	  3、一个多边形近似的包含另外一个多边形的顶点、边、或者全部的顶点
/// ··多边形的点云需要按照ArrangePointCloud进行排序
/// </summary>
/// <param name="pointCloud">近似共面的点云(外部保证是近似共面)</param>
/// <param name="minVertexNum">每个形状中至少拥有的顶点数量</param>
/// <param name="maxVertexNum">每个形状中最多拥有的顶点数量</param>
/// <param name="maxSideDist">每个形状中顶点的最大边长距离阈值</param>
/// <param name="minSideDist">每个形状中顶点的最小边长距离阈值</param>
/// <param name="threshold">临近阈值，用于计算临近状态</param>
/// <returns>返回每个切割法中形状的多边形数组,该多边形的顶点以索引(在pointCloud中索引)的形式表示,
/// </returns>
CAM_API PLANE_SHAPE_CUT_VEC PlaneShapeCut(
	const PointCloudPtr& pointCloud,
	int minVertexNum,
	int maxVertexNum,
	double minSideDist = 10,
	double maxSideDist = 200,
	double threshold = 15.0);

/// <summary>
/// 基于三维点集拟合空间椭圆（复用已有的 CalPCA 结果）
/// </summary>
/// <param name="points">输入的三维点集（至少需要 6 个点）</param>
/// <param name="center">输出：椭圆的中心点 (Point3D)</param>
/// <param name="shortRadio">输出：短轴半径 (float)</param>
/// <param name="longRadio">输出：长轴半径 (float)</param>
/// <param name="ellipse">根据ellipse的大小，填充全部椭圆顶点</param>
/// <param name="incompEllipse">根据incompEllipse的大小，只填充覆盖points的椭圆顶点</param>
/// <returns>拟合成功返回 true，点数不足或退化为非椭圆返回 false</returns>
bool FitEllipse(
	const std::vector<Point3D>& points, 
	Point3D* center, 
	float* shortRadio, 
	float* longRadio,
	std::vector<Point3D>* ellipse = nullptr,
	std::vector<Point3D>* incompEllipse = nullptr);
	