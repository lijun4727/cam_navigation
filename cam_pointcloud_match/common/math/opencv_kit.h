#pragma once

#include "type.h"
#include <opencv2/opencv.hpp>
#include <vector>

struct SpotInfo {
    cv::Point2f center;     // 中心坐标
    float radius;           // 包围圆半径
};

/// <summary>
/// 检测白色圆形光斑，按半径从大到小排序
/// </summary>
/// <param name="image">输入图像 (BGR 或灰度)</param>
/// <param name="need_sort">是否需要排序</param>
/// <param name="min_radius">最小半径 (像素)</param>
/// <param name="max_radius">最大半径 (像素)</param>
/// <param name="threshold">白色阈值 (0-255)</param>
/// <param name="min_circularity">最小圆度 (0-1)</param>
/// <param name="exclude_overlaps">是否排除重合光斑（默认排除）</param>
/// <returns>光斑列表，按照顺时针进行排序，角度相同则按距质心距离从近到远排序</returns>
CAM_API std::vector<SpotInfo> DetectWhiteSpots(
    const cv::Mat& image,
	bool need_sort = false,
    float min_radius = 1.f,
    float max_radius = 50.f,
    int threshold = 200,
    float min_circularity = 0.6f,
    bool exclude_overlaps = false);

/// <summary>
/// 将leftImage和rightImage中共同存在的圆形光斑返回,并且按照leftSpots顺时针排序，rightImage和其对应的光斑也
/// 按照相同顺序排序放到rightSpots中，leftSpots和rightSpots大小相同，返回的圆形光斑半径不超过max_radius,且
/// 半径不小于 min_radius,且圆度不小于min_circularity,且亮度不小于threshold。
/// 算法如下：
/// ·调用DetectWhiteSpots(用DetectWhiteSpotsFromMore的参数填充DetectWhiteSpots中的对应参数，DetectWhiteSpots中
///   别的参数保持默认值)对leftImage和rightImage分别检测光斑，得到leftSpots和rightSpots列表
/// ·leftSpots和rightSpots的数量必须都大于3，否则返回空列表
/// ·先遍历leftSpots的每个leftSpot，算出leftSpot的Y坐标设为leftY，如果rightSpots上的y坐标在leftY±max_radius
///   像素范围内没有点，则删除这个leftSpot；同样的，遍历rightSpots的每个rightSpot，算出rightSpot的Y坐标设为
///   rightY，如果leftSpots上的y坐标在rightY±max_radius像素范围内没有点，则删除这个rightSpot
/// ·计算leftSpots中每个点leftSpot周围所有点的向量leftVec
///   计算rightSpots中每个点rightSpots周围所有点的向量rightVec,然后对比leftSpots和rightSpots中每个
///   点的refVec，设leftSpots对应rightSpots中的匹配集合为matchSpots,
///   matchSpots每个成员的类型为:
///     struct MatchSpot{
///         double matchScore; //匹配度为负数，越大越好
///         int leftIndex;     //在leftSpots中的索引
///         int rightIndex;    //在rightSpots中的索引
///     }
///   对应点之间的匹配度这么计算:
///   std::vector<MatchSpot> matchSpots; // 存储匹配结果
///   for(auto& leftSpot : leftSpots){
///       MatchSpot matchSpot; // 临时变量存储当前leftSpot的最佳匹配
///       matchSpot.matchScore = -std::numeric_limits<double>::infinity(); // 初始化匹配度为负无穷
///       for(auto& rightSpot : rightSpots){
///          设leftSpot的3个refVec为leftVec，rightSpot的3个refVec为rightVec
///          double matchScore = 0; // 向量相似度，越大越好
///          for(auto& lv : leftVec){
///             double vecScore = -std::numeric_limits<double>::infinity(); // 初始化匹配度为负无穷
///             for(auto& rv : rightVec){
///                  double vecAngle = lv和rv2个向量的形成的夹角(单位:弧度)                
///                  double vecLenRatio = 计算lv和rv的长度比值，取较小的长度除以较大的长度，范围在0-1之间
///                  auto score = (CV_PI - vecAngle) * (1 + vecLenRatio);
///                  if(score > vecScore){
///     	            vecScore = score;                 
///                  }
///             }
///             matchScore += vecScore;
///          }
///          if(matchSpot.matchScore < matchScore){
///             matchSpot.matchScore = matchScore;
///             matchSpot.leftIndex = leftSpot的索引;
///             matchSpot.rightIndex = rightSpot的索引;
///          }
///       }
///       matchSpots.emplace_back(matchSpot); // 将当前leftSpot的最佳匹配加入结果列表
///   }
///   将matchSpots按照匹配度从大到小排序，取前3个匹配度最高的匹配对
///   std::vector<MatchSpot> bestMatchSpots; // 存储匹配结果
///   double bestTotalMatchScore = -std::numeric_limits<double>::infinity(); // 总匹配度
///   for(auto& matchSpot : matchSpots){
///        如果已经算了超过3个匹配对，则停止遍历
///        从leftSpots和rightSpots中取出matchSpot对应的光斑，设为matchedLeftSpot和matchedRightSpot
///        计算vec = matchedRightSpot.center - matchedLeftSpot.center; // 计算匹配点的向量
///        然后将leftSpots中所有点的坐标加上vec设为matchLeftSpots,实现匹配点重合，计算匹配度(matchScore)，匹配度越大越好
///        //此处计算匹配度的算法如下：
///        std::vector<MatchSpot> matchTempSpots; // 存储匹配结果
///        for(auto& matchLeftSpot : matchLeftSpots){
///             matchLeftSpots中每个点matchLeftSpot找到rightSpots中距离最近的点nearRightSpot，计算matchLeftSpot和
///             nearRightSpot的的距离nearDist2
///             MatchSpot matchSpot; // 临时变量存储当前匹配对的匹配度
///             matchSpot.matchScore = -nearDist2; // 存储匹配度
///             matchSpot.leftIndex = matchLeftSpot的索引
///             matchSpot.rightIndex = nearRightSpot的索引
///             matchTempSpots.emplace_back(matchSpot); // 将当前匹配对的匹配度加入结果列表
///        }
///        检查matchTempSpots中每个成员matchTempSpot,保证matchTempSpot.leftIndex和matchTempSpot.rightIndex是一一对应，如果有
///        重复的leftIndex或者rightIndex，则留下matchTempSpot.matchScore最高的匹配对;
///        计算总匹配度totalMatchScore = matchTempSpots每个成员的匹配度之和，取totalMatchScore最大的匹配对设为bestMatchSpots
///        if(bestTotalMatchScore < totalMatchScore){
///             bestMatchSpots = matchTempSpots;
///         }
///   }
///   返回bestMatchSpots中leftIndex和rightIndex对应对应的光斑列表，跳充函数参数leftSpots和rightSpots，并且把leftSpots按照按照
///   顺时针进行排序，如果角度和质心距离一样，则按索引顺序排序，然后rightSpots填充为和leftSpots索引对应的光斑列表
/// 
/// </summary>
/// <param name="leftImage">左图像</param>
/// <param name="rightImage">右图像</param>
/// <param name="leftSpots">左图像光斑列表,按照顺时针进行排序，角度相同则按距质心距离从近到远排序</param>
/// <param name="rightSpots">右图像光斑列表，和leftSpots索引对应</param>
/// <param name="min_radius">最小半径 (像素)</param>
/// <param name="max_radius">最大半径 (像素)</param>
/// <param name="threshold">白色阈值 (0-255)</param>
/// <param name="min_circularity">最小圆度 (0-1)</param>
CAM_API void DetectWhiteSpotsFromMore(
    const cv::Mat& leftImage,
	const cv::Mat& rightImage,
	std::vector<SpotInfo>& leftSpots,
	std::vector<SpotInfo>& rightSpots,
    float min_radius = 0.1f,
    float max_radius = 50.f,
    int threshold = 127,
    float min_circularity = 0.6f);

/// <summary>
/// 在图像上绘制检测结果
/// </summary>
CAM_API cv::Mat DrawSpots(const cv::Mat& image, const std::vector<SpotInfo>& spots);

/**
 * @brief 使用左右红外像素坐标三角化，输出指定相机坐标系的3D点（支持不同畸变模型）
 *
 * @param left_pixel         左红外像素坐标 (u, v)
 * @param right_pixel        右红外像素坐标 (u, v)
 * @param K_left             左红外相机内参矩阵 3x3（OpenCV格式）
 * @param D_left             左红外相机畸变参数（OpenCV格式，1x5或1x4）
 * @param left_model         左红外畸变模型类型（DistortionModel枚举，见下方）
 * @param K_right            右红外相机内参矩阵 3x3（OpenCV格式）
 * @param D_right            右红外相机畸变参数（OpenCV格式，1x5或1x4）
 * @param right_model        右红外畸变模型类型（DistortionModel枚举，见下方）
 * @param R_left_to_right    左->右相机旋转矩阵 3x3（OpenCV格式）
 * @param t_left_to_right    左->右相机平移向量 3x1（OpenCV格式）
 * @param R_left_to_target   左->目标相机旋转矩阵 3x3（OpenCV格式）
 * @param t_left_to_target   左->目标相机平移向量 3x1（OpenCV格式）
 * @param point_depth        输出：目标相机坐标系下的3D点（Point3D*，需分配空间）
 * @return                   成功返回 true，失败返回 false
 *
 * DistortionModel 枚举说明：
 *   DISTORTION_NONE = 0           // 无畸变
 *   DISTORTION_BROWN_CONRADY = 1  // 标准畸变（OpenCV默认）
 *   DISTORTION_FISHEYE = 2        // 鱼眼畸变（F-Theta/Kannala-Brandt4等）
 *   DISTORTION_OTHER = 99         // 其他/未知畸变
 */
enum DistortionModel {
    DISTORTION_NONE = 0,
    DISTORTION_BROWN_CONRADY = 1,
    DISTORTION_FISHEYE = 2,
    DISTORTION_OTHER = 99
};
CAM_API bool TriangulatePointToCamera(
    const cv::Point2f& left_pixel,
    const cv::Point2f& right_pixel,
    const cv::Mat& K_left,
    const cv::Mat& D_left,
    int left_model,
    const cv::Mat& K_right,
    const cv::Mat& D_right,
    int right_model,
    const cv::Mat& R_left_to_right,
    const cv::Mat& T_left_to_right,
    const cv::Mat& R_left_to_target,
    const cv::Mat& T_left_to_target,
    Point3D* point_target);

/// <summary>
/// 
/// </summary>
/// <param name="left_pixel"></param>
/// <param name="right_pixel"></param>
/// <param name="K_left"></param>
/// <param name="D_left"></param>
/// <param name="left_model"></param>
/// <param name="K_right"></param>
/// <param name="D_right"></param>
/// <param name="right_model"></param>
/// <param name="left_to_right">左到右相机的外参矩阵 4x4（OpenCV格式）</param>
/// <param name="left_to_target">左到目标相机的外参矩阵 4x4（OpenCV格式）</param>
/// <param name="point_target"></param>
/// <returns></returns>
CAM_API bool TriangulatePointToCamera(
    const cv::Point2f& left_pixel,
    const cv::Point2f& right_pixel,
    const cv::Mat& K_left,
    const cv::Mat& D_left,
    int left_model,
    const cv::Mat& K_right,
    const cv::Mat& D_right,
    int right_model,
    const cv::Mat& left_to_right,
    const cv::Mat& left_to_target,
    Point3D* point_target);

CAM_API bool TriangulatePointToCamera(
    const cv::Point2f& left_pixel,
    const cv::Point2f& right_pixel,
	const CameraParameter& camParam,
    Point3D* point_target);

/// @brief 计算 left 的质心 LC、right 的质心 RC，把 right 全部
/// 平移 delta = LC - RC，然后只做循环位移匹配（不旋转），对每
/// 个位移计算点对平方距离和（更高效且与距离和单调），选取最小
/// 者并返回映射 <left_index, right_index>，同时输出最佳位移和 
/// RMSE。
/// @param left  左相机屏幕点（逆时针顺序）
/// @param right 右相机屏幕点（逆时针顺序）
/// @param outRmse 输出：旋转并移位后两组点的 RMS 误差（像素）
/// @return 返回索引对列表，元素为 <left_index, right_index>
std::vector<std::pair<int, int>> MatchCyclicOrderedPointsByRotation(
    const std::vector<cv::Point2f>& left,
    const std::vector<cv::Point2f>& right,
    double* outRmse = nullptr);