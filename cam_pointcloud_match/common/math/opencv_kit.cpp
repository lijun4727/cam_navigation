#include "opencv_kit.h"

#include <algorithm>
#include <cmath>

/// <summary>
/// 计算两个圆的交叠面积
/// r1, r2: 两个圆半径
/// d: 两个圆心距离
/// </summary>
double CircleIntersectionArea(double r1, double r2, double d) {
	if (d >= r1 + r2) {                    // 圆心距大于等于半径之和 ⇒ 不相交
		return 0.0;                        // 交叠面积为 0
	}
	if (d <= std::abs(r1 - r2)) {          // 一个圆完全包含另一个圆
		double r = std::min(r1, r2);       // 取小圆半径
		return CV_PI * r * r;              // 交叠面积等于小圆面积
	}

	double r1_2 = r1 * r1;                 // r1²
	double r2_2 = r2 * r2;                 // r2²
	double alpha = std::acos((d * d + r1_2 - r2_2) / (2.0 * d * r1)); // 圆心角 α
	double beta = std::acos((d * d + r2_2 - r1_2) / (2.0 * d * r2)); // 圆心角 β

	// 交叠面积公式（两个扇形面积之和减去三角形面积）
	double area = r1_2 * alpha + r2_2 * beta - 0.5 * std::sqrt(
		(-d + r1 + r2) * (d + r1 - r2) * (d - r1 + r2) * (d + r1 + r2));
	return area;                           // 返回交叠面积
}

/// <summary>
/// 重合判断（无论圆或椭圆，统一用包围圆近似）
/// overlap_ratio = 交叠面积 / 小圆面积
/// </summary>
bool IsOverlapping(const SpotInfo& a, const SpotInfo& b, double ratio_threshold) {
	double dx = a.center.x - b.center.x;   // 圆心 x 方向距离
	double dy = a.center.y - b.center.y;   // 圆心 y 方向距离
	double d = std::sqrt(dx * dx + dy * dy); // 圆心欧氏距离
	double r1 = a.radius;                  // 圆半径1（包围圆）
	double r2 = b.radius;                  // 圆半径2（包围圆）
	double inter_area = CircleIntersectionArea(r1, r2, d); // 计算交叠面积
	double min_area = CV_PI * std::min(r1, r2) * std::min(r1, r2); // 小圆面积
	if (min_area <= 1e-6) {                // 防止除零（极小值）
		return false;
	}

	double overlap_ratio = inter_area / min_area; // 交叠比例
	return overlap_ratio >= ratio_threshold;      // 大于阈值视为重合
}

cv::Mat SplitOverlappingSpots_NEW(
	const cv::Mat& gray,
	const cv::Mat& binary,
	float min_radius) {
	if (binary.empty()) {
		return cv::Mat();
	}

	cv::Mat binary8;
	if (binary.channels() == 1) {
		if (binary.depth() == CV_8U)
			binary8 = binary.clone();
		else
			binary.convertTo(binary8, CV_8U);
	}
	else {
		cv::Mat tmp;
		cv::cvtColor(binary, tmp, cv::COLOR_BGR2GRAY);
		cv::threshold(tmp, binary8, 127, 255, cv::THRESH_BINARY);
	}

	if (min_radius <= 0.f) {
		return binary8.clone();
	}

	cv::Mat dist;
	cv::distanceTransform(binary8, dist, cv::DIST_L2, 5);

	double max_dist = 0.0;
	cv::minMaxLoc(dist, nullptr, &max_dist);
	if (max_dist <= 1.0) {
		return binary8.clone();
	}

	double peak_thresh = std::max(1.0, min_radius * 0.5);
	if (peak_thresh >= max_dist) {
		peak_thresh = max_dist * 0.5;
	}
	if (peak_thresh <= 0.0) {
		return binary8.clone();
	}

	cv::Mat dist_thresh;
	cv::threshold(dist, dist_thresh, peak_thresh, 255.0, cv::THRESH_BINARY);
	dist_thresh.convertTo(dist_thresh, CV_8U);

	if (cv::countNonZero(dist_thresh) == 0) {
		return binary8.clone();
	}

	cv::Mat markers;
	int comp_count = cv::connectedComponents(dist_thresh, markers);
	if (comp_count <= 1) {
		return binary8.clone();
	}

	cv::Mat gray8;
	if (gray.empty()) {
		gray8 = binary8;
	}
	else if (gray.channels() == 1) {
		if (gray.depth() == CV_8U)
			gray8 = gray;
		else
			gray.convertTo(gray8, CV_8U);
	}
	else {
		cv::cvtColor(gray, gray8, cv::COLOR_BGR2GRAY);
	}

	cv::Mat color;
	cv::cvtColor(gray8, color, cv::COLOR_GRAY2BGR);
	cv::watershed(color, markers);

	binary8.setTo(0, markers == -1 & binary8 == 255);

	return binary8;
}

/// <summary>
/// 分水岭分割重合光斑
/// </summary>
cv::Mat SplitOverlappingSpots(
	const cv::Mat& gray,
	const cv::Mat& binary,
	int min_radius) {
	if (binary.empty()) {
		return cv::Mat();
	}

	cv::Mat binary8;
	if (binary.channels() == 1) {
		if (binary.depth() == CV_8U)
			binary8 = binary.clone();
		else
			binary.convertTo(binary8, CV_8U);
	}
	else {
		cv::Mat tmp;
		cv::cvtColor(binary, tmp, cv::COLOR_BGR2GRAY);
		cv::threshold(tmp, binary8, 127, 255, cv::THRESH_BINARY);
	}

	if (min_radius <= 0) {
		return binary8.clone();
	}

	cv::Mat dist;
	cv::distanceTransform(binary8, dist, cv::DIST_L2, 5);

	double max_dist = 0.0;
	cv::minMaxLoc(dist, nullptr, &max_dist);
	if (max_dist <= 1.0) {
		return binary8.clone();
	}

	double peak_thresh = std::max(1.0, min_radius * 0.5);
	if (peak_thresh >= max_dist) {
		peak_thresh = max_dist * 0.5;
	}
	if (peak_thresh <= 0.0) {
		return binary8.clone();
	}

	cv::Mat dist_thresh;
	cv::threshold(dist, dist_thresh, peak_thresh, 255, cv::THRESH_BINARY);
	dist_thresh.convertTo(dist_thresh, CV_8U);

	if (cv::countNonZero(dist_thresh) == 0) {
		return binary8.clone();
	}

	cv::Mat markers;
	int comp_count = cv::connectedComponents(dist_thresh, markers);
	if (comp_count <= 1) {
		return binary8.clone();
	}

	markers = markers + 1;                 // 背景标签从1开始，前景从2开始
	markers.setTo(1, binary8 == 0);        // 固定背景标签

	cv::Mat gray8;
	if (gray.empty()) {
		gray8 = binary8;
	}
	else if (gray.channels() == 1) {
		if (gray.depth() == CV_8U)
			gray8 = gray;
		else
			gray.convertTo(gray8, CV_8U);
	}
	else {
		cv::cvtColor(gray, gray8, cv::COLOR_BGR2GRAY);
	}

	cv::Mat color;
	cv::cvtColor(gray8, color, cv::COLOR_GRAY2BGR);
	cv::watershed(color, markers);

	cv::Mat separated = cv::Mat::zeros(binary8.size(), CV_8U);
	separated.setTo(255, markers > 1);     // 只保留分割出的光斑区域
	separated.setTo(0, markers == -1);     // 分割线设为0
	separated.setTo(0, binary8 == 0);      // 背景设为0

	cv::Mat foreground_mask = (binary8 == 255); // 原始前景掩码
	cv::Mat split_mask;                          // 分割线掩码（仅限前景区域）
	cv::compare(markers, -1, split_mask, cv::CMP_EQ);
	split_mask &= foreground_mask;

	cv::Mat orig_labels;                         // 原始前景连通域
	int orig_count = cv::connectedComponents(binary8, orig_labels);

	std::vector<bool> has_split(orig_count, false);
	for (int y = 0; y < orig_labels.rows; ++y) {
		const int* label_row = orig_labels.ptr<int>(y);
		const uchar* split_row = split_mask.ptr<uchar>(y);
		for (int x = 0; x < orig_labels.cols; ++x) {
			if (split_row[x] != 0) {
				int id = label_row[x];
				if (id > 0 && id < orig_count) {
					has_split[id] = true;
				}
			}
		}
	}

	for (int id = 1; id < orig_count; ++id) {
		if (!has_split[id]) {                     // 未发生分割的区域回填原始光斑
			separated.setTo(255, orig_labels == id);
		}
	}

	return separated;
}

std::vector<SpotInfo> DetectWhiteSpots(
	const cv::Mat& image,
	bool need_sort,
	float min_radius,
	float max_radius,
	int threshold,
	float min_circularity,
	bool exclude_overlaps) {

	std::vector<SpotInfo> spots;               // 保存光斑结果
	if (image.empty()) {
		return spots;
	}

	// 转灰度
	cv::Mat gray;                              // 灰度图
	if (image.channels() == 3) {              // BGR 图像
		cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
	}
	else if (image.channels() == 4) {         // BGRA 图像
		cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
	}
	else {
		gray = image;                          // 已经是灰度图，直接引用
	}

	// 高斯模糊，削弱边缘渐变
	cv::Mat blurred = gray;                           // 模糊图
	//cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0); // 5x5 高斯滤波

	double maxVal;
	cv::minMaxLoc(gray, nullptr, &maxVal);

	// 二值化提取白色区域
	cv::Mat binary;                            // 二值图
	maxVal = std::min(maxVal, static_cast<double>(threshold));
	cv::threshold(blurred, binary, maxVal, 255, cv::THRESH_BINARY); // 白色阈值分割

	// 闭运算填充小孔洞
	//cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)); // 结构元素
	//cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel); // 闭运算修复小孔

	// 重合分割（避免光斑粘连）
	cv::Mat separated = exclude_overlaps ? SplitOverlappingSpots_NEW(blurred, binary, min_radius) : binary;

	// 腐蚀
	//auto kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	//cv::erode(separated, separated, kernel, cv::Point(-1, -1), 1);

	// 查找轮廓
	std::vector<std::vector<cv::Point>> contours; // 所有轮廓点
	cv::findContours(separated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE); // 只取外轮廓
	for (auto& contour : contours) {
		// 用 bbox 限制填充范围，避免对整张大图操作，提升效率
		cv::Rect bbox = cv::boundingRect(contour);
		// 边界加1像素余量，避免边缘像素因坐标转换被裁掉
		bbox.x = std::max(0, bbox.x - 1);
		bbox.y = std::max(0, bbox.y - 1);
		bbox.width = std::min(separated.cols - bbox.x, bbox.width + 2);
		bbox.height = std::min(separated.rows - bbox.y, bbox.height + 2);

		// 轮廓坐标平移到局部小图坐标系
		std::vector<cv::Point> shifted;
		shifted.reserve(contour.size());
		for (const auto& p : contour) {
			shifted.emplace_back(p.x - bbox.x, p.y - bbox.y);
		}

		// 在局部小图上填充轮廓
		cv::Mat localMask = cv::Mat::zeros(bbox.size(), CV_8U);
		std::vector<std::vector<cv::Point>> single = { shifted };
		cv::drawContours(localMask, single, 0, cv::Scalar(255), cv::FILLED);

		// 取出填充区域内所有非零像素坐标（局部坐标）
		std::vector<cv::Point> filledLocal;
		cv::findNonZero(localMask, filledLocal);

		// 平移回原图坐标系，替换原 contour（原来只有边界点，现在含边界+内部）
		std::vector<cv::Point> filledGlobal;
		filledGlobal.reserve(filledLocal.size());
		for (const auto& p : filledLocal) {
			filledGlobal.emplace_back(p.x + bbox.x, p.y + bbox.y);
		}
		contour = std::move(filledGlobal);
	}
	// 用 connectedComponentsWithStats 替代 findContours，检测极小连通域并构造 contours 列表
	//std::vector<std::vector<cv::Point>> contours;
	//{
	//	cv::Mat labels, stats, centroids;
	//	// 使用 8 邻域连通组件统计
	//	int ncomp = cv::connectedComponentsWithStats(separated, labels, stats, centroids, 8, CV_32S);
	//	// 0 为背景，从 1 开始
	//	for (int id = 1; id < ncomp; ++id) {
	//		int area = stats.at<int>(id, cv::CC_STAT_AREA);
	//		// area 是像素数；这里允许非常小的区域，阈值按需调整，例如 >=1
	//		if (area < 1) continue;

	//		int left = stats.at<int>(id, cv::CC_STAT_LEFT);
	//		int top = stats.at<int>(id, cv::CC_STAT_TOP);
	//		int width = stats.at<int>(id, cv::CC_STAT_WIDTH);
	//		int height = stats.at<int>(id, cv::CC_STAT_HEIGHT);

	//		// 收集该连通域的像素点（只在 bbox 内遍历，效率可接受）
	//		std::vector<cv::Point> pts;
	//		for (int y = top; y < top + height; ++y) {
	//			const int* row = labels.ptr<int>(y);
	//			for (int x = left; x < left + width; ++x) {
	//				if (row[x] == id) pts.emplace_back(x, y);
	//			}
	//		}
	//		if (!pts.empty())
	//			contours.push_back(std::move(pts));
	//	}
	//}
	for (const auto& contour : contours) {     // 遍历轮廓	
		// 最小外接圆
		cv::Point2f center;                    // 外接圆中心
		float radius;                          // 外接圆半径
		cv::minEnclosingCircle(contour, center, radius); // 计算外接圆

		if (/*radius < min_radius || */radius > max_radius) continue; // 半径范围过滤

		std::vector<uchar> contourPixels;
		contourPixels.reserve(contour.size());
		for (const auto& p : contour) {
			if (p.x >= 0 && p.x < gray.cols && p.y >= 0 && p.y < gray.rows) {
				contourPixels.push_back(gray.at<uchar>(p.y, p.x));
			}
		}

		// 归一化灰度
		std::vector<double> normColor;
		normColor.reserve(contourPixels.size());
		std::transform(
			contourPixels.begin(),
			contourPixels.end(),
			std::back_inserter(normColor),
			[threshold](uchar v) { return v / 255.0; });

		// 计算权重（归一化灰度占比）
		auto sum = std::accumulate(normColor.begin(), normColor.end(), 0.0);
		std::vector<double> weight;
		std::transform(
			normColor.begin(),
			normColor.end(),
			std::back_inserter(weight),
			[sum](double v) { return v / sum; });


		//根据权重计算加权中心（质心）
		center = cv::Point2f(0.0f, 0.0f);
		for (size_t i = 0; i < contour.size(); ++i) {
			center.x += contour[i].x * weight[i];
			center.y += contour[i].y * weight[i];
		}

		// 圆度
		//double area = cv::contourArea(contour);              // 轮廓面积
		//if (area) {
		//	double perimeter = cv::arcLength(contour, true);    // 周长
		//	//if (area <= 0.0 || perimeter <= 1e-6) {
		//	//	continue; // 防止无效轮廓或除零
		//	//}

		//	float circularity = static_cast<float>(4.0 * CV_PI * area / (perimeter * perimeter)); // 圆度
		//	if (!std::isfinite(circularity)) {
		//		continue; // 防止极端情况下出现 NaN / Inf
		//	}

		//	bool is_circle_like = (circularity >= min_circularity); // 圆度达标

		//	if (!is_circle_like)
		//		continue;      // 不是圆跳过
		//}


		spots.push_back({ center, radius });                    // 保存结果
	}

	// 以 spots 的中点（centroid）为参考，按顺时针排序
	if (!spots.empty() && need_sort) {
		// 计算中点（质心）
		double cx = 0.0, cy = 0.0;
		for (const auto& s : spots) {
			cx += static_cast<double>(s.center.x);
			cy += static_cast<double>(s.center.y);
		}
		cx /= static_cast<double>(spots.size());
		cy /= static_cast<double>(spots.size());

		const double PI = std::acos(-1.0);
		// 将角度归一到 [0, 2π) 并按顺时针排序（顺时针等于角度从大到小）
		std::sort(spots.begin(), spots.end(),
			[cx, cy, PI](const SpotInfo& a, const SpotInfo& b) {
				double angA = std::atan2(static_cast<double>(a.center.y) - cy,
					static_cast<double>(a.center.x) - cx);
				double angB = std::atan2(static_cast<double>(b.center.y) - cy,
					static_cast<double>(b.center.x) - cx);
				if (angA < 0) angA += 2.0 * PI;
				if (angB < 0) angB += 2.0 * PI;
				if (angA == angB) {
					// 角度相同则按距质心距离从近到远（或其他策略）排序
					double da = (a.center.x - cx) * (a.center.x - cx) + (a.center.y - cy) * (a.center.y - cy);
					double db = (b.center.x - cx) * (b.center.x - cx) + (b.center.y - cy) * (b.center.y - cy);
					return da < db;
				}
				// ang 大 ⇒ 先出现，实现顺时针
				return angA > angB;
			});
	}

	if (!exclude_overlaps) {                    // 不排除重合则直接返回
		return spots;
	}

	// 重合过滤（基于包围圆，涵盖圆-椭圆、椭圆-椭圆）
	constexpr double overlap_ratio_threshold = 0.3; // 重合阈值
	std::vector<bool> overlapped(spots.size(), false);
	for (size_t i = 0; i < spots.size(); ++i) {
		for (size_t j = i + 1; j < spots.size(); ++j) {
			if (IsOverlapping(spots[i], spots[j], overlap_ratio_threshold)) {
				overlapped[i] = true;
				overlapped[j] = true;
			}
		}
	}
	std::vector<SpotInfo> filtered;
	for (size_t i = 0; i < spots.size(); ++i) {
		if (!overlapped[i]) {
			filtered.push_back(spots[i]);
		}
	}

	return filtered;                            // 返回过滤后的结果
}

CAM_API void DetectWhiteSpotsFromMore(
	const cv::Mat& leftImage,
	const cv::Mat& rightImage,
	std::vector<SpotInfo>& leftSpots,
	std::vector<SpotInfo>& rightSpots,
	float min_radius,
	float max_radius,
	int threshold,
	float min_circularity)
{
	// 先清空输出，保证任意提前返回都不会带出旧数据。
	leftSpots.clear();
	rightSpots.clear();

	// -----------------------------
	// 1. 基本参数检查
	// -----------------------------
	if (leftImage.empty() || rightImage.empty()) {
		return;
	}
	if (min_radius < 0.0f || max_radius < min_radius) {
		return;
	}

	// 对阈值做夹紧，避免传入非法值。
	if (threshold < 0) {
		threshold = 0;
	}
	else if (threshold > 255) {
		threshold = 255;
	}

	// -----------------------------
	// 2. 内部辅助结构
	// -----------------------------
	struct MatchSpot {
		double matchScore = -std::numeric_limits<double>::infinity(); // 匹配度，越大越好
		int leftIndex = -1;   // 左图索引
		int rightIndex = -1;  // 右图索引
	};

	struct SpotPair {
		SpotInfo left;   // 左图光斑
		SpotInfo right;  // 与左图对应的右图光斑
		int order = 0;   // 原始顺序，用于稳定排序
	};

	// -----------------------------
	// 3. 数值辅助函数
	// -----------------------------

	// 将数值限制在指定区间，主要用于 acos 之前防止浮点误差越界。
	auto ClampDouble = [](double value, double minValue, double maxValue) -> double {
		if (value < minValue) return minValue;
		if (value > maxValue) return maxValue;
		return value;
		};

	// 判断某个光斑在另一组光斑中，是否存在 y 坐标相差不超过 yTolerance 的候选点。
	auto HasYNeighbor = [](const SpotInfo& spot, const std::vector<SpotInfo>& others, float yTolerance) -> bool {
		for (const auto& other : others) {
			if (std::abs(spot.center.y - other.center.y) <= yTolerance) {
				return true;
			}
		}
		return false;
		};

	// 按头文件要求：删除在另一张图中 y±5 范围内找不到对应候选点的光斑。
	auto FilterByYRange = [&](const std::vector<SpotInfo>& src, const std::vector<SpotInfo>& dst) -> std::vector<SpotInfo> {
		std::vector<SpotInfo> filtered;
		filtered.reserve(src.size());

		for (const auto& spot : src) {
			if (HasYNeighbor(spot, dst, max_radius)) {
				filtered.push_back(spot);
			}
		}
		return filtered;
		};

	// 计算每个点与周边所有点形成的向量
	auto BuildReferenceVectors = [max_radius](const std::vector<SpotInfo>& spots) -> std::vector<std::vector<cv::Point2f>> {
		std::vector<std::vector<cv::Point2f>> refVectors(spots.size());

		if (spots.size() <= 1) {
			return refVectors;
		}

		for (size_t i = 0; i < spots.size(); ++i) {
          refVectors[i].reserve(spots.size() - 1);

			for (size_t j = 0; j < spots.size(); ++j) {
				if (i == j) {
					continue;
				}

				auto V = spots[j].center - spots[i].center;
				refVectors[i].push_back(V);
			}
		}

		return refVectors;
		};

	// 按头文件注释中的公式计算两个参考向量的匹配分数：
	// score = (PI - vecAngle) * (1 + vecLenRatio)
	auto CalcVectorScore = [&](
		const cv::Point2f& lv,
		const cv::Point2f& rv,
		float leftRadius,
		float rightRadius,
		float maxRadius) -> double {
			constexpr double eps = 1e-12;

			double leftLen = cv::norm(lv);
			double rightLen = cv::norm(rv);
			if (leftLen <= eps || rightLen <= eps) {
				return -std::numeric_limits<double>::infinity();
			}

			double cosValue = static_cast<double>(lv.dot(rv)) / (leftLen * rightLen);
			cosValue = ClampDouble(cosValue, -1.0, 1.0);

			double vecAngle = std::acos(cosValue); // 单位：弧度
			double maxLen = std::max(leftLen, rightLen);
			double minLen = std::min(leftLen, rightLen);
			double vecLenRatio = (maxLen <= eps) ? 0.0 : (minLen / maxLen);

			return (CV_PI - vecAngle) * (1.0 + vecLenRatio) * (maxRadius - std::abs(leftRadius - rightRadius));
		};

	// 计算两个圆之间的 LOT（交叉比）。
	// 按头文件注释：LOT = 交叉面积 / 总面积，其中总面积 = areaA + areaB。
	//auto CalcLOT = [max_radius](const SpotInfo& a, const SpotInfo& b) -> double {
	//	double dx = static_cast<double>(a.center.x) - static_cast<double>(b.center.x);
	//	double dy = static_cast<double>(a.center.y) - static_cast<double>(b.center.y);
	//	double distance = std::sqrt(dx * dx + dy * dy);

	//	//double areaA = CV_PI * static_cast<double>(a.radius) * static_cast<double>(a.radius);
	//	//double areaB = CV_PI * static_cast<double>(b.radius) * static_cast<double>(b.radius);
	//	double totalArea = CV_PI * static_cast<double>(max_radius) * static_cast<double>(max_radius) * 2.0;
	//	//if (totalArea <= 1e-12) {
	//	//	return 0.0;
	//	//}

	//	double interArea = CircleIntersectionArea(a.radius, b.radius, distance);
	//	return interArea / totalArea;
	//	};

	// 保证匹配关系 leftIndex / rightIndex 一一对应。
	// 若出现重复索引，则只保留 matchScore 更高的项。
	auto KeepUniqueMatches = [](std::vector<MatchSpot> matches, size_t leftCount, size_t rightCount) -> std::vector<MatchSpot> {
		std::sort(
			matches.begin(),
			matches.end(),
			[](const MatchSpot& a, const MatchSpot& b) {
				return a.matchScore > b.matchScore;
			});

		std::vector<bool> leftUsed(leftCount, false);
		std::vector<bool> rightUsed(rightCount, false);
		std::vector<MatchSpot> uniqueMatches;
		uniqueMatches.reserve(matches.size());

		for (const auto& match : matches) {
			if (match.leftIndex < 0 || match.rightIndex < 0) {
				continue;
			}
			if (match.leftIndex >= static_cast<int>(leftCount) ||
				match.rightIndex >= static_cast<int>(rightCount)) {
				continue;
			}
			if (leftUsed[match.leftIndex] || rightUsed[match.rightIndex]) {
				continue;
			}

			leftUsed[match.leftIndex] = true;
			rightUsed[match.rightIndex] = true;
			uniqueMatches.push_back(match);
		}

		return uniqueMatches;
		};

	// 按左图点顺时针排序。
	// 若角度相同，则按离质心距离从近到远；
	// 若角度和距离都相同，则按原始顺序稳定排序。
	auto SortPairsClockwiseByLeft = [](std::vector<SpotPair>& pairs) {
		if (pairs.empty()) {
			return;
		}

		double cx = 0.0;
		double cy = 0.0;
		for (const auto& pair : pairs) {
			cx += static_cast<double>(pair.left.center.x);
			cy += static_cast<double>(pair.left.center.y);
		}
		cx /= static_cast<double>(pairs.size());
		cy /= static_cast<double>(pairs.size());

		const double PI = std::acos(-1.0);
		constexpr double eps = 1e-9;

		std::sort(
			pairs.begin(),
			pairs.end(),
			[cx, cy, PI, eps](const SpotPair& a, const SpotPair& b) {
				double angA = std::atan2(
					static_cast<double>(a.left.center.y) - cy,
					static_cast<double>(a.left.center.x) - cx);
				double angB = std::atan2(
					static_cast<double>(b.left.center.y) - cy,
					static_cast<double>(b.left.center.x) - cx);

				if (angA < 0.0) angA += 2.0 * PI;
				if (angB < 0.0) angB += 2.0 * PI;

				if (std::abs(angA - angB) > eps) {
					return angA > angB; // 顺时针
				}

				double da =
					(static_cast<double>(a.left.center.x) - cx) * (static_cast<double>(a.left.center.x) - cx) +
					(static_cast<double>(a.left.center.y) - cy) * (static_cast<double>(a.left.center.y) - cy);
				double db =
					(static_cast<double>(b.left.center.x) - cx) * (static_cast<double>(b.left.center.x) - cx) +
					(static_cast<double>(b.left.center.y) - cy) * (static_cast<double>(b.left.center.y) - cy);

				if (std::abs(da - db) > eps) {
					return da < db;
				}

				return a.order < b.order;
			});
		};

	// -----------------------------
	// 4. 分别检测左右图中的候选圆形光斑
	// 这里严格使用 DetectWhiteSpots，并把当前函数参数映射过去。
	// -----------------------------
	std::vector<SpotInfo> leftCandidates = DetectWhiteSpots(
		leftImage,
		false,
		min_radius,
		max_radius,
		threshold,
		min_circularity,
		false);

	std::vector<SpotInfo> rightCandidates = DetectWhiteSpots(
		rightImage,
		false,
		min_radius,
		max_radius,
		threshold,
		min_circularity,
		false);
	if (leftCandidates.empty() ||
		rightCandidates.empty() ||
		leftCandidates.size() > MAX_SPOT_NUM ||
		rightCandidates.size() > MAX_SPOT_NUM) {
		return;
	}

	//#ifdef DEBUG_LOG
	//	auto leftImageSpots = DrawSpots(leftImage, leftCandidates);
	//	auto rightImageSpots = DrawSpots(rightImage, rightCandidates);
	//#endif // DEBUG_LOG

		// 按头文件注释要求：
		// 仅当左右两边数量都小于 3 时，才直接返回空结果。
	if (leftCandidates.size() < 3 || rightCandidates.size() < 3) {
		return;
	}

	// -----------------------------
	// 5. 用 y±5 规则剔除单侧独有点
	// 按注释要求先过滤 left，再过滤 right。
	// -----------------------------
	leftCandidates = FilterByYRange(leftCandidates, rightCandidates);
	rightCandidates = FilterByYRange(rightCandidates, leftCandidates);

	// 经过这一步之后，如果任意一侧为空，则无法继续。
	if (leftCandidates.empty() || rightCandidates.empty()) {
		return;
	}

	// -----------------------------
	// 6. 为每个点建立最近邻参考向量
	// -----------------------------
	std::vector<std::vector<cv::Point2f>> leftRefVectors = BuildReferenceVectors(leftCandidates);
	std::vector<std::vector<cv::Point2f>> rightRefVectors = BuildReferenceVectors(rightCandidates);

	// -----------------------------
	// 7. 为每个左点找“局部几何结构最相似”的最佳右点
	// 每个左点只保留一个得分最高的右点。
	// -----------------------------
	std::vector<MatchSpot> matchSpots;
	matchSpots.reserve(leftCandidates.size());

	auto GetCappedMaxRadius = [max_radius](const std::vector<SpotInfo>& candidates) -> float {
		float radiusMax = 0.0f;
		for (const auto& candidate : candidates) {
			radiusMax = std::max(radiusMax, candidate.radius);
		}
		return std::min(radiusMax, max_radius);
		};

	auto leftCandidatesMaxRadius = GetCappedMaxRadius(leftCandidates);
	auto rightCandidatesMaxRadius = GetCappedMaxRadius(rightCandidates);
	auto maxRadius = std::max(leftCandidatesMaxRadius, rightCandidatesMaxRadius);

	for (size_t leftIndex = 0; leftIndex < leftCandidates.size(); ++leftIndex) {
		const auto& leftVecs = leftRefVectors[leftIndex];
		if (leftVecs.empty()) {
			continue;
		}

		MatchSpot bestMatch;
		bestMatch.leftIndex = static_cast<int>(leftIndex);

		for (size_t rightIndex = 0; rightIndex < rightCandidates.size(); ++rightIndex) {
			const auto& rightVecs = rightRefVectors[rightIndex];
			if (rightVecs.empty()) {
				continue;
			}

			double matchScore = 0.0;
			bool valid = true;

			for (const auto& lv : leftVecs) {
				double vecScore = -std::numeric_limits<double>::infinity();

				for (const auto& rv : rightVecs) {
					double score = CalcVectorScore(
						lv,
						rv,
						leftCandidates[leftIndex].radius,
						rightCandidates[rightIndex].radius,
						maxRadius);
					if (score > vecScore) {
						vecScore = score;
					}
				}

				if (!std::isfinite(vecScore)) {
					valid = false;
					break;
				}

				matchScore += vecScore;
			}

			if (!valid) {
				continue;
			}

			if (matchScore > bestMatch.matchScore) {
				bestMatch.matchScore = matchScore;
				bestMatch.rightIndex = static_cast<int>(rightIndex);
			}
		}

		if (bestMatch.rightIndex >= 0 && std::isfinite(bestMatch.matchScore)) {
			matchSpots.push_back(bestMatch);
		}
	}

	matchSpots = KeepUniqueMatches(
		matchSpots,
		leftCandidates.size(),
		rightCandidates.size());

	if (matchSpots.empty()) {
		return;
	}

	std::vector<SpotPair> pairs;
	pairs.reserve(matchSpots.size());
	for (size_t i = 0; i < matchSpots.size(); ++i) {
		const auto& match = matchSpots[i];
		if (match.leftIndex < 0 || match.rightIndex < 0) {
			continue;
		}
		if (match.leftIndex >= static_cast<int>(leftCandidates.size()) ||
			match.rightIndex >= static_cast<int>(rightCandidates.size())) {
			continue;
		}

		SpotPair pair;
		pair.left = leftCandidates[match.leftIndex];
		pair.right = rightCandidates[match.rightIndex];
		pair.order = static_cast<int>(i);
		pairs.push_back(pair);
	}
	if (pairs.empty()) {
		return;
	}

	SortPairsClockwiseByLeft(pairs);

	leftSpots.reserve(pairs.size());
	rightSpots.reserve(pairs.size());

	for (const auto& pair : pairs) {
		leftSpots.push_back(pair.left);
		rightSpots.push_back(pair.right);
	}
}

cv::Mat DrawSpots(const cv::Mat& image, const std::vector<SpotInfo>& spots) {
	cv::Mat out;                                // 输出图像
	if (image.channels() == 1)                  // 输入是灰度图
		cv::cvtColor(image, out, cv::COLOR_GRAY2BGR); // 转为 BGR 便于彩色绘制
	else
		out = image.clone();                    // 彩色图直接复制

	for (size_t i = 0; i < spots.size(); ++i) { // 遍历光斑
		const auto& s = spots[i];               // 当前光斑
		cv::circle(out, s.center, static_cast<int>(s.radius), cv::Scalar(0, 255, 0), 2); // 绿色包围圆
		cv::drawMarker(out, s.center, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 8, 2);    // 红色中心点
		cv::putText(out, std::to_string(i),
			cv::Point(static_cast<int>(s.center.x + s.radius + 3),
				static_cast<int>(s.center.y)),
			cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1); // 编号标注
	}
	return out;                                 // 返回可视化结果
}

namespace {
	bool UndistortPointByModel(
		const cv::Point2f& pixel,
		const cv::Mat& K,
		const cv::Mat& D,
		int model,
		std::vector<cv::Point2f>& undistorted)
	{
		if (K.empty()) return false;

		bool no_distortion = D.empty() || cv::countNonZero(D) == 0;

		if (no_distortion) {
			// 只做归一化变换
			cv::Mat Kinv = K.inv();
			cv::Mat pt = (cv::Mat_<double>(3, 1) << pixel.x, pixel.y, 1.0);
			cv::Mat norm = Kinv * pt;
			undistorted.clear();
			undistorted.emplace_back(
				static_cast<float>(norm.at<double>(0, 0) / norm.at<double>(2, 0)),
				static_cast<float>(norm.at<double>(1, 0) / norm.at<double>(2, 0))
			);
			return true;
		}

		cv::Mat K64;
		K.convertTo(K64, CV_64F);

		cv::Mat D64;
		if (D.empty()) {
			D64 = cv::Mat::zeros(1, 5, CV_64F);
		}
		else {
			D.convertTo(D64, CV_64F);
		}

		std::vector<cv::Point2f> src = { pixel };

		switch (model) {
		case DISTORTION_FISHEYE: {
			cv::Mat Df;
			if (D64.total() >= 4) {
				Df = D64.reshape(1, 1).colRange(0, 4).clone();
			}
			else {
				Df = cv::Mat::zeros(1, 4, CV_64F);
			}
			cv::fisheye::undistortPoints(src, undistorted, K64, Df);
			break;
		}
		case DISTORTION_NONE:
		case DISTORTION_OTHER: {
			cv::Mat Dz = cv::Mat::zeros(1, 5, CV_64F);
			cv::undistortPoints(src, undistorted, K64, Dz);
			break;
		}
		default:
			cv::undistortPoints(src, undistorted, K64, D64);
			break;
		}
		return !undistorted.empty();
	}
}

bool TriangulatePointToCamera(
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
	Point3D* point_target)
{
	if (K_left.empty() || K_right.empty() || R_left_to_right.empty() || T_left_to_right.empty()
		|| R_left_to_target.empty() || T_left_to_target.empty()) {
		std::cout << "TriangulatePointToCamera:point empty" << std::endl;
		return false;
	}

	// 1) 归一化/去畸变点（保持 float 输出）
	std::vector<cv::Point2f> left_norm_vec, right_norm_vec;
	if (!UndistortPointByModel(left_pixel, K_left, D_left, left_model, left_norm_vec)) {
		std::cout << "TriangulatePointToCamera:UndistortPointByModel1失败" << std::endl;
		return false;
	}
	if (!UndistortPointByModel(right_pixel, K_right, D_right, right_model, right_norm_vec)) {
		std::cout << "TriangulatePointToCamera:UndistortPointByModel2失败" << std::endl;
		return false;
	}
	if (left_norm_vec.empty() || right_norm_vec.empty()) {
		std::cout << "TriangulatePointToCamera:归一化失败" << std::endl;
		return false;
	}

	// 2) 将归一化点转换为 CV_64F 的 2xN 矩阵（triangulatePoints 要么接受 2xN 的 Mat）
	cv::Mat pts1(2, 1, CV_64F), pts2(2, 1, CV_64F);
	pts1.at<double>(0, 0) = static_cast<double>(left_norm_vec[0].x);
	pts1.at<double>(1, 0) = static_cast<double>(left_norm_vec[0].y);
	pts2.at<double>(0, 0) = static_cast<double>(right_norm_vec[0].x);
	pts2.at<double>(1, 0) = static_cast<double>(right_norm_vec[0].y);

	// 3) 准备投影矩阵（归一化坐标系下），类型 CV_64F
	cv::Mat Rlr, tlr, Rld, tld;
	R_left_to_right.convertTo(Rlr, CV_64F);
	T_left_to_right.convertTo(tlr, CV_64F);
	R_left_to_target.convertTo(Rld, CV_64F);
	T_left_to_target.convertTo(tld, CV_64F);

	cv::Mat P1 = cv::Mat::zeros(3, 4, CV_64F);
	cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
	I.copyTo(P1(cv::Rect(0, 0, 3, 3))); // P1 = [I|0]

	cv::Mat P2 = cv::Mat::zeros(3, 4, CV_64F);
	Rlr.copyTo(P2(cv::Rect(0, 0, 3, 3))); // left->right 旋转
	// 注意：这里的 t (tlr) 必须是相机坐标系下的平移（米），不要提前乘 fx/fy 或 K
	P2.at<double>(0, 3) = tlr.at<double>(0, 0);
	P2.at<double>(1, 3) = tlr.at<double>(1, 0);
	P2.at<double>(2, 3) = tlr.at<double>(2, 0);

	// 调试输出（便于定位）
	//std::cout << "P1:\n" << P1 << std::endl;
	//std::cout << "P2:\n" << P2 << std::endl;
	//std::cout << "left_norm: " << left_norm_vec[0] << " right_norm: " << right_norm_vec[0] << std::endl;

	// 4) 三角化（输出 points4d 为 CV_64F，4xN）
	cv::Mat points4d;
	cv::triangulatePoints(P1, P2, pts1, pts2, points4d);

	//std::cout << "points4d:\n" << points4d << std::endl;

	// 5) 读取齐次坐标（确保 points4d 是 CV_64F）
	if (points4d.empty() || points4d.rows < 4 || points4d.cols < 1) {
		std::cout << "TriangulatePointToCamera:无法读取齐次坐标" << std::endl;
		return false;
	}

	double w = points4d.at<double>(3, 0);
	//std::cout << "w: " << w << std::endl;
	if (!std::isfinite(w) || std::abs(w) < 1e-12) {
		std::cout << "TriangulatePointToCamera:w过小" << points4d << std::endl;
		return false;
	}

	cv::Vec3d X_left(
		points4d.at<double>(0, 0) / w,
		points4d.at<double>(1, 0) / w,
		points4d.at<double>(2, 0) / w);

	// 6) 从左相机坐标系变换到深度相机
	cv::Matx33d Rl2d(Rld);
	cv::Vec3d tl2d(tld.at<double>(0, 0), tld.at<double>(1, 0), tld.at<double>(2, 0));
	cv::Vec3d X_depth = Rl2d * X_left + tl2d;

	if (point_target)
		*point_target = Point3D(X_depth[0], X_depth[1], X_depth[2]);
	return true;
}

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
	Point3D* point_target)
{
	auto lr_R = left_to_right(cv::Rect(0, 0, 3, 3)).clone();
	auto lr_t = left_to_right(cv::Rect(3, 0, 1, 3)).clone();
	auto lt_R = left_to_target(cv::Rect(0, 0, 3, 3)).clone();
	auto lt_t = left_to_target(cv::Rect(3, 0, 1, 3)).clone();
	return TriangulatePointToCamera(left_pixel,
		right_pixel,
		K_left,
		D_left,
		left_model,
		K_right,
		D_right,
		right_model,
		lr_R,
		lr_t,
		lt_R,
		lt_t,
		point_target);
}

CAM_API bool TriangulatePointToCamera(
	const cv::Point2f& left_pixel,
	const cv::Point2f& right_pixel,
	const CameraParameter& camParam,
	Point3D* point_target)
{
	return TriangulatePointToCamera(left_pixel, right_pixel,
		camParam.KLeft, camParam.DLeft, camParam.distModeLeft,
		camParam.KRight, camParam.DRight, camParam.distModeRight,
		camParam.LeftToRight, camParam.LeftToColor,
		point_target);
}

std::vector<std::pair<int, int>> MatchCyclicOrderedPointsByRotation(
	const std::vector<cv::Point2f>& left,
	const std::vector<cv::Point2f>& right,
	double* outRmse)
{
	std::vector<std::pair<int, int>> empty;
	if (left.empty() || right.empty() || left.size() != right.size()) {
		if (outRmse) *outRmse = std::numeric_limits<double>::infinity();
		return empty;
	}

	const int n = static_cast<int>(left.size());

	// 计算 left 和 right 的质心（double 精度）
	double lx = 0.0, ly = 0.0, rx = 0.0, ry = 0.0;
	for (int i = 0; i < n; ++i) {
		lx += static_cast<double>(left[i].x);
		ly += static_cast<double>(left[i].y);
		rx += static_cast<double>(right[i].x);
		ry += static_cast<double>(right[i].y);
	}
	lx /= n; ly /= n; rx /= n; ry /= n;

	// 将 right 平移到与 left 质心对齐
	double dx = lx - rx;
	double dy = ly - ry;
	std::vector<cv::Point2d> Rshifted;
	Rshifted.reserve(n);
	for (int i = 0; i < n; ++i) {
		Rshifted.emplace_back(static_cast<double>(right[i].x) + dx,
			static_cast<double>(right[i].y) + dy);
	}

	// 遍历所有循环位移，选择使欧氏距离和最小的位移
	double bestSum = std::numeric_limits<double>::infinity();
	int bestShift = 0;
	for (int shift = 0; shift < n; ++shift) {
		double sumDist = 0.0;
		for (int i = 0; i < n; ++i) {
			const auto& Lp = left[i];
			const auto& Rp = Rshifted[(i + shift) % n];
			double ex = static_cast<double>(Lp.x) - Rp.x;
			double ey = static_cast<double>(Lp.y) - Rp.y;
			sumDist += std::sqrt(ex * ex + ey * ey); // 使用距离和
		}
		if (sumDist < bestSum) {
			bestSum = sumDist;
			bestShift = shift;
		}
	}

	// 构造映射 left i -> right (i + bestShift) % n
	std::vector<std::pair<int, int>> mapping;
	mapping.reserve(n);
	for (int i = 0; i < n; ++i) {
		mapping.emplace_back(i, (i + bestShift) % n);
	}

	if (outRmse) {
		// 使用平均距离作为误差度量（RMSE 未再计算，返回 mean distance）
		*outRmse = (n > 0) ? (bestSum / n) : std::numeric_limits<double>::infinity();
	}

	return mapping;
}