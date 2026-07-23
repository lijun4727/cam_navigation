#pragma once

#include <memory>
#include "cam/depth_camera.h"

namespace ob {
	class Pipeline;
	class PointCloudFilter;
	class Align;
	class Frame;
	class FrameSet;
}

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

class CAM_API OrbbecvCam : public IDepthCamera {
public:
	OrbbecvCam();
	~OrbbecvCam() override = default;
	bool Open(const CameConfig& camConfig = {}) override;
	void Close() override;
	bool Snapshot(int32_t timeout = 3000) override;
	PointCloudPtr GetPointCloud(
		float maxZ = 3000.0,
		float maxX = 1500.0,
		float maxY = 1500.0) override;
	bool GetPointAtPixel(
		int x,
		int y,
		Point3D* p = nullptr,
		STREAM_TYPE streamType = STREAM_TYPE::VIDEO) override;
	PointCloudPtr GetPointColudFromPixel(
		const VecPixelCoord& pixelCoords,
		std::vector<size_t >* failPCIndexes = nullptr,
		VecRGB* rgb = nullptr) override;
	int GetWidth() const override;
	int GetHeight() const override;
	void GetFrame(UNMAP_MAT& frames) const override;

private:
	PointCloudPtr OrbbecToOpen3D(
		std::shared_ptr<ob::Frame> orbbecFrame,
		float maxZ = 3000.0,
		float maxX = 1500.0,
		float maxY = 1500.0);

private:
	std::shared_ptr<ob::Pipeline> pipeline_;
	std::shared_ptr<ob::FrameSet> frameSet_;
	std::shared_ptr<ob::PointCloudFilter> pointCloudFilter_;
	std::shared_ptr<ob::Align> align_;
};

#ifdef _MSC_VER
#pragma warning(pop)
#endif