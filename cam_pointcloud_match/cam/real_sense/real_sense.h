#pragma once

#include <memory>

#include "cam/depth_camera.h"

namespace rs2 {
	class pipeline;
	class frame;
	class video_frame;
	class depth_frame;
	class frameset;
}

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

class CAM_API RealSenseCam : public IDepthCamera {
public:
	RealSenseCam() = default;
	~RealSenseCam() override;

	bool Open(const CameConfig & = {}) override;
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
	Point3D BackProject3DFrom2D(
		float left_u,
		float left_v,
		float right_u,
		float right_v,
		const CameraParameter& camParam) override;
	// 新增：彩色相机内参
	bool GetColorIntrinsics(CameraIntrinsics& out) const override;
	float GetVFOV() override;
	bool SetResolution(STREAM_TYPE streamType, int width, int height) override;
	bool SetOption(OPTION_TYPE optionType, float value, int msTimeout = 5000) override;
	float GetOption(OPTION_TYPE optionType) override;
	float GetPointCloudExposure() override;
	float GetTrackExposure() override;
	bool GetCamParam(CameraParameter& camParam) const override;
		
private:
	bool GetPointAtPixel(
		const rs2::depth_frame& depth,
		const rs2::video_frame& frame,
		int x,
		int y,
		Point3D* p,
		float depth_min = 1.0f,
		float depth_max = 10000.0f);
	Mat VideoFrameToMat(const rs2::video_frame& frame) const;

private:
	std::shared_ptr<rs2::pipeline> pipe_;
	std::shared_ptr<rs2::frameset> frameSet_;
};

#ifdef _MSC_VER
#pragma warning(pop)
#endif