#pragma once

#include <memory>
#include <string>

#include "cam/depth_camera.h"
#include <opencv2/core/mat.hpp>

class CAM_API PercipioCam : public IDepthCamera {
public:
	PercipioCam() = default;
	~PercipioCam() override;

	bool Open(const CameConfig & = {}) override;
	void Close() override;
	bool Snapshot(int32_t timeout = 3000) override;
	PointCloudPtr GetPointCloud(
		float maxZ = 3000.0,
		float maxX = 1500.0,
		float maxY = 1500.0) override;
	void GetFrame(UNMAP_MAT& frames) const override;
	Point3D BackProject3DFrom2D(
		float left_u,
		float left_v,
		float right_u,
		float right_v,
		const CameraParameter& camParam) override;
	bool GetColorIntrinsics(CameraIntrinsics& out) const override;
	float GetVFOV() override;
	bool SetOption(OPTION_TYPE optionType, float value, int msTimeout = 5000) override;
	float GetPointCloudExposure() override;
	float GetTrackExposure() override;
	bool GetCamParam(CameraParameter& camParam) const override;

private:
	bool initDistMap() const;

private:
	struct Impl;
	std::shared_ptr<Impl> impl_;
};