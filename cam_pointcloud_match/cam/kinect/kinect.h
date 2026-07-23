#pragma once

#include "cam/depth_camera.h"

struct _k4a_device_t;
struct _k4a_transformation_t;
struct _k4a_image_t;

using k4a_device_t = struct _k4a_device_t*;
using k4a_transformation_t = struct _k4a_transformation_t*;
using k4a_image_t = struct _k4a_image_t*;

class CAM_API KinectV2Cam : public IDepthCamera {
public:
	KinectV2Cam() = default;
	~KinectV2Cam() override = default;
	bool Open(const CameConfig & = {}) override;
	void Close() override;
	bool Snapshot(int32_t timeout = 3000) override;
	PointCloudPtr GetPointCloud(
		float maxZ = 3000.0,
		float maxX = 1500.0,
		float maxY = 1500.0) override;
	virtual bool GetPointAtPixel(
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
	k4a_device_t device_ = nullptr;
	k4a_transformation_t transformation_ = nullptr;
	k4a_image_t depth_image_ = nullptr;
	k4a_image_t color_image_ = nullptr;
};