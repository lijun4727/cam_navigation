#pragma once

#include "type.h"

#include <unordered_map>

enum class STREAM_TYPE
{
	VIDEO = 0X0001,
	DEPTH = 0X0002,
	IR = 0X0004,
	LEFT_IR = 0X0008,
	RIGHT_IR = 0X0010,
	ACCEL = 0X00011,
	GYRO = 0X0012,
    CONFIDENCE = 0X0014
};

struct CameConfig {
	int stremType = (int)STREAM_TYPE::VIDEO | (int)STREAM_TYPE::DEPTH;
    //是否打开红外光泛光，针对图漾相机这种有内置红外光泛光的
    bool enableFloodLight = false;
};

enum OPTION_TYPE {
    OPTION_TYPE_IR_EXPOSURE = 0,
    OPTION_TYPE_EMITTER = 1
};

using UNMAP_MAT = std::unordered_map<STREAM_TYPE, Mat>;

struct CameraIntrinsics {
    int width = 0;
    int height = 0;
    double fx = 0.0;
    double fy = 0.0;
    double ppx = 0.0;
    double ppy = 0.0;
};

class CAM_API IDepthCamera {
public:
    virtual ~IDepthCamera() = default;
    virtual bool Open(const CameConfig& camConfig = {}) {
        config_ = camConfig; 
        exit_ = false; 
        return true; 
    }
    virtual bool Snapshot(int32_t timeout = 3000) = 0;
    virtual void Close() { exit_ = true; }
    virtual PointCloudPtr GetPointCloud(
        float maxZ = 3000.0, 
        float maxX = 1500.0, 
        float maxY = 1500.0) = 0;
    virtual bool GetPointAtPixel(
        int x,
        int y,
        Point3D* p = nullptr,
        STREAM_TYPE streamType = STREAM_TYPE::VIDEO) { return false; }
    virtual PointCloudPtr GetPointColudFromPixel(
        const VecPixelCoord& pixelCoords, 
        std::vector<size_t >* failPCIndexes = nullptr,
        VecRGB* rgb = nullptr) { return nullptr; }
    virtual int GetWidth() const { return 0; }
	virtual int GetHeight() const { return 0; }
    virtual void GetFrame(UNMAP_MAT& frames) const = 0; 
	//此函数会多线程调用，要求线程安全
	virtual Point3D BackProject3DFrom2D(
        float left_u,
        float left_v,
        float right_u,
        float right_v,
        const CameraParameter& camParam) { return Point3D::Zero(); }
    virtual bool GetColorIntrinsics(CameraIntrinsics& out) const { return false; }
    virtual float GetVFOV() { return -1.0f; }
    virtual bool SetResolution(STREAM_TYPE streamType, int width, int height) { return false; }
	virtual bool SetOption(OPTION_TYPE optionType, float value, int msTimeout = 3000) { return -1.f; }
	virtual float GetOption(OPTION_TYPE optionType) { return -1.f; }
    //获取点云时的曝光值
    virtual float GetPointCloudExposure() { return 0.f; }
    //获取跟踪时的曝光值
    virtual float GetTrackExposure() { return 0.f; }
    virtual bool GetCamParam(CameraParameter& camParam) const
    { 
        if(!camParam_.LeftToRight.empty()) 
        { 
            camParam = camParam_; 
            return true;
        } 
        return false; 
    }

protected:
    bool exit_ = true;
    CameConfig config_;
	mutable CameraParameter camParam_;
    std::string m_camName;
};
