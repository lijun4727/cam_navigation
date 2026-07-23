#pragma once

#include <QObject>
#include <QTimer>
#include <memory>

#include "cam/depth_camera.h"
#include <opencv2/core/mat.hpp>
#include "business/business.h"

class IDepthCamera;
class WorkPump;

class CameraPump : public QObject
{
    Q_OBJECT
public:
    explicit CameraPump(QObject* parent = nullptr);
    ~CameraPump() override;
    void Start(int intervalMs = 17);
	void Stop();
    void GrabPointCloud();
    float GetVFOV() const;
    void StartTrack();
	void StopTrack();
    bool IsRunning() const;	

    // 新增：获取彩色相机内参
    bool GetColorIntrinsics(CameraIntrinsics& out) const;

signals:
    // 抓到帧后发出（建议在主线程以 QueuedConnection 连接）
    void frameReady(
        const cv::Mat& videoFrame, 
		const cv::Mat& depthFrame,
		const cv::Mat& leftIRFrame,
		const cv::Mat& rightIRFrame);
	void pointCloudReady(const PointCloudPtr& pointCloud);
	void trackReady(const PointCloudPtr& trackCloud, const OBJECT_UNMAP& trackPointCloud);

private slots:
    // 定时器触发抓帧
    void grabFrame();
    // 在所属线程中启动定时抓帧（intervalMs 毫秒）
    void start(int intervalMs);
    // 停止抓帧
    void stop();
	//获取当前帧的点云数据
	void grabPointCloud();
	//跟踪踪迹
    void startTrack();
	void stopTrack();

private:
    IDepthCamera* m_camera;
    QTimer* m_timer;
	float m_vfov = 0.f;

    CameraIntrinsics m_intrinsics;
    bool m_intrinsicsValid = false;

    bool m_isTack = false;

    std::unique_ptr<WorkPump> m_workPump;

    //探针的Mesh
    TriangleMeshPtr m_probeMesh;
    //引导器的Mesh
    TriangleMeshPtr m_guideMesh;
    //基座模型
    OBJECT_MODEL_INFO m_baseModel;
    //探针模型
    OBJECT_MODEL_INFO m_probeModel;
    //引导器模型
    OBJECT_MODEL_INFO m_guideModel;

#pragma region 测量精度相关
    //测量器模型
	OBJECT_MODEL_INFO m_measProbeModel;
    //测量器坐标模型
    OBJECT_MODEL_INFO m_measCoordModel;

    //艾目易精度测量工装的各个球的坐标
	std::vector<Point3D> m_measBallsPoints = {
        Point3D(),
		Point3D(0, 0, 0.0029267),
		Point3D(0.0084674, 14.4392311, 0.0867141),
		Point3D(14.4781134, -0.0020573, 0.0624109),
		Point3D(0.0023544, 28.869548, 0.1008206),
		Point3D(28.9189513, -0.009712, 0.0619179),
		Point3D(0.000574, 43.3179467, 0.0829292),
        Point3D(43.3534512, -0.0064024, 0.0664068),
        Point3D(-0.0015456, 57.754507, 0.0912968),
        Point3D(57.796893, -0.0049598, 0.090238),
        Point3D(0.0050891, 72.2016061, 0.0911234), //10
        Point3D(72.2330189, -0.0019506, 0.1026063), 
        Point3D(-0.0042545, 86.6462545, 0.0734792),
        Point3D(86.6680012, 0.0048919, 0.1092549),
        Point3D(0.0016949, 101.0792589, 0.1045568),
        Point3D(101.0917567, -0.000205, 0.1013658),
        Point3D(-0.0060864, 115.5238597, 0.0661647),
        Point3D(115.5046522, 0.0109691, 0.0896796),
        Point3D(-0.0048668, 129.9630354, 0.0988916),
        Point3D(129.9282106, 0.0067279, 0.0981517),
        Point3D(32.5310817, 32.4816686, 0.0945884), //20
        Point3D(16.2331679, 113.7557617, 0.1272057), 
        Point3D(32.4624773, 97.5109494, 0.0894254),
        Point3D(48.7138981, 81.2543136, 0.0805808), 
        Point3D(64.9569459, 65.0124442, 0.0913743),
        Point3D(81.2148267, 48.7615814, 0.104676),
        Point3D(97.4618307, 32.5091876, 0.1011396),
        Point3D(113.7400407, 16.2590513, 0.1300758),
        Point3D(35.36, 129.9682487, 10.1239299),
        Point3D(59.3289892, 115.8761751, 10.1233648),
        Point3D(68.524756, 96.7904965, 10.0940477), //30
        Point3D(87.6231516, 87.6227557, 15.0871041), 
	};
#pragma endregion

	friend class MainWindow;
};