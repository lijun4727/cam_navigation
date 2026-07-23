#include "CameraPump.h"
#include "cam/real_sense/real_sense.h"
#include "cam/percipio/percipio.h"
#include "common/math/opencv_kit.h"
#include "common/WorkPump.h"
#include "common/math/math_kit.h"
#include "common/common_kit.h"
#include "common//math/open3d_kit.h"

#include <QThread>
#include <QElapsedTimer>
#include <QDebug>

CameraPump::CameraPump(QObject * parent)
    : QObject(parent)
    , m_timer(nullptr)
    , m_camera(nullptr)
{
    //cv::Mat leftIRFrame = cv::imread("D:\\left.jpg", cv::IMREAD_COLOR);
    //cv::Mat rightIRFrame = cv::imread("D:\\right.jpg", cv::IMREAD_COLOR);
    //std::vector<SpotInfo> leftSpots;
    //std::vector<SpotInfo> rightSpots;
    //DetectWhiteSpotsFromMore(leftIRFrame, rightIRFrame, leftSpots, rightSpots, 1, 50, 200);

    //auto img = cv::imread("D:\\test.jpg", cv::IMREAD_COLOR);
    //auto candidates1 = DetectWhiteSpots(img);
    //auto drawImage1 = DrawSpots(img, candidates1);

    auto exeDir = GetExecutableDirectory();
    m_probeMesh = LoadSTLMesh(exeDir + "\\meshes\\probe.stl", false);
    m_guideMesh = LoadSTLMesh(exeDir + "\\meshes\\guide.stl", false);

    /*
    -79.6985 -4.14776 989.441 0 255 0
    -22.035 13.7316 992.746 0 255 0
    -4.78799 -48.2125 966.232 0 255 0
    -83.3027 -42.6236 970.34 0 255 0
    */
    m_baseModel.type = OT_BASE;
    m_baseModel.cloud = std::make_shared<PointCloud>();
    m_baseModel.cloud->points_ = {
        Point3D(-79.6985, -4.14776, 989.441),
        Point3D(-22.035, 13.7316, 992.746),
        Point3D(-4.78799, -48.2125, 966.232),
        Point3D(-83.3027, -42.6236, 970.34)
    };
    ArrangePointCloud(m_baseModel.cloud);
    MakaGradient(m_baseModel.cloud, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 });

    m_guideModel.type = OT_GUIDE;
    m_guideModel.cloud = std::make_shared<PointCloud>();
    m_guideModel.cloud->points_ = {
        m_guideMesh->vertices_[66518],
        m_guideMesh->vertices_[67482],
        m_guideMesh->vertices_[67964],
        m_guideMesh->vertices_[67000]
    };
    ArrangePointCloud(m_guideModel.cloud);
    MakaGradient(m_guideModel.cloud, { 0.0, 1.0, 0.0 }, { 1.0, 0.0, 0.0 });

    m_probeModel.type = OT_PROBE;
    m_probeModel.cloud = std::make_shared<PointCloud>();
    m_probeModel.cloud->points_ = {
        m_probeMesh->vertices_[51247],
        m_probeMesh->vertices_[50765],
        m_probeMesh->vertices_[52211],
        m_probeMesh->vertices_[51729]
    };
    ArrangePointCloud(m_probeModel.cloud);
    MakaGradient(m_probeModel.cloud, { 1.0, 0.0, 0.0 }, { 0.0, 0.0, 1.0 });

	m_measProbeModel.type = OT_MEASURE_PROBE;
	m_measProbeModel.cloud = std::make_shared<PointCloud>();
	m_measProbeModel.cloud->points_ = {
		Point3D(-50.7735, 156.7913, 12.606),     //左
		Point3D(0.3404, 184.076, 12.5695),       //上
		Point3D(54.0283, 130.6269, 12.6125),     //右
		Point3D(0.2884, 89.0902, 12.6538),       //下
	};
    ArrangePointCloud(m_measProbeModel.cloud);
    MakaGradient(m_measProbeModel.cloud, { 1.0, 1.0, 1.0 }, { 0.0, 0.0, 0.0 });

    m_measCoordModel.type = OT_MEASURE_COORD;
    m_measCoordModel.cloud = std::make_shared<PointCloud>();
    m_measCoordModel.cloud->points_ = {
        Point3D(-59.9029398, -3.929834, 37.0509663),    //48
        Point3D(-78.0805659, -28.2557914, -11.65197),   //51
		Point3D(-25.6829324, -83.0519707, -15.5541816), //50
        Point3D(4.8984608, -62.6279896, 42.5949767),    //49

    };
    ArrangePointCloud(m_measCoordModel.cloud);
    MakaGradient(m_measCoordModel.cloud, { 1.0, 1.0, 1.0 }, { 0.5, 0.5, 0.5 });

}

CameraPump::~CameraPump()
{
}

void CameraPump::Start(int intervalMs)
{
    if (QThread::currentThread() == this->thread()) {
        start(intervalMs);
    }
    else {
        QMetaObject::invokeMethod(this, "start", Qt::QueuedConnection,
            Q_ARG(int, intervalMs));
    }
}

void CameraPump::Stop()
{ 
    if (QThread::currentThread() == this->thread()) {
        stop();
    }
    else {
        QMetaObject::invokeMethod(this, "stop", Qt::BlockingQueuedConnection);
    }
}

void CameraPump::GrabPointCloud()
{
    if (QThread::currentThread() == this->thread()) {
        grabPointCloud();
    }
    else {
        QMetaObject::invokeMethod(this, "grabPointCloud", Qt::QueuedConnection);
    }
}

float CameraPump::GetVFOV() const
{
    return m_vfov;
}

void CameraPump::StartTrack()
{
    if (QThread::currentThread() == this->thread()) {
        startTrack();
    }
    else {
        QMetaObject::invokeMethod(this, "startTrack", Qt::QueuedConnection);
    }
}

void CameraPump::StopTrack()
{
    if (QThread::currentThread() == this->thread()) {
        stopTrack();
    }
    else {
        QMetaObject::invokeMethod(this, "stopTrack", Qt::QueuedConnection);
    }
}

bool CameraPump::IsRunning() const
{
    bool active = false;
    QMetaObject::invokeMethod(m_timer, [&] { active = m_timer->isActive(); }, 
        Qt::BlockingQueuedConnection);
	return active;
}

bool CameraPump::GetColorIntrinsics(CameraIntrinsics& out) const
{
    if (!m_intrinsicsValid)
        return false;
    out = m_intrinsics;
    return true;
}

void CameraPump::start(int intervalMs)
{
    if (!m_camera) {
        m_camera = new PercipioCam();
		auto ret = m_camera->Open(CameConfig{
		(int)STREAM_TYPE::VIDEO |
		(int)STREAM_TYPE::DEPTH /*|
		(int)STREAM_TYPE::LEFT_IR |
		(int)STREAM_TYPE::RIGHT_IR */}
		);
        if(ret) {
            m_vfov = m_camera->GetVFOV();
            m_intrinsicsValid = m_camera->GetColorIntrinsics(m_intrinsics);
        }
        else {
			return;
        }
    }

    if (!m_workPump)
    {
        m_workPump = std::make_unique<WorkPump>();
        m_workPump->startPump();
    }

    if (m_timer) return;
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &CameraPump::grabFrame, Qt::DirectConnection);
    m_timer->start(intervalMs > 0 ? intervalMs : 33);
}

void CameraPump::stop()
{
    if (m_workPump)
    {
		m_workPump->stopPump();
		m_workPump.reset();
    }

    if (m_timer) {
        m_timer->stop();
        m_timer->deleteLater();
        m_timer = nullptr;
    }

    if (m_camera) {
        m_camera->Close();
        delete m_camera;
		m_camera = nullptr;
    }
}

void CameraPump::grabPointCloud()
{
    auto exposure = m_camera->GetPointCloudExposure();
    if (!m_camera || 
        !m_camera->SetOption(OPTION_TYPE_IR_EXPOSURE, exposure) ||
        !m_camera->SetOption(OPTION_TYPE_EMITTER, 1) ||
        !m_camera->Snapshot())
        return;

     PointCloudPtr pc = m_camera->GetPointCloud(2000, 200, 200);
     if (pc) {
         emit pointCloudReady(pc);
	 }
}

void CameraPump::startTrack()
{
    auto trackExposure = m_camera->GetTrackExposure();
    if (!m_camera || 
        !m_camera->SetOption(OPTION_TYPE_EMITTER, 0) ||
        !m_camera->SetOption(OPTION_TYPE_IR_EXPOSURE, trackExposure))
        return;
	m_isTack = true;
}

void CameraPump::stopTrack()
{
    auto exposure = m_camera->GetPointCloudExposure();
    m_isTack = false;
    m_camera->SetOption(OPTION_TYPE_EMITTER, 1);
    m_camera->SetOption(OPTION_TYPE_IR_EXPOSURE, exposure);
}

void CameraPump::grabFrame()
{
//#ifdef DEBUG_LOG
//    const auto _start = std::chrono::steady_clock::now();
//#endif // DEBUG_LOG

    if (!m_camera || !m_camera->Snapshot()) {
        qInfo() << "grabFrame return\n";
        return;
    }       

    UNMAP_MAT frames{
        {STREAM_TYPE::LEFT_IR, {}},
        {STREAM_TYPE::RIGHT_IR, {}},
        {STREAM_TYPE::VIDEO, {}}
    };
    m_camera->GetFrame(frames);

    //激活计算线程
    //将相机参数和像素点传递给计算线程
    if(m_isTack) {
        do {
            if(!m_workPump->isQueueEmpty()){
                break;
			}

            CameraParameter camParam;
            if (!m_camera->GetCamParam(camParam)) {
                break;
            }

            auto& leftIRFrame = frames[STREAM_TYPE::LEFT_IR];
            auto& rightIRFrame = frames[STREAM_TYPE::RIGHT_IR];          
                           
			auto hardFun = [this](
                const Mat& leftIRFrame,
				const Mat& rightIRFrame,
				const CameraParameter& camParam)
				{
                    std::vector<SpotInfo> leftSpots;
                    std::vector<SpotInfo> rightSpots;
                    DetectWhiteSpotsFromMore(
                        leftIRFrame, 
                        rightIRFrame, 
                        leftSpots,
                        rightSpots, 
                        1, 
                        50,
                        200);
                    if (leftSpots.empty() ||
                        rightSpots.empty() ||
                        leftSpots.size() > MAX_SPOT_NUM ||
                        rightSpots.size() > MAX_SPOT_NUM ||
                        leftSpots.size() != rightSpots.size())
                    {
						emit trackReady(nullptr, {});
                        return;
                    }

                    PointCloud pointCloud;
                    for (size_t i = 0; i < leftSpots.size(); ++i)
                    {
                        auto P = this->m_camera->BackProject3DFrom2D(
                            leftSpots[i].center.x,
                            leftSpots[i].center.y,
                            rightSpots[i].center.x,
                            rightSpots[i].center.y,
                            camParam);
                        if (!IsValidPoint3D(P)) {
                            continue;
                        }

                        //看到的是球的表面顶点，将球心就行返回
                        //constexpr auto BALL_RADIUS = 5.5f; // mm
                        //auto ballCenterVec = P.normalized() * BALL_RADIUS;
                        //P += ballCenterVec;
                        pointCloud.points_.emplace_back(P);
                        pointCloud.colors_.emplace_back(Eigen::Vector3d(0.0, 1.0, 0.0));
                    }

                    if (pointCloud.HasPoints()) {
                        PointCloudPtr trackedCloud = std::make_shared<open3d::geometry::PointCloud>(pointCloud);

						OBJECT_UNMAP models = {
							{ OT_BASE, m_baseModel.cloud },
							{ OT_PROBE, m_probeModel.cloud },
							{ OT_GUIDE, m_guideModel.cloud},
							{ OT_MEASURE_PROBE, m_measProbeModel.cloud },
                            { OT_MEASURE_COORD, m_measCoordModel.cloud }
						};
                        auto objects = CalculateObject3(trackedCloud, models);
						emit trackReady(trackedCloud, objects);
                    }
                    else
                    {
						emit trackReady(nullptr, {});
                    }
				};
            m_workPump->postTask(hardFun, leftIRFrame, rightIRFrame, camParam);
        } while (false);        
	}
    emit frameReady(
        frames[STREAM_TYPE::VIDEO],  
        frames[STREAM_TYPE::DEPTH],
        frames[STREAM_TYPE::LEFT_IR],
        frames[STREAM_TYPE::RIGHT_IR]);
//#ifdef DEBUG_LOG
//    const auto _end = std::chrono::steady_clock::now();
//    const auto _ms = std::chrono::duration_cast<std::chrono::milliseconds>(_end - _start).count();
//    qInfo() << "程序运行时长(ms):" << _ms;
//#endif // DEBUG_LOG
}