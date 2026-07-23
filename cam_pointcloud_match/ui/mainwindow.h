#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include "type.h"
#include "business/business.h"

#include <vtkImageImport.h>

#include <memory>

QT_BEGIN_NAMESPACE
namespace Ui {
    class MainWindow;
}
QT_END_NAMESPACE

class QThread;
class CameraPump;
class vtkPointPicker;
class vtkCellPicker;
class vtkTextActor;
class vtkCommand;
class FileTransfer;

// 前向声明 VTK 回调类（定义在 .cpp 中）
class VtkMousePickCallback;
class VtkRenderSyncCallback;

class MainWindow : public QMainWindow
{
    Q_OBJECT

    // 回调类需要访问私有成员
    friend class VtkMousePickCallback;
    friend class VtkRenderSyncCallback;

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void OnFrameReady(
        const cv::Mat& videoFrame, 
        const cv::Mat& depthFrame,
        const cv::Mat& leftIRFrame,
        const cv::Mat& rightIRFrame);
    void OnPointCloudReady(const PointCloudPtr& pointCloud);
	void OnTrackReady(const PointCloudPtr& trackCloud, const OBJECT_UNMAP& trackPointCloud);

    void on_btnLoadMesh_clicked();
    void on_btnOpenCam_clicked();
    void on_btnGetPointCloud_clicked();
    void on_btnRegiste_clicked();
    void on_btnTrack_toggled(bool checked);
    void on_btnShowPointCloud_toggled(bool checked);
    void on_btnSetBase_clicked();
    void on_btnConnectServer_clicked();
    void on_btnTravel_clicked();
    void on_btnMeasProbe_clicked();
    void on_btnOutError_clicked();
    void on_btnResetMeas_clicked();
    void on_btnDelBallErorr_clicked();

private:
    // 每个选点的记录
    struct PickRecord {
        vtkIdType pointId = -1;                      // 点云中的点索引
        double worldPos[3] = { 0.0, 0.0, 0.0 };     // 世界坐标
        vtkSmartPointer<vtkActor2D> markerActor;     // 2D 圆点标记
        vtkSmartPointer<vtkTextActor> labelActor;    // 数字标签 actor
    };

private:
    void startCameraLoop(int intervalMs = 33);
    void stopCameraLoop();
    void updateBackgroundRenderCam();
	void updateTravelRenderCam(const PointCloudPtr& probePoint);

    void InitUI();                                                      // 初始化UI界面

    void updateRegisterResult(const
        FineAlignmentResult& result, qint64 elapsedMs);                 // 更新配准结果

    // 选点交互
    void setupPointPicking();                                           // 初始化选点
    void onLeftMousePick(int x, int y);                                 // 左键双击入口
    void onRightMouseUndo(int x, int y);                                // 右键双击入口
    void tryPickPoint(vtkRenderer* renderer, int x, int y);             // 用 Picker 选点
    void undoPickedMarker(vtkRenderer* renderer, int x, int y);         // 撤销指定标记
    void renumberPickLabels(vtkRenderer* renderer);                     // 重编号
    void addPickMarker(
        vtkRenderer* renderer, 
        vtkIdType pointId,
        const double worldPos[3]);                                      // 添加标记
    void clearRendererPicks(vtkRenderer* renderer);                     // 清空标记
    void updateLabelPositions(vtkRenderer* renderer);                   // 更新标签位置/大小
    void GetPickedPointIndices(
        std::vector<int>& leftIndices,
        std::vector<int>& rightIndices) const;
    void SetLayerVisible(int layerIndex, bool visible);
    void UpdateBackgroundMesh(const TriangleMeshPtr& mesh);
	void UpdateBase(const PointCloudPtr& basePointCloud);
    void ShowTrackedMesh(
        const TriangleMeshPtr& mesh, 
        const PointCloudPtr& modelPointClod,
        const PointCloudPtr& trackPointCloud,
        ActorPtr& actor);

#pragma region 测量精度相关
	void ClearTrackActor();
	void UpdateMeasureProbe(const PointCloudPtr& trackPointCloud);
    void UpdateMeasureCoord(const PointCloudPtr& trackPointCloud);
    void CalMeasurePoint(int ballNum);

    //测量器探针的Actor
    ActorPtr m_measProbeActor;
    //测量器坐标系的Actor
    ActorPtr m_measCoordActor;
    //测量精度的球信息
    struct MeasBall
    {
        Point3D realPoint;                       // 实际球心坐标,以工装坐标系为基准
        std::vector<Point3D> measPoints;         // 测量球心坐标,以工装坐标系为基准
        int ballNum;                             // 球编号
        double minError;
        double maxError;
        double avgError;
        std::vector<double> errors;               // 每次测量的误差
    };
    std::unordered_map<int, MeasBall> m_measBalls; // 球编号到球信息的映射
    Eigen::Matrix4d m_measToRealT;
#pragma endregion

private:
    Ui::MainWindow* ui;
    TriangleMeshPtr m_mesh;
    //探针的Mesh
    //TriangleMeshPtr m_probeMesh;
	//引导器的Mesh
    //TriangleMeshPtr m_guideMesh;
    PointCloudPtr m_camPointCloud;
	PointCloudPtr m_track;

    // layer 0：共享背景（全屏）
    RenderPtr m_backgroundRender;
    // layer 1：左右背景
    RenderPtr m_leftRenderBK;
    RenderPtr m_rightRenderBK;
    vtkSmartPointer<vtkActor2D> m_leftMaskActor;
    vtkSmartPointer<vtkActor2D> m_rightMaskActor;
    // layer 2：左右前景（3D）
    RenderPtr m_leftRender;
    RenderPtr m_rightRender;

    // 背景图像
    vtkSmartPointer<vtkImageImport> m_imageImport;
    vtkSmartPointer<vtkTexture> m_backgroundTexture;
    std::vector<unsigned char> m_imageBuffer;

    //相机点云actor
	ActorPtr m_camPointCloudActor;
    //原始Mesh的Actor
	ActorPtr m_rightRenderMeshActor;
    //配准后的Mesh的Actor
	ActorPtr m_meshRegisterActor;
    //跟踪的点的Actor
	ActorPtr m_trackActor;
	//基座点的Actor
    ActorPtr m_baseActor;
	//探针的Actor
    ActorPtr m_probeActor;
    //引导器的Actor
    ActorPtr m_guideActor;
    //探针顶点的Actor
    ActorPtr m_probePointActor;

    // 选点相关
    vtkSmartPointer<vtkCellPicker> m_cellPicker;
    std::vector<PickRecord> m_leftPickRecords;
    std::vector<PickRecord> m_rightPickRecords;

    // 渲染同步回调（跟踪相机变化以更新标签）
    vtkSmartPointer<vtkCommand> m_leftRenderSyncCallback;
    vtkSmartPointer<vtkCommand> m_rightRenderSyncCallback;

    // 摄像头线程与抓帧器
    QThread* m_camThread;
    CameraPump* m_cameraPump;

    double m_bkScale = 1.0;
    int m_bkX = 0;
    int m_bkY = 0;

    // 防止 updateLabelPositions 与 EndEvent 回调形成无限递归
    bool m_updatingLabels = false;

    //基座点
	std::vector<PointCloudPtr> m_oldBase;
    std::vector<PointCloudPtr> m_newBase;
    ActorPtr m_oldBaseActor;
    ActorPtr m_newBaseActor;
    //相机是否被移动了
    bool m_camIsMoved = false;
	//是否开启自动观察相机移动的功能
	bool m_isAutoObserveCamMove = false;
	//自动观察相机移动的开始时间（ms）
    std::chrono::steady_clock::time_point m_autoObserveCamMoveStart;
	//自动观察相机移动的延迟时间（ms），如果相机在这个时间内没有移动，
    //就认为相机没有移动了
	const long long m_autoObserveCamMoveDelay = 500;
	//自动观察相机移动的最小移动距离的平方，如果相机移动的距离小于这个
    //值，就认为相机没有移动了
	const double m_autoObserveCamMoveMinDist = 10.0;
    Point3D m_autoObserveLastCenter = InvalidPoint3D();

    //渲染背景时是否需要调用ResetCameraClippingRange
    bool m_backgroundSceneDirty = true;

	std::unique_ptr<FileTransfer> m_fileTransfer;
};

#endif // MAINWINDOW_H
