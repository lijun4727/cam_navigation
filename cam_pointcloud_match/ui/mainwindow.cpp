#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "type.h"
#include "common/math/open3d_kit.h"
#include "common/common_kit.h"
#include "business/business.h"
#include "business/CameraPump.h"
#include "common/math/math_kit.h"
#include "business/FileTransfer.h"

#include <QFileDialog>
#include <QThread>
#include <QDebug>
#include <QMessageBox>
#include <QUuid>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkNew.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkImageImport.h>
#include <vtkTexture.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkActor2D.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkProperty2D.h>
#include <vtkCommand.h>
#include <vtkPointPicker.h>
#include <vtkCellPicker.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkTextProperty.h>
#include <vtkRegularPolygonSource.h>
#include <vtkTextActor.h>
#include <vtkOutputWindow.h>
#include <vtkRendererCollection.h>
#include <vtkMatrix4x4.h>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <iostream>

constexpr double camDefaultZ = 1500.0;

// ===========================
// VTK 回调类：鼠标双击事件
// 左键双击选点，右键双击撤销点击位置附近的标记球
// ===========================
class VtkMousePickCallback : public vtkCommand
{
public:
    static VtkMousePickCallback* New() { return new VtkMousePickCallback(); }
    void SetOwner(MainWindow* owner) { m_owner = owner; }

    void Execute(vtkObject* caller, unsigned long eventId, void* callData) override
    {
        (void)callData;
        if (!m_owner) return;

        auto* interactor = vtkRenderWindowInteractor::SafeDownCast(caller);
        if (!interactor) return;

        int pos[2] = { 0, 0 };
        interactor->GetEventPosition(pos);

        if (eventId == vtkCommand::LeftButtonDoubleClickEvent) {
            m_owner->onLeftMousePick(pos[0], pos[1]);
        }
        else if (eventId == vtkCommand::RightButtonDoubleClickEvent) {
            m_owner->onRightMouseUndo(pos[0], pos[1]);
        }
    }

private:
    MainWindow* m_owner = nullptr;
};

// ===========================
// VTK 回调类：滚轮缩放后限制相机最小距离
// 防止相机无限逼近焦点导致旋转操作卡顿
// ===========================
class VtkWheelClampCallback : public vtkCommand
{
public:
    static VtkWheelClampCallback* New() { return new VtkWheelClampCallback(); }

    void Execute(vtkObject* caller, unsigned long eventId, void* callData) override
    {
        (void)callData;
        auto* interactor = vtkRenderWindowInteractor::SafeDownCast(caller);
        if (!interactor) return;

        // 获取当前交互的 renderer
        int pos[2] = { 0, 0 };
        interactor->GetEventPosition(pos);
        auto* renderer = interactor->FindPokedRenderer(pos[0], pos[1]);
        if (!renderer) return;

        auto* cam = renderer->GetActiveCamera();
        if (!cam) return;

        double fp[3], cp[3];
        cam->GetFocalPoint(fp);
        cam->GetPosition(cp);

        double dx = cp[0] - fp[0];
        double dy = cp[1] - fp[1];
        double dz = cp[2] - fp[2];
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        // 根据场景对角线计算最小距离（对角线的 1%）
        double bounds[6];
        renderer->ComputeVisiblePropBounds(bounds);
        double bx = bounds[1] - bounds[0];
        double by = bounds[3] - bounds[2];
        double bz = bounds[5] - bounds[4];
        double diag = std::sqrt(std::max(0.0, bx * bx + by * by + bz * bz));
        double minDist = std::max(0.1, diag * 0.01);

        if (dist < minDist && dist > 1e-12) {
            // 将相机拉回到最小距离处，保持方向不变
            double scale = minDist / dist;
            cam->SetPosition(
                fp[0] + dx * scale,
                fp[1] + dy * scale,
                fp[2] + dz * scale);
        }

        renderer->ResetCameraClippingRange();
    }
};

// ===========================
// VTK 回调类：渲染结束事件
// 每次渲染后更新数字标签的位置和大小，保证跟随相机变化
// ===========================
class VtkRenderSyncCallback : public vtkCommand
{
public:
    static VtkRenderSyncCallback* New() { return new VtkRenderSyncCallback(); }
    void SetOwner(MainWindow* owner) { m_owner = owner; }

    void Execute(vtkObject* caller, unsigned long eventId, void* callData) override
    {
        (void)eventId;
        (void)callData;
        if (!m_owner) return;

        auto* renderer = vtkRenderer::SafeDownCast(caller);
        if (!renderer) return;

        m_owner->updateLabelPositions(renderer);
    }

private:
    MainWindow* m_owner = nullptr;
};

void MainWindow::InitUI()
{
    ui->btnWidget->setObjectName("btnContainer");
    ui->btnWidget->setStyleSheet("#btnContainer { background-color: rgba(0, 0, 0, 140); }");
    ui->statusWidget->setObjectName("statusContainer");
    ui->statusWidget->setStyleSheet("#statusContainer { background-color: rgba(0, 0, 0, 140); }");
    ui->registerResultWidget->setObjectName("registerResultContainer");
    ui->registerResultWidget->setStyleSheet("#registerResultContainer { background-color: rgba(0, 0, 0, 140); }");

    ui->btnOpenCam->setIcon(QIcon(":/png/openCam.png"));
    ui->btnScreenshot->setIcon(QIcon(":/png/Screenshot.png"));
    ui->btnVideo->setIcon(QIcon(":/png/Video.png"));
    ui->btnSave->setIcon(QIcon(":/png/Save.png"));
    ui->btnGetPointCloud->setIcon(QIcon(":/png/Scan.png"));

    ui->btnOpenCam->setStyleSheet(R"(
        QPushButton {
            background-color: #177DDC;
        }
        QPushButton:hover{
            background-color: #0000DC;
        }
    )");

    ui->btnGetPointCloud->setStyleSheet(R"(
        QPushButton {
            background-color: #177DDC;
        }
        QPushButton:hover{
            background-color: #0000DC;
        }
    )");

    ui->btnRegiste->setStyleSheet(R"(
        QPushButton {
            background-color: #177DDC;
        }
        QPushButton:hover{
            background-color: #0000DC;
        }
    )");

    ui->btnOpenCam->setCheckable(true);

    //ui->btnRegiste->hide();
    //ui->registerResultWidget->hide();
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_camThread(nullptr)
    , m_cameraPump(nullptr)
{
    ui->setupUi(this);


    // 禁用 VTK 弹窗，避免 VTK 错误窗口导致 UI 卡死
    vtkOutputWindow::SetGlobalWarningDisplay(0);

    qRegisterMetaType<cv::Mat>("cv::Mat");
    this->showMaximized();

    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    ui->vtkWidget->setRenderWindow(renderWindow);

    // 初始化UI界面
    InitUI();

    // 0: 背景图层, 1: 遮罩层, 2: 前景点云层
    renderWindow->SetNumberOfLayers(3);
    renderWindow->SetAlphaBitPlanes(1);
    renderWindow->SetMultiSamples(0);

    // layer 0 背景
    m_backgroundRender = vtkSmartPointer<vtkRenderer>::New();
    m_backgroundRender->SetLayer(0);
    m_backgroundRender->SetViewport(0.0, 0.0, 1.0, 1.0);
    m_backgroundRender->SetTexturedBackground(true);
    m_backgroundRender->InteractiveOff();
    m_backgroundRender->SetUseDepthPeeling(true);
    m_backgroundRender->SetMaximumNumberOfPeels(4);   // 集成显卡保守值
    m_backgroundRender->SetOcclusionRatio(0.5);       // 容忍较大误差以提高性能
    m_backgroundRender->SetBackground(0.2, 0.2, 0.2); // 设置背景为灰色
    renderWindow->AddRenderer(m_backgroundRender);

    // layer 2 左前景点云
    m_leftRender = vtkSmartPointer<vtkRenderer>::New();
    m_leftRender->SetLayer(2);
    m_leftRender->SetViewport(0.1, 0.1, 0.45, 0.9);
    m_leftRender->SetPreserveColorBuffer(true);
    m_leftRender->SetErase(true);
    m_leftRender->InteractiveOn();
    m_leftRender->SetDraw(false);
    renderWindow->AddRenderer(m_leftRender);

    // layer 2 右前景点云
    m_rightRender = vtkSmartPointer<vtkRenderer>::New();
    m_rightRender->SetLayer(2);
    m_rightRender->SetViewport(0.55, 0.1, 0.9, 0.9);
    m_rightRender->SetPreserveColorBuffer(true);
    m_rightRender->SetErase(true);
    m_rightRender->InteractiveOn();
    m_rightRender->SetDraw(false);
    renderWindow->AddRenderer(m_rightRender);

    // layer 1 左遮罩层（不清屏，仅叠加2D半透明黑）
    m_leftRenderBK = vtkSmartPointer<vtkRenderer>::New();
    m_leftRenderBK->SetLayer(1);
    m_leftRenderBK->SetViewport(0.1, 0.1, 0.45, 0.9);
    m_leftRenderBK->SetPreserveColorBuffer(true);
    m_leftRenderBK->SetErase(true);
    m_leftRenderBK->InteractiveOff();
    m_leftRenderBK->SetDraw(false);
    renderWindow->AddRenderer(m_leftRenderBK);

	auto leftCam = m_leftRender->GetActiveCamera();
	leftCam->SetPosition(0.0, 0.0, 0.0);
	leftCam->SetViewUp(0.0, 1.0, 0.0);
	leftCam->SetFocalPoint(0.0, 0.0, camDefaultZ);

    // layer 1 右遮罩层
    m_rightRenderBK = vtkSmartPointer<vtkRenderer>::New();
    m_rightRenderBK->SetLayer(1);
    m_rightRenderBK->SetViewport(0.55, 0.1, 0.9, 0.9);
    m_rightRenderBK->SetPreserveColorBuffer(true);
    m_rightRenderBK->SetErase(true);
    m_rightRenderBK->InteractiveOff();
    m_rightRenderBK->SetDraw(false);
    renderWindow->AddRenderer(m_rightRenderBK);

    auto addMask = [](vtkRenderer* renderer, double opacity) -> vtkSmartPointer<vtkActor2D> {
        vtkNew<vtkPoints> pts;
        pts->InsertNextPoint(0.0, 0.0, 0.0);
        pts->InsertNextPoint(1.0, 0.0, 0.0);
        pts->InsertNextPoint(1.0, 1.0, 0.0);
        pts->InsertNextPoint(0.0, 1.0, 0.0);

        vtkNew<vtkCellArray> polys;
        vtkIdType ids[4] = { 0, 1, 2, 3 };
        polys->InsertNextCell(4, ids);

        vtkNew<vtkPolyData> quad;
        quad->SetPoints(pts);
        quad->SetPolys(polys);

        vtkNew<vtkCoordinate> coord;
        coord->SetCoordinateSystemToNormalizedViewport();

        vtkNew<vtkPolyDataMapper2D> mapper2D;
        mapper2D->SetInputData(quad);
        mapper2D->SetTransformCoordinate(coord);

        vtkSmartPointer<vtkActor2D> maskActor = vtkSmartPointer<vtkActor2D>::New();
        maskActor->SetMapper(mapper2D);
        maskActor->GetProperty()->SetColor(0.0, 0.0, 0.0);
        maskActor->GetProperty()->SetOpacity(opacity);
		maskActor->SetVisibility(true);
        renderer->AddViewProp(maskActor);
        return maskActor;
        };

    m_leftMaskActor = addMask(m_leftRenderBK, 0.5);
    m_rightMaskActor = addMask(m_rightRenderBK, 0.5);	

    // 初始化选点交互
    setupPointPicking();

    // 注册渲染同步回调：每次渲染结束后更新数字标签位置/大小
    auto leftSync = vtkSmartPointer<VtkRenderSyncCallback>::New();
    leftSync->SetOwner(this);
    m_leftRenderSyncCallback = leftSync;
    m_leftRender->AddObserver(vtkCommand::EndEvent, 
        m_leftRenderSyncCallback.GetPointer());

    auto rightSync = vtkSmartPointer<VtkRenderSyncCallback>::New();
    rightSync->SetOwner(this);
    m_rightRenderSyncCallback = rightSync;
    m_rightRender->AddObserver(vtkCommand::EndEvent, 
        m_rightRenderSyncCallback.GetPointer());

	m_fileTransfer = std::make_unique<FileTransfer>();
	connect(m_fileTransfer.get(), &FileTransfer::fileTransferCome, this,
		[this](const QString& url, const QString& displayName) {
            auto guid = QUuid::createUuid().toString(QUuid::WithoutBraces).toLower();
			auto savePath = QString("D:\\%1_%2").arg(guid).arg(displayName);
			m_fileTransfer->AnswerFileTransfer(url, true, savePath);
		});
    connect(m_fileTransfer.get(), &FileTransfer::fileTransferFinish, this,
        [this](const QString& url) {
            Q_UNUSED(url);
            ui->editConnectRes->setText(QStringLiteral("文件传输已完成"));
		});
    connect(m_fileTransfer.get(), &FileTransfer::fileTransferProcess, this,
        [this](const QString& url, qint64 bytesReceived, qint64 writeSize, qint64 totalSize) {
            Q_UNUSED(url);
            Q_UNUSED(bytesReceived);
            Q_UNUSED(writeSize);
            Q_UNUSED(totalSize);
        });
    connect(m_fileTransfer.get(), &FileTransfer::fileTransferFail, this,
        [this](const QString& url, HttpErrorCode errorCode) {
            Q_UNUSED(url);
            Q_UNUSED(errorCode);
        });
}

MainWindow::~MainWindow()
{
    stopCameraLoop();
    delete ui;
}

void MainWindow::on_btnLoadMesh_clicked()
{
    const QString filter = tr("STL Files (*.stl *.STL)");
    QString filePath = QFileDialog::getOpenFileName(
        this, tr("选择 STL 文件"), QString(), filter);

    m_mesh = LoadSTLMesh(filePath.toStdString());
    if (!m_mesh) {
        return;
    }

    for (auto& v : m_mesh->vertices_) {
        v *= 10.0; // cm -> mm
    }

    clearRendererPicks(m_rightRender);
    m_rightRender->SetDraw(true);
    m_rightRenderBK->SetDraw(true);

    if(m_rightRenderMeshActor)
		m_rightRender->RemoveActor(m_rightRenderMeshActor);
	m_rightRenderMeshActor = MeshToActor(m_mesh);
	ShowActor(m_rightRender, m_rightRenderMeshActor);
    m_rightRender->ResetCamera();

	ui->vtkWidget->renderWindow()->Render();
}

static void InitImageImportIfNeeded(vtkImageImport* imageImport,
    std::vector<unsigned char>& buffer,
    int width, int height)
{
    if (!imageImport) return;

    buffer.resize(static_cast<size_t>(width) * height * 3);

    imageImport->SetDataSpacing(1, 1, 1);
    imageImport->SetDataOrigin(0, 0, 0);
    imageImport->SetWholeExtent(0, width - 1, 0, height - 1, 0, 0);
    imageImport->SetDataExtentToWholeExtent();
    imageImport->SetNumberOfScalarComponents(3);
    imageImport->SetDataScalarTypeToUnsignedChar();
    imageImport->SetImportVoidPointer(buffer.data(), 1);
    imageImport->Update();
}

void MainWindow::OnFrameReady(
    const cv::Mat& videoFrame,
    const cv::Mat& depthFrame,
    const cv::Mat& leftIRFrame,
    const cv::Mat& rightIRFrame)
{
    if (ui->btnTravel->isChecked())
    {
		// 漫游模式下不显示摄像头背景，避免性能问题
        if (m_backgroundRender->GetBackgroundTexture())
        {
            m_backgroundRender->SetBackgroundTexture(nullptr);
        }            
        ui->vtkWidget->renderWindow()->Render();
        return;
    }
    else
    {
        if (!m_backgroundRender->GetBackgroundTexture())
        {
            m_backgroundRender->SetBackgroundTexture(m_backgroundTexture);
        }
    }

    if (videoFrame.empty()) return;  

    cv::Mat rgb;
    if (videoFrame.channels() == 3) {
        cv::cvtColor(videoFrame, rgb, cv::COLOR_BGR2RGB);
    }
    else if (videoFrame.channels() == 4) {
        cv::cvtColor(videoFrame, rgb, cv::COLOR_BGRA2RGB);
    }
    else if (videoFrame.channels() == 1) {
        cv::cvtColor(videoFrame, rgb, cv::COLOR_GRAY2RGB);
    }
    else {
        return;
    }

    cv::Mat flipped;
    cv::flip(rgb, flipped, 0);

    int dst_w = ui->vtkWidget->width();
    int dst_h = ui->vtkWidget->height();
    if (dst_w <= 0 || dst_h <= 0) {
        dst_w = flipped.cols;
        dst_h = flipped.rows;
    }

    double scale = std::max(static_cast<double>(dst_w) / flipped.cols,
        static_cast<double>(dst_h) / flipped.rows);
    int resized_w = static_cast<int>(flipped.cols * scale + 0.5);
    int resized_h = static_cast<int>(flipped.rows * scale + 0.5);

    cv::Mat resized;
    if (resized_w == flipped.cols && resized_h == flipped.rows) {
        resized = flipped;
    }
    else {
        cv::resize(flipped, resized, cv::Size(resized_w, resized_h), 
            0, 0, cv::INTER_LINEAR);
    }

    int x = std::max(0, (resized_w - dst_w) / 2);
    int y = std::max(0, (resized_h - dst_h) / 2);
    cv::Rect roi(x, y, dst_w, dst_h);
    if (roi.x + roi.width > resized.cols) roi.x = resized.cols - roi.width;
    if (roi.y + roi.height > resized.rows) roi.y = resized.rows - roi.height;
    cv::Mat finalImg = resized(roi).clone();

    int width = finalImg.cols;
    int height = finalImg.rows;

    if (!m_imageImport) {
        m_imageImport = vtkSmartPointer<vtkImageImport>::New();
        InitImageImportIfNeeded(m_imageImport, m_imageBuffer, width, height);

        m_backgroundTexture = vtkSmartPointer<vtkTexture>::New();
        m_backgroundTexture->SetInputConnection(m_imageImport->GetOutputPort());
        m_backgroundTexture->InterpolateOn();

        m_backgroundRender->SetBackgroundTexture(m_backgroundTexture);
    }
    else {
        if (static_cast<int>(m_imageBuffer.size()) != width * height * 3) {
            InitImageImportIfNeeded(m_imageImport, m_imageBuffer, width, height);
            m_backgroundTexture->SetInputConnection(m_imageImport->GetOutputPort());
            m_backgroundRender->SetBackgroundTexture(m_backgroundTexture);
        }
    }

    size_t bytes = static_cast<size_t>(width) * height * 3;
    if (finalImg.isContinuous()) {
        std::memcpy(m_imageBuffer.data(), finalImg.data, bytes);
    }
    else {
        unsigned char* dst = m_imageBuffer.data();
        for (int r = 0; r < height; ++r) {
            const unsigned char* srcRow = finalImg.ptr<unsigned char>(r);
            std::memcpy(dst + static_cast<size_t>(r) * width * 3, 
                srcRow, static_cast<size_t>(width) * 3);
        }
    }

    m_imageImport->Modified();
    if (scale != m_bkScale || x != m_bkX || y != m_bkY) {
        m_bkScale = scale;
        m_bkX = x;
        m_bkY = y;
        updateBackgroundRenderCam();
    }

    if (m_backgroundSceneDirty) {
        m_backgroundRender->ResetCameraClippingRange();
        m_backgroundSceneDirty = false;
    }
    
	ui->vtkWidget->renderWindow()->Render();
}

void MainWindow::OnPointCloudReady(const PointCloudPtr& pointCloud)
{
    clearRendererPicks(m_leftRender);
	m_leftRender->SetDraw(true);
    m_leftRenderBK->SetDraw(true);

    // 移除左侧已有的点云 actor（如果存在），再添加新的，避免重复渲染
    if (m_camPointCloudActor) {
        m_leftRender->RemoveActor(m_camPointCloudActor);
		//m_backgroundRender->RemoveActor(m_camPointCloudActor);
    }      
    
    m_camPointCloudActor = PointCloudToActor(pointCloud);
	m_camPointCloudActor->GetProperty()->SetPointSize(8);
    ShowActor(m_leftRender, m_camPointCloudActor);
    //ShowActor(m_backgroundRender, m_camPointCloudActor);
    //updateBackgroundRenderCam();
    ui->vtkWidget->renderWindow()->Render();

	m_camPointCloud = pointCloud;
}

void MainWindow::OnTrackReady(const PointCloudPtr& trackCloud, const OBJECT_UNMAP& trackPointCloud)
{
	ClearTrackActor();

    if (!trackCloud)
    {
        return;
    }

	m_trackActor = PointCloudToActor(trackCloud);
	m_trackActor->GetProperty()->SetPointSize(8);
	m_trackActor->GetProperty()->SetOpacity(0.5);

    if (m_leftRender->GetDraw())
    {
        ShowActor(m_leftRender, m_trackActor, false);
        m_backgroundSceneDirty = true;
    }

    for (auto it : trackPointCloud) {
        auto objectType = it.first;
        switch (objectType)
        {
        case OT_UNKNOWN:
            break;
        case OT_BASE:
            UpdateBase(it.second);
            break;
        case OT_PROBE:
            if (m_cameraPump)
            {               
                if(ui->btnTravel->isChecked())
                {
                    if (m_probeActor && m_probeActor->GetVisibility())
                        m_probeActor->SetVisibility(false);
                    updateTravelRenderCam(it.second);
                }
                else
                {
					ShowTrackedMesh(
						m_cameraPump->m_probeMesh,
						m_cameraPump->m_probeModel.cloud,
						it.second,
						m_probeActor);
                }
            }
            break;
        case OT_GUIDE:
            if (m_cameraPump)
            {
                ShowTrackedMesh(
                    m_cameraPump->m_guideMesh,
                    m_cameraPump->m_guideModel.cloud,
                    it.second,
                    m_guideActor);
            }
            break;
        case OT_MEASURE_PROBE:
			UpdateMeasureProbe(it.second);
            break;
        case OT_MEASURE_COORD:
            UpdateMeasureCoord(it.second);
            break;
        default:
            break;
        }
    }

    if (m_backgroundSceneDirty) {
        m_backgroundRender->ResetCameraClippingRange();
        m_backgroundSceneDirty = false;
    }

    ui->vtkWidget->renderWindow()->Render();
}

void MainWindow::startCameraLoop(int intervalMs)
{
    if (m_cameraPump)
    {
        if (!m_cameraPump->IsRunning()) {
			stopCameraLoop();
        }
        else {
			return;
        }
    }

    m_camThread = new QThread(this);
    m_cameraPump = new CameraPump();
    m_cameraPump->moveToThread(m_camThread);

    connect(m_cameraPump, &CameraPump::frameReady,
        this, &MainWindow::OnFrameReady, Qt::QueuedConnection);

    connect(m_cameraPump, &CameraPump::pointCloudReady,
        this, &MainWindow::OnPointCloudReady, Qt::QueuedConnection);

    connect(m_cameraPump, &CameraPump::trackReady,
        this, &MainWindow::OnTrackReady, Qt::QueuedConnection);

    connect(m_camThread, &QThread::finished, m_cameraPump, &QObject::deleteLater);

    m_camThread->start();
    m_cameraPump->Start(intervalMs);
}

void MainWindow::stopCameraLoop()
{
    if (m_cameraPump) {
        m_cameraPump->Stop();
        m_cameraPump->deleteLater();
        m_cameraPump = nullptr;
    }

    if (m_camThread) {
        m_camThread->quit();
        m_camThread->wait();
        m_camThread->deleteLater();
        m_camThread = nullptr;
    }
}

void MainWindow::on_btnOpenCam_clicked()
{
    if (ui->btnOpenCam->isChecked())
    {
        ui->btnOpenCam->setStyleSheet(R"(
        QPushButton:checked { 
            background-color: #FF383C;
            }
        QPushButton:hover {
          background-color: #FF643C;
        }
        )");
        ui->btnOpenCam->setIcon(QIcon(":/png/closeCam.png"));
        ui->btnOpenCam->setText(tr("断开连接"));

        startCameraLoop();
    }
    else
    {
        m_backgroundRender->SetBackgroundTexture(nullptr);
        m_imageImport = nullptr;
        m_backgroundTexture = nullptr;
        m_imageBuffer.clear();

        ui->vtkWidget->renderWindow()->Render();

        stopCameraLoop();

        ui->btnOpenCam->setIcon(QIcon(":/png/openCam.png"));
        ui->btnOpenCam->setText(tr("连接相机"));
        ui->btnOpenCam->setStyleSheet(R"(
        QPushButton {
            background-color: #177DDC;
            }
        QPushButton:hover{
            background-color: #0000DC;
            }
        )");
    }
}

void MainWindow::on_btnGetPointCloud_clicked()
{
	ui->btnTrack->setChecked(false);
    if (m_cameraPump) {
        m_cameraPump->GrabPointCloud();
    }
}

void MainWindow::updateBackgroundRenderCam()
{
    auto cam = m_backgroundRender->GetActiveCamera();
    if (!cam) {
        vtkNew<vtkCamera> newCam;
        m_backgroundRender->SetActiveCamera(newCam);
        cam = m_backgroundRender->GetActiveCamera();
    }

    double bounds[6];
    m_backgroundRender->ComputeVisiblePropBounds(bounds);

    double centerZ = camDefaultZ;
    bool hasBounds = (bounds[0] <= bounds[1] && bounds[2] <= bounds[3] 
        && bounds[4] <= bounds[5]);
    if (hasBounds) {
        double minZ = bounds[4];
        double maxZ = bounds[5];
        if (maxZ > 0.001) {
            double frontMinZ = std::max(minZ, 0.001);
            centerZ = std::max(50.0, (frontMinZ + maxZ) * 0.5);
        }
    }

    cam->SetPosition(0.0, 0.0, 0.0);
    cam->SetViewUp(0.0, 1.0, 0.0);
    cam->SetFocalPoint(0.0, 0.0, centerZ);

    CameraIntrinsics intr;
    int dst_w = ui->vtkWidget->width();
    int dst_h = ui->vtkWidget->height();
    bool hasIntr = m_cameraPump && m_cameraPump->GetColorIntrinsics(intr);
    double scale = m_bkScale;
    int x = m_bkX;
    int y = m_bkY;

    if (hasIntr && scale > 0.0 && dst_w > 0 && dst_h > 0) {
        double fy_scaled = intr.fy * scale;
        double vfov_rad = 2.0 * std::atan((dst_h * 0.5) / fy_scaled);
        double vfov_deg = vfov_rad * 180.0 / M_PI;
        cam->SetViewAngle(vfov_deg);

        double ppx_scaled = intr.ppx * scale - x;
        double ppy_flipped = (intr.height - 1.0 - intr.ppy);
        double ppy_scaled = ppy_flipped * scale - y;

        double wcx = (ppx_scaled - dst_w * 0.5) / (dst_w * 0.5);
        double wcy = (dst_h * 0.5 - ppy_scaled) / (dst_h * 0.5);

        wcx = -wcx;
        cam->SetWindowCenter(wcx, wcy);
    }
    else {
        double vfov = m_cameraPump ? m_cameraPump->GetVFOV() : 60.0;
        cam->SetViewAngle(vfov);
        cam->SetWindowCenter(0.0, 0.0);
    }

    double nearPlane = 1.0;
    double farPlane = std::max(nearPlane + 1.0, bounds[5] * 1.2);
    cam->SetClippingRange(nearPlane, farPlane);
    m_backgroundRender->ResetCameraClippingRange();
}

void MainWindow::updateTravelRenderCam(const PointCloudPtr& probePoint)
{
	//探针的四个点以顺时针排序，从最端边界的点开始，分别是P0、P1、P2、P3
	auto& P0 = probePoint->points_[0];
	auto& P1 = probePoint->points_[1];
	auto& P2 = probePoint->points_[2];
    auto& P3 = probePoint->points_[3];
    auto VLong = (P3 - P1).normalized();
	auto VShort = (P2 - P0).normalized();
	auto VNormal = VLong.cross(VShort);
    auto cam = m_backgroundRender->GetActiveCamera();
    auto focalPoint = P1 + VLong *  camDefaultZ;

    //针尖VNormal反方向偏移13.2, VLong方向偏移105.2
	auto camPos = P3 - VNormal * 13.2 + VLong * 105.2;

    cam->SetPosition(camPos.x(), camPos.y(), camPos.z());
    cam->SetViewUp(VNormal.x(), VNormal.y(), VNormal.z());
    cam->SetFocalPoint(focalPoint.x(), focalPoint.y(), focalPoint.z());
    m_backgroundSceneDirty = true;
}

// ===========================
// 初始化选点交互
// 使用 vtkPointPicker（VTK 内部加速，不会全量遍历点云，不会卡顿）
// ===========================
void MainWindow::setupPointPicking()
{
    auto* interactor = ui->vtkWidget->interactor();
    if (!interactor) return;

    m_cellPicker = vtkSmartPointer<vtkCellPicker>::New();
    m_cellPicker->SetTolerance(0.001);
    interactor->SetPicker(m_cellPicker);

    // 注册鼠标双击回调
    auto cb = vtkSmartPointer<VtkMousePickCallback>::New();
    cb->SetOwner(this);
    interactor->AddObserver(vtkCommand::LeftButtonDoubleClickEvent, cb.GetPointer());
    interactor->AddObserver(vtkCommand::RightButtonDoubleClickEvent, cb.GetPointer());

    // 注册滚轮回调：缩放后限制相机最小距离，防止旋转卡顿
    auto wheelCb = vtkSmartPointer<VtkWheelClampCallback>::New();
    interactor->AddObserver(vtkCommand::MouseWheelForwardEvent, wheelCb.GetPointer());
    interactor->AddObserver(vtkCommand::MouseWheelBackwardEvent, wheelCb.GetPointer());
}

// ===========================
// 左键双击选择点云顶点
// ===========================
void MainWindow::onLeftMousePick(int x, int y)
{
    auto* interactor = ui->vtkWidget->interactor();
    if (!interactor) return;

    // 判断鼠标在哪个 renderer 上
    vtkRenderer* renderer = interactor->FindPokedRenderer(x, y);
    if (renderer != m_leftRender && renderer != m_rightRender) return;

    tryPickPoint(renderer, x, y);

    auto* rw = static_cast<vtkGenericOpenGLRenderWindow*>(ui->vtkWidget->renderWindow());
    if (rw) rw->Render();
}

// ===========================
// 需求2：右键双击撤销点击位置附近的标记球
// ===========================
void MainWindow::onRightMouseUndo(int x, int y)
{
    auto* interactor = ui->vtkWidget->interactor();
    if (!interactor) return;

    vtkRenderer* renderer = interactor->FindPokedRenderer(x, y);
    if (renderer != m_leftRender && renderer != m_rightRender) return;

    undoPickedMarker(renderer, x, y);

    auto* rw = static_cast<vtkGenericOpenGLRenderWindow*>(ui->vtkWidget->renderWindow());
    if (rw) rw->Render();
}

// ===========================
// 尝试选点：用 vtkCellPicker 对隐藏三角网格做射线求交
// 射线只能命中最前面的三角面，天然处理遮挡
// 然后在命中三角面的顶点中选离交点最近的，映射回点云索引
// ===========================
void MainWindow::tryPickPoint(vtkRenderer* renderer, int x, int y)
{
    if (!renderer || !m_cellPicker) return;

    // 确定目标点云 actor
    vtkActor* targetActor = (renderer == m_leftRender)
        ? m_camPointCloudActor.GetPointer()
        : m_rightRenderMeshActor.GetPointer();
    if (!targetActor) return;

    // 左侧按点云原始方式选点：直接使用 vtkPointPicker
    if (renderer == m_leftRender) {
        vtkNew<vtkPointPicker> pointPicker;
        pointPicker->SetTolerance(0.01);
        pointPicker->InitializePickList();
        pointPicker->AddPickList(targetActor);
        pointPicker->SetPickFromList(1);

        int pointPicked = pointPicker->Pick(x, y, 0, renderer);
        if (!pointPicked) return;

        vtkIdType pointId = pointPicker->GetPointId();
        if (pointId < 0) return;

        auto* mapper = targetActor->GetMapper();
        auto* polyData = mapper ? vtkPolyData::SafeDownCast(mapper->GetInput()) : nullptr;
        if (!polyData || pointId >= polyData->GetNumberOfPoints()) return;

        double worldPos[3];
        polyData->GetPoint(pointId, worldPos);

        addPickMarker(renderer, pointId, worldPos);

        qDebug() << "[左]"
                 << "选中点" << pointId
                 << "位置:" << worldPos[0] << worldPos[1] << worldPos[2];
        return;
    }

    // 右侧保持 mesh 三角面片射线拾取
    vtkActor* pickMeshActor = m_rightRenderMeshActor.GetPointer();

    vtkIdType bestId = -1;
    double worldPos[3] = { 0.0, 0.0, 0.0 };

    if (pickMeshActor) {
        // 优先使用三角网格射线求交（可处理遮挡）
        m_cellPicker->InitializePickList();
        m_cellPicker->AddPickList(pickMeshActor);
        m_cellPicker->SetPickFromList(1);

        int picked = m_cellPicker->Pick(x, y, 0, renderer);

        m_cellPicker->SetPickFromList(0);
        m_cellPicker->InitializePickList();

        vtkIdType cellId = picked ? m_cellPicker->GetCellId() : -1;

        if (cellId >= 0) {
            // 获取射线与三角面的交点
            double hitPos[3];
            m_cellPicker->GetPickPosition(hitPos);

            // 获取命中三角面的顶点，选离交点最近的顶点
            auto* pickMapper = pickMeshActor->GetMapper();
            auto* meshPoly = pickMapper ? vtkPolyData::SafeDownCast(
                pickMapper->GetInput()) : nullptr;
            auto* cell = meshPoly ? meshPoly->GetCell(cellId) : nullptr;

            if (meshPoly && cell) {
                double bestDist2 = std::numeric_limits<double>::max();
                for (vtkIdType j = 0; j < cell->GetNumberOfPoints(); ++j) {
                    vtkIdType vid = cell->GetPointId(j);
                    double pt[3];
                    meshPoly->GetPoint(vid, pt);
                    double d2 = (pt[0] - hitPos[0]) * (pt[0] - hitPos[0])
                              + (pt[1] - hitPos[1]) * (pt[1] - hitPos[1])
                              + (pt[2] - hitPos[2]) * (pt[2] - hitPos[2]);
                    if (d2 < bestDist2) {
                        bestDist2 = d2;
                        bestId = vid;
                    }
                }

                if (bestId >= 0) {
                    meshPoly->GetPoint(bestId, worldPos);
                }
            }
        }
    }

    // 若右侧三角网格拾取失败，回退到点拾取，保证可用性
    if (bestId < 0) {
        vtkNew<vtkPointPicker> fallbackPointPicker;
        fallbackPointPicker->SetTolerance(0.01);
        fallbackPointPicker->InitializePickList();
        fallbackPointPicker->AddPickList(targetActor);
        fallbackPointPicker->SetPickFromList(1);

        int pointPicked = fallbackPointPicker->Pick(x, y, 0, renderer);
        if (!pointPicked) return;

        bestId = fallbackPointPicker->GetPointId();
        if (bestId < 0) return;

        auto* mapper = targetActor->GetMapper();
        auto* polyData = mapper ? vtkPolyData::SafeDownCast(
            mapper->GetInput()) : nullptr;
        if (!polyData || bestId >= polyData->GetNumberOfPoints()) {
            return;
        }
        polyData->GetPoint(bestId, worldPos);
    }

    addPickMarker(renderer, bestId, worldPos);

    qDebug() << (renderer == m_leftRender ? "[左]" : "[右]")
             << "选中点" << bestId
             << "位置:" << worldPos[0] << worldPos[1] << worldPos[2];
}

// ===========================
// 撤销右键点击位置附近的标记球
// 在屏幕空间搜索距离点击位置最近的标记，如果未命中则删除最后一个
// ===========================
void MainWindow::undoPickedMarker(vtkRenderer* renderer, int x, int y)
{
    if (!renderer) return;

    auto& records = (renderer == m_leftRender) ? m_leftPickRecords :
        m_rightPickRecords;
    if (records.empty()) return;

    // 搜索半径：18 像素
    constexpr double kPickRadiusPx = 18.0;
    const double kPickRadius2 = kPickRadiusPx * kPickRadiusPx;

    const double mx = static_cast<double>(x);
    const double my = static_cast<double>(y);

    int bestIndex = -1;
    double bestD2 = std::numeric_limits<double>::max();

    for (int i = 0; i < static_cast<int>(records.size()); ++i) {
        const auto& rec = records[i];

        // 将标记的世界坐标转换为屏幕坐标
        renderer->SetWorldPoint(rec.worldPos[0], rec.worldPos[1], rec.worldPos[2], 1.0);
        renderer->WorldToDisplay();
        double disp[3] = { 0.0, 0.0, 0.0 };
        renderer->GetDisplayPoint(disp);

        // 计算屏幕距离（物理像素）
        const double dx = disp[0] - mx;
        const double dy = disp[1] - my;
        const double d2 = dx * dx + dy * dy;

        if (d2 <= kPickRadius2 && d2 < bestD2) {
            bestD2 = d2;
            bestIndex = i;
        }
    }

    // 若未点击到已有标记，则回退为删除最后一个标记
    if (bestIndex < 0) {
        bestIndex = static_cast<int>(records.size()) - 1;
    }

    // 从 renderer 中删除该标记的圆点和标签
    auto& rec = records[bestIndex];
    if (rec.markerActor) renderer->RemoveViewProp(rec.markerActor.GetPointer());
    if (rec.labelActor) renderer->RemoveViewProp(rec.labelActor.GetPointer());
    records.erase(records.begin() + bestIndex);

    // 需求2：重新排列数字索引
    renumberPickLabels(renderer);

    qDebug() << (renderer == m_leftRender ? "[左]" : "[右]")
             << "撤销标记，剩余:" << records.size();
}

// ===========================
// 重新编号所有标签（1, 2, 3, ...）
// ===========================
void MainWindow::renumberPickLabels(vtkRenderer* renderer)
{
    auto& records = (renderer == m_leftRender) ? m_leftPickRecords : m_rightPickRecords;
    for (size_t i = 0; i < records.size(); ++i) {
        if (records[i].labelActor) {
            records[i].labelActor->SetInput(
                std::to_string(static_cast<int>(i + 1)).c_str());
        }
    }
    updateLabelPositions(renderer);
}

// ===========================
// 添加选点标记（2D 圆点 + 数字标签）
// 使用 2D overlay，标记在屏幕上始终为固定像素大小
// ===========================
void MainWindow::addPickMarker(
    vtkRenderer* renderer, 
    vtkIdType pointId, 
    const double worldPos[3])
{
    if (!renderer) return;

    auto& records = (renderer == m_leftRender) ? m_leftPickRecords : m_rightPickRecords;
    const int order = static_cast<int>(records.size() + 1);

    // 固定屏幕像素半径（增大 0.5 倍）
    constexpr int kMarkerRadius = 9;
    const int kLabelFontSize = static_cast<int>(std::lround(2.0 * kMarkerRadius * 0.7));

    // 用 vtkRegularPolygonSource 生成 2D 圆（50 边近似）
    vtkNew<vtkRegularPolygonSource> circle;
    circle->SetNumberOfSides(50);
    circle->SetRadius(kMarkerRadius);
    circle->SetCenter(0.0, 0.0, 0.0);
    circle->GeneratePolygonOn();
    circle->Update();

    vtkNew<vtkCoordinate> coord;
    coord->SetCoordinateSystemToDisplay();

    vtkNew<vtkPolyDataMapper2D> mapper2D;
    mapper2D->SetInputConnection(circle->GetOutputPort());
    mapper2D->SetTransformCoordinate(coord);

    auto markerActor = vtkSmartPointer<vtkActor2D>::New();
    markerActor->SetMapper(mapper2D);
    markerActor->PickableOff();
    // 统一为绿色
    markerActor->GetProperty()->SetColor(0.0, 1.0, 0.0);
    markerActor->GetProperty()->SetOpacity(0.8);
    renderer->AddViewProp(markerActor);

    // 创建数字标签
    auto labelActor = vtkSmartPointer<vtkTextActor>::New();
    labelActor->SetInput(std::to_string(order).c_str());
    auto* tp = labelActor->GetTextProperty();
    tp->BoldOn();
    tp->SetFontSize(kLabelFontSize);
    tp->SetColor(0.0, 0.0, 0.0);
    tp->SetJustificationToCentered();
    tp->SetVerticalJustificationToCentered();
    renderer->AddViewProp(labelActor);

    // 记录
    PickRecord rec;
    rec.pointId = pointId;
    rec.worldPos[0] = worldPos[0];
    rec.worldPos[1] = worldPos[1];
    rec.worldPos[2] = worldPos[2];
    rec.markerActor = markerActor;
    rec.labelActor = labelActor;
    records.push_back(rec);

    // 立即更新标记和标签的屏幕位置
    updateLabelPositions(renderer);
}

// ===========================
// 更新 2D 标记和数字标签的屏幕位置
// - 将世界坐标投影到屏幕，更新 2D 圆点和标签位置
// - 每次渲染后通过 EndEvent 回调自动调用
// ===========================
void MainWindow::updateLabelPositions(vtkRenderer* renderer)
{
    if (!renderer) return;

    // 防止递归：修改 actor 属性会触发渲染 → EndEvent → 再次进入本函数
    if (m_updatingLabels) return;
    m_updatingLabels = true;

    auto& records = (renderer == m_leftRender) ? 
        m_leftPickRecords : m_rightPickRecords;

    for (size_t i = 0; i < records.size(); ++i) {
        auto& rec = records[i];

        // 世界坐标投影到屏幕坐标
        renderer->SetWorldPoint(
            rec.worldPos[0], 
            rec.worldPos[1], 
            rec.worldPos[2], 1.0);
        renderer->WorldToDisplay();
        double disp[3] = { 0.0, 0.0, 0.0 };
        renderer->GetDisplayPoint(disp);

        const int sx = static_cast<int>(std::lround(disp[0]));
        const int sy = static_cast<int>(std::lround(disp[1]));

        // 更新 2D 圆点标记位置
        if (rec.markerActor) {
            rec.markerActor->SetPosition(sx, sy);
        }

        // 更新数字标签位置（中心放在圆点中心）
        if (rec.labelActor) {
            rec.labelActor->SetInput(std::to_string(static_cast<int>(i + 1)).c_str());
            rec.labelActor->SetDisplayPosition(sx, sy);
        }
    }

    m_updatingLabels = false;
}

// ===========================
// 清空指定 renderer 的所有选点标记
// ===========================
void MainWindow::clearRendererPicks(vtkRenderer* renderer)
{
    if (!renderer) return;

    auto& records = (renderer == m_leftRender) ? m_leftPickRecords : m_rightPickRecords;
    for (auto& rec : records) {
        if (rec.markerActor) 
            renderer->RemoveViewProp(rec.markerActor.GetPointer());
        if (rec.labelActor) 
            renderer->RemoveViewProp(rec.labelActor.GetPointer());
    }
    records.clear();

}

// ===========================
// 获取左右 renderer 中被选中的顶点索引
// ===========================
void MainWindow::GetPickedPointIndices(
    std::vector<int>& leftIndices,
    std::vector<int>& rightIndices) const
{
    leftIndices.clear();
    rightIndices.clear();
    leftIndices.reserve(m_leftPickRecords.size());
    rightIndices.reserve(m_rightPickRecords.size());

    for (const auto& rec : m_leftPickRecords)
        leftIndices.push_back(static_cast<int>(rec.pointId));
    for (const auto& rec : m_rightPickRecords)
        rightIndices.push_back(static_cast<int>(rec.pointId));
}

void MainWindow::SetLayerVisible(int layerIndex, bool visible)
{
    auto rw = ui->vtkWidget->renderWindow();
    vtkRendererCollection* rc = rw->GetRenderers();
    rc->InitTraversal();

    vtkRenderer* ren = nullptr;
    while ((ren = rc->GetNextItem()) != nullptr) {
        if (ren->GetLayer() != layerIndex) continue;
        ren->SetDraw(visible);
    }

	rw->Render();
}

void MainWindow::UpdateBackgroundMesh(const TriangleMeshPtr& mesh)
{
    if (m_meshRegisterActor) {
        m_backgroundRender->RemoveActor(m_meshRegisterActor);
    }

    m_meshRegisterActor = MeshToActor(mesh);
    m_meshRegisterActor->GetProperty()->SetOpacity(1.0);           // 半透明         
    ShowActor(m_backgroundRender, m_meshRegisterActor);
    //updateBackgroundRenderCam();
}

void MainWindow::UpdateBase(const PointCloudPtr& basePointCloud)
{
	auto basePoint = std::make_shared<PointCloud>(*basePointCloud);
	MakaGradient(basePoint, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 });
    m_baseActor = PointCloudToActor(basePoint);
    m_baseActor->GetProperty()->SetPointSize(8);
    //m_baseActor->GetProperty()->SetOpacity(1.0);
    ShowActor(m_backgroundRender, m_baseActor, false);
    m_backgroundSceneDirty = true;

    //如果发现基座中点偏移了阈值，则开始计时倒数，查看基座中心顶点是否再
    //移动指定阈值，如果再移动，重置倒数时间，倒数完毕时，发现基座中心顶点
    //不再移动，此时触发移动相机，如果在倒数的过程中，发现相机移动标志为true，
    //则不再倒数计时
    if (!IsValidPoint3D(m_autoObserveLastCenter)) {
        m_autoObserveLastCenter = basePointCloud->GetCenter();
    }
    else {
        auto& older = m_autoObserveLastCenter;
        auto newer = basePointCloud->GetCenter();
        auto dist = (older - newer).squaredNorm();
        if (m_camIsMoved) {
            m_autoObserveLastCenter = newer;
            m_isAutoObserveCamMove = false;
            if (dist >= m_autoObserveCamMoveMinDist) {
                m_newBase.clear();
                qDebug() << "Camera moved, reset new base points. Dist: " << dist;
            }
        }
        else {
            if (!m_isAutoObserveCamMove) {
                if (dist >= m_autoObserveCamMoveMinDist) {
                    m_isAutoObserveCamMove = true;
                    m_autoObserveCamMoveStart = std::chrono::steady_clock::now();
                    m_autoObserveLastCenter = newer;
                    qDebug() << "start observer camera move " << dist;
                }
            }
            else {
                if (dist >= m_autoObserveCamMoveMinDist) {
                    m_autoObserveCamMoveStart = std::chrono::steady_clock::now();
                    m_autoObserveLastCenter = newer;
                    qDebug() << "continue observer camera move " << dist;
                }
                else {
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - m_autoObserveCamMoveStart).count();
                    if (elapsed >= m_autoObserveCamMoveDelay) {
                        m_camIsMoved = true;
                        m_isAutoObserveCamMove = false;
                        m_newBase.clear();
                        qDebug() << "camera stop moving " << dist;
                    }
                }
            }
        }
    }

    //后期可以根据点的数量和几何结构识别是否为基座,如果是基座进入这个代码段
    //如果再开启追踪后,m_oldBase的数量小于30此时移动相机，则采集的初始点有
    //有问题，一秒钟现在30帧，也就是开始追踪后一秒之内不能移动基座
    constexpr int basePointNum = 5;
    if (m_oldBase.size() < basePointNum) {
        m_oldBase.emplace_back(basePointCloud);
    }
    else if (m_camIsMoved) {
        if (m_newBase.size() < basePointNum) {
            m_newBase.emplace_back(basePointCloud);
        }
        else {
            do {
                if (!m_mesh)
                    break;

                TriangleMeshPtr cloneMesh = std::make_shared<TriangleMesh>(*m_mesh);
                std::vector<TriangleMeshPtr> meshes{ cloneMesh };

                //四个点是基座，三个点是探针,后期可以根据数量决定种类
                auto ret = CalibrateMeshFromReference(m_oldBase, m_newBase, meshes);
                auto& s = ret.sourceFeatures.points_;
                auto& t = ret.targetFeatures.points_;
                if ((s.size() != t.size()) || s.empty() || t.empty()) {
                    break;
                }

                UpdateBackgroundMesh(meshes[0]);
                m_camIsMoved = false;
                m_newBase.clear();
            } while (false);
        }
    }
}

void MainWindow::ShowTrackedMesh(
    const TriangleMeshPtr& mesh,
    const PointCloudPtr& modelPointClod,
    const PointCloudPtr& trackPointCloud,
    ActorPtr& actor)
{
    auto T = ComputeTransformationFromCorrespondences(
        *modelPointClod, *trackPointCloud);

    if (!actor) {
        actor = MeshToActor(mesh);
        ShowActor(m_backgroundRender, actor, false);
    }

    vtkNew<vtkMatrix4x4> m;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            m->SetElement(r, c, T(r, c));
        }
    }

    actor->SetUserMatrix(m);
    actor->SetVisibility(true);
    actor->Modified();

    m_backgroundSceneDirty = true;
}

void MainWindow::ClearTrackActor()
{
    if (m_trackActor) {
        m_backgroundRender->RemoveActor(m_trackActor);
        m_leftRender->RemoveActor(m_trackActor);
        m_trackActor = nullptr;
        m_backgroundSceneDirty = true;
    }

    if (m_baseActor) {
        m_backgroundRender->RemoveActor(m_baseActor);
        m_baseActor = nullptr;
        m_backgroundSceneDirty = true;
    }
    if (m_probeActor) {
        m_probeActor->SetVisibility(false);
    }
    if (m_guideActor) {
        m_guideActor->SetVisibility(false);
    }

    if (m_measProbeActor) {
        m_backgroundRender->RemoveActor(m_measProbeActor);
        m_measProbeActor = nullptr;
        m_backgroundSceneDirty = true;
    }
    if (m_measCoordActor) {
        m_backgroundRender->RemoveActor(m_measCoordActor);
        m_measCoordActor = nullptr;
        m_backgroundSceneDirty = true;
    }
}

#pragma region 测量精度相关
void MainWindow::UpdateMeasureProbe(const PointCloudPtr& trackPointCloud)
{
    auto T = ComputeTransformationFromCorrespondences(
        *m_cameraPump->m_measProbeModel.cloud, *trackPointCloud);

    auto measureOriginPoint = Point3D(0.0, 0.0, 0.0);
    Eigen::Affine3d A(T);
    measureOriginPoint = A * measureOriginPoint;

    auto measurePoint = std::make_shared<PointCloud>(*trackPointCloud);
    ArrangePointCloud(measurePoint);
	measurePoint->points_.emplace_back(measureOriginPoint);

    MakaGradient(
        measurePoint, 
        { 1.0, 1.0, 0.0 },  //黄绿色
        { 0.0, 1.0, 1.0 }); //青色
    m_measProbeActor = PointCloudToActor(measurePoint);
    m_measProbeActor->GetProperty()->SetPointSize(8);
    ShowActor(m_backgroundRender, m_measProbeActor, false);
    m_backgroundSceneDirty = true;
}

void MainWindow::UpdateMeasureCoord(const PointCloudPtr& trackPointCloud)
{
    auto measurePoint = std::make_shared<PointCloud>(*trackPointCloud);
    ArrangePointCloud(measurePoint);
    MakaGradient(
        measurePoint,
        { 1.0, 0.0, 1.0 },  //粉红色
        { 1.0, 1.0, 0.0 }); //黄色);
    m_measCoordActor = PointCloudToActor(measurePoint);
    m_measCoordActor->GetProperty()->SetPointSize(8);
    ShowActor(m_backgroundRender, m_measCoordActor, false);
    m_backgroundSceneDirty = true;

    m_measToRealT = ComputeTransformationFromCorrespondences(
        *trackPointCloud, *m_cameraPump->m_measCoordModel.cloud);
}

void MainWindow::CalMeasurePoint(int ballNum)
{
    if (!m_measProbeActor)
        return;

    auto& realPoint = m_cameraPump->m_measBallsPoints[ballNum];

	auto measure = ActorToPointCloud(m_measProbeActor);
	auto measPoint = measure->points_.back();
    Eigen::Affine3d A(m_measToRealT);
    measPoint = A * measPoint;

    auto error = (measPoint - realPoint).norm();
    if(m_measBalls.find(ballNum) == m_measBalls.end())
	    m_measBalls[ballNum] = {};

	auto& measBall = m_measBalls[ballNum];
    measBall.measPoints.emplace_back(measPoint);
    measBall.realPoint = realPoint;
    measBall.ballNum = ballNum;
    measBall.errors.emplace_back(error);
    std::cout << "Ball " << ballNum << ":" 
        << "Meas("<< measPoint.x() << "," << measPoint.y() << "," << measPoint.z() << "),"
        << "Real("<< realPoint.x() << ", " << realPoint.y() << ", " << realPoint.z() << ")," 
        << "Error: " << error << "mm" << std::endl;

	auto& errors = measBall.errors;
    auto mm = std::minmax_element(errors.begin(), errors.end());
    auto sum = std::accumulate(errors.begin(), errors.end(), 0.0);
    auto avgError = sum / errors.size();
    measBall.minError = *mm.first;
    measBall.maxError = *mm.second;
    measBall.avgError = avgError;
}

void MainWindow::on_btnMeasProbe_clicked()
{
    auto ballNumStr = ui->editMeasBallNum->text().trimmed();
    bool ok;
    int ballNum = ballNumStr.toInt(&ok);
    if (!ok || m_cameraPump->m_measBallsPoints.size() < ballNum + 1)
    {
        QMessageBox::warning(this, tr("错误"), tr("请输入有效的球编号"));
        return;
    }
    CalMeasurePoint(ballNum);
}

void MainWindow::on_btnOutError_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("保存测量误差"), "", "TXT Files (*.txt)");
    if (fileName.isEmpty()) return;
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, tr("错误"), tr("无法打开文件"));
        return;
    }

    QTextStream out(&file);
    std::vector<double> errors;
    for(auto& it : m_measBalls)
    {
        auto& ball = it.second;
        out << tr("球编号: ") << ball.ballNum<< "\n";
        out << tr("真实坐标: ")
            << ball.realPoint.x() << ", "
            << ball.realPoint.y() << ", " 
            << ball.realPoint.z() << "\n";
        out << tr("测量坐标:\n");
        for (auto& p : ball.measPoints) {
            out << p.x() << ", " << p.y() << ", " << p.z() << "\n";
        }
        out << tr("误差:\n");
        for (auto e : ball.errors) {
            out << e << "\n";
			errors.push_back(e);
        }
        out << tr("最小误差: ") << ball.minError << "\n";
        out << tr("最大误差: ") << ball.maxError << "\n";
        out << tr("平均误差: ") << ball.avgError << "\n";
        out << "-----------------------------------------\n";
	}
    if (!errors.empty()) {
        auto mm = std::minmax_element(errors.begin(), errors.end());
        out << tr("所有球误差范围: ") << *mm.first << " ~ " << *mm.second << "\n";

        auto sum = std::accumulate(errors.begin(), errors.end(), 0.0);
        auto avgError = sum / errors.size();
        out << tr("所有球平均误差: ") << avgError << "\n";
        out << "=============================================================\n";
	}

	out.flush();
	file.close();
}

void MainWindow::on_btnResetMeas_clicked()
{
	m_measBalls.clear();
}

void MainWindow::on_btnDelBallErorr_clicked()
{
    auto ballNumStr = ui->editMeasBallNum->text().trimmed();
    bool ok;
    int ballNum = ballNumStr.toInt(&ok);
    if (ok)
    {
		m_measBalls.erase(ballNum);
    }
}

#pragma endregion

void MainWindow::updateRegisterResult(const
    FineAlignmentResult& result, qint64 elapsedMs)
{
    ui->registerPrecisionLabel->setText(QString::number(result.inlier_rmse, 'f', 1) + "mm");
    ui->registerTimeLabel->setText(QString::number(elapsedMs / 1000.0, 'f', 1) + "s");
    ui->matchDegreeLabel->setText(QString::number(result.fitness * 100.0, 'f', 0) + "%");

    QString quality = result.converged && result.inlier_rmse <= 1.5 ? tr("优") : tr("中");
    if (!result.converged && result.inlier_rmse > 2) quality = tr("差");
	ui->evaluationLabel->setText(quality);
}

void MainWindow::on_btnRegiste_clicked()
{
    if (!m_mesh)
        return;

	std::vector<int> targetIndices, sourceIndices;
	GetPickedPointIndices(targetIndices, sourceIndices);
	auto source = MeshToPointCloud(m_mesh);
    auto& target = m_camPointCloud;
    if (source && 
        target && 
        targetIndices.size() && 
        targetIndices.size() == sourceIndices.size()) {
        QElapsedTimer timer;
        timer.start();

        FineAlignmentResult result;
        auto T = RegisterPointCloudFromPoints(
            sourceIndices, 
            targetIndices, 
            *source, 
            *target,
            &result);

        updateRegisterResult(result, timer.elapsed());

		m_mesh->Transform(T);
        qDebug() << "配准结果: 误差=" << result.inlier_rmse;
		UpdateBackgroundMesh(m_mesh);
        ui->vtkWidget->renderWindow()->Render();
    }

    m_camIsMoved = false;
    m_isAutoObserveCamMove = false;
    m_autoObserveLastCenter = InvalidPoint3D();
    m_oldBase.clear();
    m_newBase.clear();
}

void MainWindow::on_btnTrack_toggled(bool checked)
{
    if(m_cameraPump)
	    checked ? m_cameraPump->StartTrack() : m_cameraPump->StopTrack();

    if (!checked)
    {
        ClearTrackActor();
        m_backgroundSceneDirty = true;
    }
}

void MainWindow::on_btnShowPointCloud_toggled(bool checked)
{
	SetLayerVisible(1, checked);
    SetLayerVisible(2, checked);
}

void MainWindow::on_btnSetBase_clicked()
{
    if (!m_trackActor)
        return;

    auto base = ActorToPointCloud(m_trackActor);
    DeleteOuterPoints(base, 200);
    ArrangePointCloud(base);
    MakaGradient(base, { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 });

    OBJECT_UNMAP models = {
            { OT_PROBE,m_cameraPump->m_probeModel.cloud },
            { OT_GUIDE,m_cameraPump->m_guideModel.cloud },
    };
	auto ret = CheckBaseValid(base, models);
    if (ret != BCR_SUCESS) {
        std::unordered_map<BASE_CHECK_RESULT, QString> errorMessages = {
			{BCR_SIDE_TOO_SHORT, QStringLiteral("最小边长小于10mm")},
			{BCR_SIDE_TOO_LONG, QStringLiteral("最大边长大于200mm")},
			{BCR_MINSIDE_INVALID, QStringLiteral("最小边长和第二短边长的差的绝对值不能小于10mm")},
			{BCR_POINT_NUM_ERROR, QStringLiteral("点的数量错误")},
		};
        QMessageBox::warning(this, QStringLiteral("错误"), errorMessages[ret]);
    }
    else {
        m_cameraPump->m_baseModel.type = OT_BASE;
        m_cameraPump->m_baseModel.cloud = base;
    }
}

void MainWindow::on_btnConnectServer_clicked()
{
	auto IP = ui->editServerIP->text().trimmed();
    auto ret = m_fileTransfer->Start(IP);
    if (ret) 
    {
		ui->editConnectRes->setText(QStringLiteral("连接成功"));
    }
}

void MainWindow::on_btnTravel_clicked()
{
    if (!ui->btnTravel->isChecked())
    {
        updateBackgroundRenderCam();
        ui->vtkWidget->renderWindow()->Render();
    }
}