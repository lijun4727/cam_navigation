#include "orbbecv_cam.h"
#include "utils_types.h"

#include "libobsensor/hpp/Utils.hpp"
#include <libobsensor/ObSensor.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>

OrbbecvCam::OrbbecvCam() {

}

// check if the given stream profiles support hardware depth-to-color alignment
bool checkIfSupportHWD2CAlign(
	std::shared_ptr<ob::Pipeline> pipe,
	std::shared_ptr<ob::StreamProfile> colorStreamProfile,
	std::shared_ptr<ob::StreamProfile> depthStreamProfile) {
	auto hwD2CSupportedDepthStreamProfiles = pipe->getD2CDepthProfileList(colorStreamProfile, ALIGN_D2C_HW_MODE);
	if (hwD2CSupportedDepthStreamProfiles->count() == 0) {
		return false;
	}

	// Iterate through the supported depth stream profiles and check if there is a match with the given depth stream profile
	auto depthVsp = depthStreamProfile->as<ob::VideoStreamProfile>();
	auto count = hwD2CSupportedDepthStreamProfiles->getCount();
	for (uint32_t i = 0; i < count; i++) {
		auto sp = hwD2CSupportedDepthStreamProfiles->getProfile(i);
		auto vsp = sp->as<ob::VideoStreamProfile>();
		if (vsp->getWidth() == depthVsp->getWidth() && vsp->getHeight() == depthVsp->getHeight() && vsp->getFormat() == depthVsp->getFormat()
			&& vsp->getFps() == depthVsp->getFps()) {
			// Found a matching depth stream profile, it is means the given stream profiles support hardware depth-to-color alignment
			return true;
		}
	}
	return false;
}

// create a config for hardware depth-to-color alignment
std::shared_ptr<ob::Config> createHwD2CAlignConfig(std::shared_ptr<ob::Pipeline> pipe) {
	auto coloStreamProfiles = pipe->getStreamProfileList(OB_SENSOR_COLOR);
	auto depthStreamProfiles = pipe->getStreamProfileList(OB_SENSOR_DEPTH);

	// Iterate through all color and depth stream profiles to find a match for hardware depth-to-color alignment
	auto colorSpCount = coloStreamProfiles->getCount();
	auto depthSpCount = depthStreamProfiles->getCount();
	for (uint32_t i = 0; i < colorSpCount; i++) {
		auto colorProfile = coloStreamProfiles->getProfile(i);
		auto colorVsp = colorProfile->as<ob::VideoStreamProfile>();
		for (uint32_t j = 0; j < depthSpCount; j++) {
			auto depthProfile = depthStreamProfiles->getProfile(j);
			auto depthVsp = depthProfile->as<ob::VideoStreamProfile>();

			// make sure the color and depth stream have the same fps, due to some models may not support different fps
			if (colorVsp->getFps() != depthVsp->getFps()) {
				continue;
			}

			// Check if the given stream profiles support hardware depth-to-color alignment
			if (checkIfSupportHWD2CAlign(pipe, colorProfile, depthProfile)) {
				// If support, create a config for hardware depth-to-color alignment
				auto hwD2CAlignConfig = std::make_shared<ob::Config>();
				hwD2CAlignConfig->enableStream(colorProfile);                                                     // enable color stream
				hwD2CAlignConfig->enableStream(depthProfile);                                                     // enable depth stream
				hwD2CAlignConfig->setAlignMode(ALIGN_D2C_HW_MODE);                                                // enable hardware depth-to-color alignment
				hwD2CAlignConfig->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);  // output frameset with all types of frames
				return hwD2CAlignConfig;
			}
		}
	}

	return nullptr;
}

bool OrbbecvCam::Open(const CameConfig& camConfig)
{
	IDepthCamera::Open(camConfig);

	pipeline_ = std::make_shared<ob::Pipeline>();
	pipeline_->enableFrameSync();

	auto config = createHwD2CAlignConfig(pipeline_);
	if (!config) {
		config = std::make_shared<ob::Config>();
		config->enableVideoStream(
			OB_STREAM_DEPTH,
			OB_WIDTH_ANY,
			OB_HEIGHT_ANY,
			OB_FPS_ANY,
			OB_FORMAT_ANY);
		config->enableVideoStream(
			OB_STREAM_COLOR,
			OB_WIDTH_ANY,
			OB_HEIGHT_ANY,
			OB_FPS_ANY,
			OB_FORMAT_RGB);
		config->setFrameAggregateOutputMode(
			OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
		align_ = std::make_shared<ob::Align>(OB_STREAM_COLOR);
		std::cout << "ʹ      D2C     " << std::endl;
	}

	// Enumerate and config all sensors.
	auto device = pipeline_->getDevice();
	auto devInfo = device->getDeviceInfo();
	auto pid = devInfo->getPid();
	auto vid = devInfo->getVid();

	// Get sensor list from device.
	auto sensorList = device->getSensorList();
	for (uint32_t i = 0; i < sensorList->getCount(); i++) {
		// Get sensor type.
		auto sensorType = sensorList->getSensorType(i);

		// exclude gyro and accel sensors.
		if (sensorType == OB_SENSOR_GYRO || sensorType == OB_SENSOR_ACCEL) {
			continue;
		}

		if (IS_ASTRA_MINI_DEVICE(vid, pid)) {
			if (sensorType == OB_SENSOR_COLOR) {
				continue;
			}
		}

		if (sensorType == OB_SENSOR_IR &&
			!(camConfig.stremType & (int)STREAM_TYPE::IR)){
				continue;
		}
		if (sensorType == OB_SENSOR_IR_LEFT &&
			!(camConfig.stremType & (int)STREAM_TYPE::LEFT_IR)){
				continue;
		}
		if (sensorType == OB_SENSOR_IR_RIGHT &&
			!(camConfig.stremType & (int)STREAM_TYPE::RIGHT_IR)){
				continue;
		}
		if (sensorType == OB_SENSOR_CONFIDENCE &&
			!(camConfig.stremType & (int)STREAM_TYPE::CONFIDENCE)) {
			continue;
		}

		config->enableStream(sensorType);
	}

	pipeline_->start(config);

	pointCloudFilter_ = std::make_shared<ob::PointCloudFilter>();

	//auto device = pipeline_->getDevice();
	//bool hwAlignSupported = device->isPropertySupported(
	//	OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL,
	//	OB_PERMISSION_READ_WRITE);
	//if (!hwAlignSupported) {
	//	align_ = std::make_shared<ob::Align>(OB_STREAM_COLOR);
	//}
	//else {
	//	//     Ӳ   D2C    루  ȶ  뵽  ɫ  
	//	config->setAlignMode(ALIGN_D2C_HW_MODE);
	//}
	
	return true;
}

void OrbbecvCam::Close() {
	IDepthCamera::Close();
	pipeline_->stop();
}

bool OrbbecvCam::Snapshot(int32_t timeout) {
	using namespace std::chrono;
	try {
		auto deadline = steady_clock::now() + milliseconds(timeout);
		frameSet_ = nullptr;
		while (steady_clock::now() < deadline) {
			frameSet_ = pipeline_->waitForFrameset(timeout);
			if (frameSet_ || exit_) {
				break;
			}
		}
	}
	catch (ob::Error& e) {
		std::cerr << "function:" << 
			e.getFunction() << "\nargs:" << 
			e.getArgs() << "\nmessage:" << 
			e.what() << "\ntype:" << 
			e.getExceptionType() << 
			std::endl;
	}

	return frameSet_ != nullptr;
}

PointCloudPtr OrbbecvCam::GetPointColudFromPixel(
	const VecPixelCoord& pixelCoords,
	std::vector<size_t >* failPCIndexes,
	VecRGB* rgb) {
	PointCloudPtr cloud = std::make_shared<PointCloud>();
	for(size_t i = 0; i < pixelCoords.size(); i++) {
		Point3D p;
		auto pixel = pixelCoords[i];
		if(GetPointAtPixel(pixel.x(), pixel.y(), &p)) {
			cloud->points_.emplace_back(p.x(), p.y(), p.z());
			if(rgb)
				cloud->colors_.emplace_back((*rgb)[i]);
		} else {
			if(failPCIndexes) {
				failPCIndexes->emplace_back(i);
			}
		}
	}

	return cloud;
}

PointCloudPtr OrbbecvCam::OrbbecToOpen3D(
	std::shared_ptr<ob::Frame> orbbecFrame, 
	float maxZ,
	float maxX,
	float maxY) {
	auto cloud = std::make_shared<open3d::geometry::PointCloud>();

	// Get raw point data
	auto data = reinterpret_cast<OBColorPoint*>(orbbecFrame->getData());
	int count = orbbecFrame->getDataSize() / sizeof(OBColorPoint);

	// Reserve space
	cloud->points_.reserve(count);
	cloud->colors_.reserve(count);

	// Convert each valid point
	for (int i = 0; i < count; i++) {
		if (data[i].z > 0 && 
			data[i].z <= maxZ &&
			std::abs(data[i].x) <= maxX && 
			std::abs(data[i].y) <= maxY) {
			// Coordinates: keep in millimeters (no conversion)
			cloud->points_.emplace_back(
				Eigen::Vector3d(
					data[i].x, 
					data[i].y, 
					data[i].z)
			);

			// Colors: 0-255    0-1
			cloud->colors_.emplace_back(
				Eigen::Vector3d(
				data[i].r / 255.0,
				data[i].g / 255.0,
				data[i].b / 255.0)
			);
		}
	}

	return cloud;
}

PointCloudPtr OrbbecvCam::GetPointCloud(
	float maxZ,
	float maxX,
	float maxY) {
	// TODO: ʵ ֻ ȡ        
	pointCloudFilter_->setCreatePointFormat(OB_FORMAT_RGB_POINT);

	std::shared_ptr<ob::Frame> frame;
	if (align_) {
		auto frameSet = align_->process(frameSet_);
		frame = pointCloudFilter_->process(frameSet);
	}
	else {
		frame = pointCloudFilter_->process(frameSet_);
	}
	return OrbbecToOpen3D(frame, maxZ, maxX, maxY);
}

bool OrbbecvCam::GetPointAtPixel(
	int x, 
	int y, 
	Point3D* p, 
	STREAM_TYPE stremType)
{
	OBFrameType frameType = OB_FRAME_COLOR;
	if(stremType == STREAM_TYPE::IR) {
		frameType = OB_FRAME_IR;
	}
	else if (stremType == STREAM_TYPE::LEFT_IR) {
		frameType = OB_FRAME_IR_LEFT;
	}
	else if (stremType == STREAM_TYPE::RIGHT_IR) {
		frameType = OB_FRAME_IR_RIGHT;
	}

	auto frame = frameSet_->getFrame(frameType);
	auto depthFrame = frameSet_->getFrame(OB_FRAME_DEPTH);
	auto depthFrameWidth = depthFrame->as<ob::VideoFrame>()->getWidth();
	auto depthFrameHeight = depthFrame->as<ob::VideoFrame>()->getHeight();
	if (x >= (int)depthFrameWidth || y >= (int)depthFrameHeight) {
		return false;
	}

	// Access the depth data from the frame
	uint16_t* pDepthData = (uint16_t*)depthFrame->getData();
	float depthValue = (float)pDepthData[y * depthFrameWidth + x];
	if (depthValue == 0) {
		return false;
	}

	// Get the stream profiles for the color and depth frames
	auto colorProfile = frame->getStreamProfile();
	auto depthProfile = depthFrame->getStreamProfile();
	auto extrinsicD2C = depthProfile->getExtrinsicTo(colorProfile);
	// Get the intrinsic and distortion parameters for the color and depth streams
	auto depthIntrinsic = depthProfile->as<ob::VideoStreamProfile>()->getIntrinsic();
	OBPoint2f sourcePixel = { static_cast<float>(x), static_cast<float>(y) };
	OBPoint3f targetPixel = {};
	bool result = ob::CoordinateTransformHelper::transformation2dto3d(
		sourcePixel,
		depthValue,
		depthIntrinsic,
		extrinsicD2C,
		&targetPixel);
	if (!result) {
		return false;
	}

	if (p) {
		p->x() = static_cast<double>(targetPixel.x);
		p->y() = static_cast<double>(targetPixel.y);
		p->z() = static_cast<double>(targetPixel.z);
	}

	return true;
}

int OrbbecvCam::GetWidth() const {
	auto colorFrame = frameSet_->getFrame(OB_FRAME_COLOR);
	auto depthFrame = frameSet_->getFrame(OB_FRAME_DEPTH);
	auto w = depthFrame->as<ob::VideoFrame>()->getWidth();
	auto w1 = colorFrame->as<ob::VideoFrame>()->getWidth();
	return w;
}

int OrbbecvCam::GetHeight() const {
	auto colorFrame = frameSet_->getFrame(OB_FRAME_COLOR);
	auto depthFrame = frameSet_->getFrame(OB_FRAME_DEPTH);
	auto h = depthFrame->as<ob::VideoFrame>()->getHeight();
	auto h1 = colorFrame->as<ob::VideoFrame>()->getHeight();
	return h;
}

void OrbbecvCam::GetFrame(UNMAP_MAT& frames) const
{
	std::unordered_map<STREAM_TYPE, std::shared_ptr<ob::VideoFrame>> mapOBFrame;
	auto fillMapOBFrame = [&]()
		{
			if (!frameSet_)
				return;

			std::shared_ptr<ob::Frame> frame;
			for (auto& it : frames) {
				switch (it.first)
				{
				case STREAM_TYPE::VIDEO:
					frame = frameSet_->getFrame(OB_FRAME_COLOR);
					if (frame) {
						auto cloneFrame = ob::FrameFactory::createFrameFromOtherFrame(frame);
						mapOBFrame[it.first] = cloneFrame->as<ob::VideoFrame>();
					}
					break;
				case STREAM_TYPE::DEPTH:
					frame = frameSet_->getFrame(OB_FRAME_DEPTH);
					if (frame) {
						auto cloneFrame = ob::FrameFactory::createFrameFromOtherFrame(frame);
						mapOBFrame[it.first] = cloneFrame->as<ob::VideoFrame>();
					}
					break;
				case STREAM_TYPE::IR:
					frame = frameSet_->getFrame(OB_FRAME_IR);
					if (frame) {
						auto cloneFrame = ob::FrameFactory::createFrameFromOtherFrame(frame);
						mapOBFrame[it.first] = cloneFrame->as<ob::VideoFrame>();
					}
					break;
				case STREAM_TYPE::LEFT_IR:
					frame = frameSet_->getFrame(OB_FRAME_IR_LEFT);
					if (frame) {
						auto cloneFrame = ob::FrameFactory::createFrameFromOtherFrame(frame);
						mapOBFrame[it.first] = cloneFrame->as<ob::VideoFrame>();
					}
					break;
				case STREAM_TYPE::RIGHT_IR:
					frame = frameSet_->getFrame(OB_FRAME_IR_RIGHT);
					if (frame) {
						auto cloneFrame = ob::FrameFactory::createFrameFromOtherFrame(frame);
						mapOBFrame[it.first] = cloneFrame->as<ob::VideoFrame>();
					}
					break;
				default:
					break;
				}
			}
		};
	fillMapOBFrame();

	for (auto& it : mapOBFrame) {
		if (it.second == nullptr) {
			continue;
		}
		//   ȡ֡    
		auto& colorFr = it.second;
		int width = colorFr->getWidth();
		int height = colorFr->getHeight();
		auto format = colorFr->getFormat();

		//   ȡ    ָ  
		const void* data = colorFr->getData();
		Mat colorFrame;
		//    ݸ ʽ    Mat
		if (format == OB_FORMAT_RGB) {
			// OpenCVĬ  ͨ  ˳  ΪBGR    ת  
			cv::Mat rgb(height, width, CV_8UC3, const_cast<void*>(data));
			cv::cvtColor(rgb, colorFrame, cv::COLOR_RGB2BGR);
		}
		else if (format == OB_FORMAT_BGR) {
			colorFrame = cv::Mat(height, width, CV_8UC3, const_cast<void*>(data)).clone();
		}
		else if (format == OB_FORMAT_YUYV) {
			cv::Mat yuyv(height, width, CV_8UC2, const_cast<void*>(data));
			cv::cvtColor(yuyv, colorFrame, cv::COLOR_YUV2BGR_YUY2);
		}
		else if(format == OB_FORMAT_Y8) {
			colorFrame = cv::Mat(height, width, CV_8UC1, const_cast<void*>(data)).clone();
		}
		else {
			continue;
		}
		frames[it.first] = colorFrame;
	}
}