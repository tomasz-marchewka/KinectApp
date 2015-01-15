#include "PLCTracking.h"
#include "Logger.h"

static Logger &logger = Logger::getInstance();

float inline convert_color(uint8_t color)
{
	return color / 255.0f;
}

PLCTracking::PLCTracking(QString name, GLDisplay *display) : TrackingMethod(name, display)//, viewer("Kinect data cloud")
{
	data = new float[DATA_SIZE * 6];
	createButtons();
}


PLCTracking::~PLCTracking()
{
	delete data;
}

void PLCTracking::createButtons()
{
	//start capture button
	QPushButton* startCaptureButton = new QPushButton("Start capture");
	connect(startCaptureButton, SIGNAL(clicked()), SLOT(startCaptureCloud()));

	//stop button 
	QPushButton* stopButton = new QPushButton("Stop capture");
	connect(stopButton, SIGNAL(clicked()), SLOT(stopCapture()));

	options << startCaptureButton << stopButton;
}

bool PLCTracking::init()
{
	//pcl::visualization::CloudViewer& v = this->viewer;
	grabber = new pcl::KinectGrabber();
	function = [this](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud){
		if (this->isRunning){
			boost::shared_ptr < const pcl::PointCloud<pcl::PointXYZRGB>>::element_type* points = cloud.get();
			int size = points->size();
			for (int i = 0, j = 0; i < size; i++, j += 6)
			{
				const pcl::PointXYZRGB point = points->at(i);
				*(data + j) = convert_color(point.r);
				*(data + j + 1) = convert_color(point.g);
				*(data + j + 2) = convert_color(point.b);
				*(data + j + 3) = point.x;
				*(data + j + 4) = point.y;
				*(data + j + 5) = point.z;
			}
			display->setPointCloudData(640, 480, data);
		}
	};
	grabber->registerCallback(function);
	grabber->start();
	return true;
}

void PLCTracking::draw()
{
}

void PLCTracking::startCaptureCloud()
{
	streamType = POINTS_3D;
	QThread::start();
}

void PLCTracking::stopCapture()
{
	isRunning = false;
}

void PLCTracking::run()
{
	logger.log("PCL thread is running...");
	isRunning = true;
	switch (streamType)
	{
	case POINTS_3D:
		if (init())
		{
			logger.log("PCL: cloud capture work.");
			while (isRunning);
			memset(data, 0, sizeof(float)* DATA_SIZE * 6);
		}
		break;
	default:
		break;
	}
	grabber->stop();
	while (grabber->isRunning());
	logger.log("PCL thread is stoped");
}

void PLCTracking::close()
{
	isRunning = false;
	QThread::wait();
}