#include "PCLTracking.h"
#include "Logger.h"
#include "qfiledialog.h"
#include <pcl/io/pcd_io.h>


static Logger &logger = Logger::getInstance();

float inline convert_color(uint8_t color)
{
	return color / 255.0f;
}

PCLTracking::PCLTracking(QString name, GLDisplay *display) : TrackingMethod(name, display)//, viewer("Kinect data cloud")
{
	fileName = "";
	saveData = false;
	isPause = false;
	data = new float[DATA_SIZE * 6];
	createButtons();
}


PCLTracking::~PCLTracking()
{
	delete data;
}

void PCLTracking::createButtons()
{
	//start capture button
	QPushButton* startCaptureButton = new QPushButton("Start capture");
	connect(startCaptureButton, SIGNAL(clicked()), SLOT(startCaptureCloud()));

	//pause button 
	QPushButton* pauseButton = new QPushButton("Pause");
	connect(pauseButton, SIGNAL(clicked()), SLOT(pause()));

	//stop button 
	QPushButton* stopButton = new QPushButton("Stop capture");
	connect(stopButton, SIGNAL(clicked()), SLOT(stopCapture()));

	//open file button 
	QPushButton* openFileButton = new QPushButton("Open cloud");
	connect(openFileButton, SIGNAL(clicked()), SLOT(openFile()));

	//record cloud button 
	QPushButton* saveFileButton = new QPushButton("Save cloud");
	connect(saveFileButton, SIGNAL(clicked()), SLOT(saveFile()));

	options << startCaptureButton << pauseButton << stopButton;
	additionalOptions << openFileButton << saveFileButton;
}

bool PCLTracking::init()
{
	//pcl::visualization::CloudViewer& v = this->viewer;
	grabber = new pcl::KinectGrabber();
	function = [this](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud){
		if (this->isRunning && !isPause){
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
			if (saveData && fileName != "")
			{
				pcl::io::savePCDFileBinary(fileName.toStdString(), *cloud);
				fileName = "";
				saveData = false;
			}
			display->setPointCloudData(640, 480, data);
		}
	};
	grabber->registerCallback(function);
	grabber->start();
	return true;
}


void PCLTracking::startCaptureCloud()
{
	streamType = POINTS_3D;
	QThread::start();
}

void PCLTracking::pause()
{
	isPause = !isPause;
}

void PCLTracking::stopCapture()
{
	isRunning = false;
}

void PCLTracking::saveFile()
{
	fileName = QFileDialog::getSaveFileName(NULL, tr("Save File"), "", tr("Files (*.pcd)"));
}

void PCLTracking::openFile()
{
	fileName = QFileDialog::getOpenFileName(NULL, tr("Open File"), "", tr("Files (*.pcd)"));
	if (isRunning)
		close();
	streamType = FROM_FILE;
	QThread::start();
}

void PCLTracking::drawFromFile(QString fileName)
{
	if (fileName != "")
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile(fileName.toStdString(), *cloud) == -1)
		{
			logger.log("Couldn't read file" + fileName);
			isRunning = false;
			return;
		}
		int size = cloud->size();
		for (int i = 0, j = 0; i < size; i++, j += 6)
		{
			const pcl::PointXYZRGB point = cloud->at(i);
			*(data + j) = convert_color(point.r);
			*(data + j + 1) = convert_color(point.g);
			*(data + j + 2) = convert_color(point.b);
			*(data + j + 3) = point.x;
			*(data + j + 4) = point.y;
			*(data + j + 5) = point.z;
		}
		display->setPointCloudData(640, 480, data);
	}
}

void PCLTracking::run()
{
	logger.log("PCL thread is running...");
	isRunning = true;
	switch (streamType)
	{
	case POINTS_3D:
		if (init())
		{
			logger.log("PCL cloud capture work.");
			while (isRunning);
		}
		grabber->stop();
		while (grabber->isRunning());
		break;
	case FROM_FILE:
		drawFromFile(fileName);
		while (isRunning);
		break;
	default:
		break;
	}
	memset(data, 0, sizeof(float)* DATA_SIZE * 6);
	logger.log("PCL thread is stoped.");
}

void PCLTracking::close()
{
	isRunning = false;
	QThread::wait();
}