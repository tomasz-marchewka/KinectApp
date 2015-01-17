#include "OpenNITracking.h"
#include "Logger.h"
#include <qfiledialog.h>


static Logger &logger = Logger::getInstance();

const int OPENNI_DEPTH_LEVEL = 10000;

OpenNITracking::OpenNITracking(QString name, GLDisplay *display) : TrackingMethod(name, display)
{
	colorSelected = false;
	depthSelected = false;
	irSelected = false;
	streamWidth = 640;
	streamHeight = 480;
	texMap = new openni::RGB888Pixel[streamWidth * streamHeight];
	data3d = new float[streamWidth * streamHeight * 6];
	createGUI();
}

OpenNITracking::~OpenNITracking()
{
	delete texMap;
	delete data3d;
	data3d = NULL;
	texMap = NULL;
}

void OpenNITracking::createGUI()
{
	//start button
	QPushButton* startColorButton = new QPushButton("Start color");
	connect(startColorButton, SIGNAL(clicked()), SLOT(startVideo()));
	//start depth button
	QPushButton* startDepthButton = new QPushButton("Start depth");
	connect(startDepthButton, SIGNAL(clicked()), SLOT(startDepth()));
	//start ir button
	QPushButton* startIrButton = new QPushButton("Start infrared");
	connect(startIrButton, SIGNAL(clicked()), SLOT(startIr()));
	//start 3d button
	QPushButton* start3dButton = new QPushButton("Start 3d");
	connect(start3dButton, SIGNAL(clicked()), SLOT(start3dPoints()));

	//stop button 
	QPushButton* stopButton = new QPushButton("Stop");
	connect(stopButton, SIGNAL(clicked()), SLOT(stopTracking()));

	//open file button 
	QPushButton* openFileButton = new QPushButton("Open file");
	connect(openFileButton, SIGNAL(clicked()), SLOT(openFile()));

	//record button
	QPushButton* startRecordButton = new QPushButton("Start record");
	connect(startRecordButton, SIGNAL(clicked()), SLOT(startRecord()));

	//check box color stream
	colorCheck = new QCheckBox("Color stream");
	connect(colorCheck, SIGNAL(clicked(bool)), SLOT(colorCheckBoxChange(bool)));

	//check box depth stream
	depthCheck = new QCheckBox("Depth stream");
	connect(depthCheck, SIGNAL(clicked(bool)), SLOT(depthCheckBoxChange(bool)));

	//check box ir stream
	irCheck = new QCheckBox("Ir stream");
	connect(irCheck, SIGNAL(clicked(bool)), SLOT(irCheckBoxChange(bool)));

	options << startColorButton << startDepthButton << startIrButton << start3dButton << stopButton ;
	additionalOptions << openFileButton << startRecordButton << colorCheck << depthCheck << irCheck;
}

bool OpenNITracking::init(const char* dev_uri)
{
	openni::Status status = openni::STATUS_OK;
	QString message;
	status = openni::OpenNI::initialize();
	if (status != openni::STATUS_OK)
	{
		message = "Couldn't initialize openNI!\n";
		message += openni::OpenNI::getExtendedError();
		logger.log(message);
		return false;
	}

	logger.log("OpenNI initalizing...");

	status = device.open(dev_uri);
	if (status != openni::STATUS_OK)
	{
		message = "Device open failed\n";
		message += openni::OpenNI::getExtendedError();
		logger.log(message);
		openni::OpenNI::shutdown();
		logger.log("OpenNI shutdown");
		return false;
	}
	
	logger.log("OpenNI initialized succesful");
	return true;
}

bool OpenNITracking::initStream(openni::SensorType sensorType, QString sensorName, openni::VideoStream* videoStream)
{
		if (!device.isValid())
		{
			if (!init())
				return false;
		}
		openni::Status status = openni::STATUS_OK;
		QString message;

		status = videoStream->create(device, sensorType);
		if (status != openni::STATUS_OK)
		{
			message = "Couldn't find " + sensorName + " stream.\n";
			message += openni::OpenNI::getExtendedError();
			logger.log(message);
			return false;
		}

		status = videoStream->start();
		if (status != openni::STATUS_OK)
		{
			message = "Couldn't start " + sensorName + " stream\n";
			message += openni::OpenNI::getExtendedError();
			logger.log(message);
			videoStream->destroy();
			return false;
		}

		if (!videoStream->isValid())
		{
			message = sensorName + " stream is invalid\n";
			message += openni::OpenNI::getExtendedError();
			logger.log(message);
			videoStream->destroy();
			return false;
		}
		openni::VideoMode videoMode = videoStream->getVideoMode();
		streamWidth = videoMode.getResolutionX();
		streamHeight = videoMode.getResolutionY();

		logger.log("OpenNI " + sensorName + " stream initialized succesful");
		return true;
}

void OpenNITracking::draw()
{
	colorVideoStream.readFrame(&videoFrame);
	if (videoFrame.isValid())
	{
		memcpy(texMap, videoFrame.getData(), videoFrame.getDataSize());
	}
	display->setImage(streamWidth, streamHeight, texMap);
}

void OpenNITracking::drawDepth()
{
	depthVideoStream.readFrame(&videoFrame);
	if (videoFrame.isValid())
	{
		streamWidth = videoFrame.getWidth();
		streamHeight = videoFrame.getHeight();
		int size = streamWidth * streamHeight * 3;

		unsigned char* iterator = (unsigned char*)texMap;
		const openni::DepthPixel* source = reinterpret_cast<const openni::DepthPixel*>(videoFrame.getData());

		for (int i = 0, j = 0; i < size; i+=3, j++)
		{
			unsigned char intensity = (*(source + j) * 256) / OPENNI_DEPTH_LEVEL;
			*(iterator + i) = intensity;
			*(iterator + i + 1) = intensity;
			*(iterator + i + 2) = intensity;
		}
	}
	display->setImage(streamWidth, streamHeight, texMap);
}

void OpenNITracking::drawIr()
{
	irVideoStream.readFrame(&videoFrame);
	if (videoFrame.isValid())
	{
		streamWidth = videoFrame.getWidth();
		streamHeight = videoFrame.getHeight();
		int size = streamWidth * streamHeight * 3;
		unsigned char* iterator = (unsigned char*)texMap;
		const unsigned char* source = reinterpret_cast<const unsigned char*>(videoFrame.getData());

		for (int i = 0, j = 0; i < size; i += 3, j++)
		{
			*(iterator + i) = *(source + j);
			*(iterator + i + 1) = *(source + j);   
			*(iterator + i + 2) = *(source + j);
		}
	}
	display->setImage(streamWidth, streamHeight, texMap);
}


void OpenNITracking::draw3dPoints()
{
	depthVideoStream.readFrame(&videoFrame);
	if (videoFrame.isValid())
	{
		streamWidth = videoFrame.getWidth();
		streamHeight = videoFrame.getHeight();
		int size = streamWidth * streamHeight;

		float* dataIterator = (float*)data3d;
		const openni::DepthPixel* source = reinterpret_cast<const openni::DepthPixel*>(videoFrame.getData());

		for (int i = 0; i < streamHeight; i++)
		{
			for (int j = 0; j < streamWidth; j++)
			{
				int numPix = j + i * streamWidth;
				int depth = *(source + numPix);
				float intensity = depth / (float)OPENNI_DEPTH_LEVEL;

				*(dataIterator++) = intensity;
				*(dataIterator++) = intensity;
				*(dataIterator++) = intensity;
				*(dataIterator++) = (2.0f * j / streamWidth) - 1.0f;
				*(dataIterator++) = -(2.0f * i / streamHeight) + 1.0f;
				*(dataIterator++) = -3.0f * intensity;
			}
		}
	}
	display->setPointCloudData(streamWidth, streamHeight, data3d);
}

void OpenNITracking::startVideo()
{
	streamType = COLOR;
	QThread::start();
}

void OpenNITracking::startDepth()
{
	streamType = DEPTH;
	QThread::start();
}

void OpenNITracking::startIr()
{
	streamType = IR;
	QThread::start();
}

void OpenNITracking::start3dPoints()
{
	streamType = POINTS_3D;
	QThread::start();
}

void OpenNITracking::startRecord()
{
	fileName = "";
	fileName = QFileDialog::getSaveFileName(NULL, tr("Save File"), "", tr("Files (*.oni)"));
	if (fileName != "") 
	{
		streamType = RECORD;
		QThread::start();
	}
}

void OpenNITracking::openFile()
{
	fileName = "";
	fileName = QFileDialog::getOpenFileName(NULL, tr("Open File"), "", tr("Files (*.oni)"));
	if (fileName != "")
	{
		streamType = FROM_FILE;
		QThread::start();
	}
}

void OpenNITracking::stopTracking()
{
	isRunning = false;
}

void OpenNITracking::run()
{
	logger.log("OpenNI thread is running...");
	isRunning = true;
	switch (streamType)
	{
	case TrackingMethod::COLOR:
		if (initStream(openni::SENSOR_COLOR, "color", &colorVideoStream))
		{
			while (isRunning)
				draw();
		}
		break;
	case TrackingMethod::DEPTH:
		if (initStream(openni::SENSOR_DEPTH, "depth", &depthVideoStream))
		{
			while (isRunning)
				drawDepth();
		}
		break;
	case TrackingMethod::IR:
		if (initStream(openni::SENSOR_IR, "infrared", &irVideoStream))
		{
			while (isRunning)
				drawIr();
		}
		break;
	case TrackingMethod::POINTS_3D:
		if (initStream(openni::SENSOR_DEPTH, "depth", &depthVideoStream))
		{
			while (isRunning)
				draw3dPoints();
		}
		break;
	case TrackingMethod::RECORD:
		if (initRecord())
		{
			while (isRunning)
				drawSelected();
			finalizeRecord();
		}
	case TrackingMethod::FROM_FILE:
		if (initFromFile(fileName.toStdString().c_str())) {
			while (isRunning)
				drawSelected();
		}
	default:
		break;
	}
	finalize();
	logger.log("OpenNI thread is stoped");
}

bool OpenNITracking::initRecord()
{
	if (init())
	{
		recorder.create(fileName.toStdString().c_str());

		if (irSelected)
		{
			initStream(openni::SENSOR_IR, "infrared", &irVideoStream);
			recorder.attach(irVideoStream, true);
		}
		if (depthSelected)
		{
			initStream(openni::SENSOR_DEPTH, "depth", &depthVideoStream);
			recorder.attach(depthVideoStream, true);
		}
		if (colorSelected)
		{
			initStream(openni::SENSOR_COLOR, "color", &colorVideoStream);
			recorder.attach(colorVideoStream, true);
		}


		recorder.start();

		return true;
	}
	return false;
}
bool OpenNITracking::initFromFile(const char* file_name)
{
	if (init(file_name))
	{

		if (device.hasSensor(openni::SENSOR_COLOR))
			initStream(openni::SENSOR_COLOR, "color", &colorVideoStream);
		if (device.hasSensor(openni::SENSOR_DEPTH))
			initStream(openni::SENSOR_DEPTH, "depth", &depthVideoStream);
		if (device.hasSensor(openni::SENSOR_IR))
			initStream(openni::SENSOR_IR, "infrared", &irVideoStream);
		return true;
	}
}

void OpenNITracking::finalizeRecord()
{
	recorder.stop();
	recorder.destroy();
}

void OpenNITracking::finalize()
{
	memset(texMap, 0, streamWidth*streamHeight*sizeof(openni::RGB888Pixel));
	memset(data3d, 0, streamWidth*streamHeight*sizeof(float)* 6);
	colorVideoStream.destroy();
	depthVideoStream.destroy();
	irVideoStream.destroy();
	device.close();
	openni::OpenNI::shutdown();
}

void OpenNITracking::drawSelected()
{
	if (colorSelected && colorVideoStream.isValid())
	{
		draw();
	} 
	else if (depthSelected && depthVideoStream.isValid())
	{
		drawDepth();
	}
	else if (irSelected && irVideoStream.isValid())
	{
		drawIr();
	}
}

void OpenNITracking::close()
{
	isRunning = false;
	QThread::wait();
}