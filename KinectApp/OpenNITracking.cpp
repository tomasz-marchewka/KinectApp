#include "OpenNITracking.h"
#include "Logger.h"

static Logger &logger = Logger::getInstance();
const char* OpenNITracking::methodName = "OpenNI";

OpenNITracking::OpenNITracking(GLDisplay *display) : TrackingMethod(QString(methodName), display)
{
	createButtons();
}


OpenNITracking::~OpenNITracking()
{
	delete texMap;
	texMap = NULL;
}

void OpenNITracking::createButtons()
{
	//start button
	QPushButton* startColorButton = new QPushButton("Start color");
	connect(startColorButton, SIGNAL(clicked()), SLOT(startVideo()));
	//start depth button
	QPushButton* startDepthButton = new QPushButton("Start depth");
	connect(startDepthButton, SIGNAL(clicked()), SLOT(startDepth()));
	//start ir button
	QPushButton* startIrButton = new QPushButton("Start ir");
	connect(startIrButton, SIGNAL(clicked()), SLOT(startIr()));

	//stop button 
	QPushButton* stopButton = new QPushButton("Stop");
	connect(stopButton, SIGNAL(clicked()), SLOT(stopTracking()));

	options  << startColorButton << startDepthButton << startIrButton<< stopButton;
}

bool OpenNITracking::init()
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

	status = device.open(openni::ANY_DEVICE);
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

bool OpenNITracking::initColor()
{
	if (init())
	{
		openni::Status status = openni::STATUS_OK;
		QString message;

		status = color.create(device, openni::SENSOR_COLOR);
		if (status != openni::STATUS_OK)
		{
			message = "Couldn't find color stream\n";
			message += openni::OpenNI::getExtendedError();
			logger.log(message);
			openni::OpenNI::shutdown();
			logger.log("OpenNI shutdown");
			return false;
		}

		status = color.start();
		if (status != openni::STATUS_OK)
		{
			message = "Couldn't start color stream\n";
			message += openni::OpenNI::getExtendedError();
			logger.log(message);
			color.destroy();
			openni::OpenNI::shutdown();
			logger.log("OpenNI shutdown");
			return false;
		}

		if (!color.isValid())
		{
			message = "Color stream is invalid\n";
			message += openni::OpenNI::getExtendedError();
			logger.log(message);
			color.destroy();
			openni::OpenNI::shutdown();
			logger.log("OpenNI shutdown");
			return false;
		}
		openni::VideoMode colorVideoMode = color.getVideoMode();
		streamWidth = colorVideoMode.getResolutionX();
		streamHeight = colorVideoMode.getResolutionY();

		texMap = new openni::RGB888Pixel[streamWidth * streamHeight];
		
		logger.log("Color stream initialized succesful");
		return true;
	}
	return false;
}

bool OpenNITracking::initDepth()
{
	if (init())
	{
		openni::Status status = openni::STATUS_OK;
		QString message;

		status = depth.create(device, openni::SENSOR_DEPTH);
		if (status != openni::STATUS_OK)
		{
			message = "Couldn't find depth stream\n";
			message += openni::OpenNI::getExtendedError();
			logger.log(message);
			openni::OpenNI::shutdown();
			logger.log("OpenNI shutdown");
			return false;
		}

		status = depth.start();
		if (status != openni::STATUS_OK)
		{
			message = "Couldn't start depth stream\n";
			message += openni::OpenNI::getExtendedError();
			logger.log(message);
			depth.destroy();
			openni::OpenNI::shutdown();
			logger.log("OpenNI shutdown");
			return false;
		}

		if (!depth.isValid())
		{
			message = "Depth stream is invalid\n";
			message += openni::OpenNI::getExtendedError();
			logger.log(message);
			depth.destroy();
			openni::OpenNI::shutdown();
			logger.log("OpenNI shutdown");
			return false;
		}
		openni::VideoMode depthVideoMode = depth.getVideoMode();
		streamWidth = depthVideoMode.getResolutionX();
		streamHeight = depthVideoMode.getResolutionY();

		texMap = new openni::RGB888Pixel[streamHeight * streamWidth];

		logger.log("Depth stream initialized succesful");
		return true;
	}
	return false;
}

bool OpenNITracking::initIr()
{
	if (init())
	{
		openni::Status status = openni::STATUS_OK;
		QString message;

		status = ir.create(device, openni::SENSOR_IR);
		if (status != openni::STATUS_OK)
		{
			message = "Couldn't find infrared stream\n";
			message += openni::OpenNI::getExtendedError();
			logger.log(message);
			openni::OpenNI::shutdown();
			logger.log("OpenNI shutdown");
			return false;
		}

		status = ir.start();
		if (status != openni::STATUS_OK)
		{
			message = "Couldn't start infrared stream\n";
			message += openni::OpenNI::getExtendedError();
			logger.log(message);
			depth.destroy();
			openni::OpenNI::shutdown();
			logger.log("OpenNI shutdown");
			return false;
		}

		if (!ir.isValid())
		{
			message = "Infrared stream is invalid\n";
			message += openni::OpenNI::getExtendedError();
			logger.log(message);
			depth.destroy();
			openni::OpenNI::shutdown();
			logger.log("OpenNI shutdown");
			return false;
		}
		openni::VideoMode irVideoMode = ir.getVideoMode();
		streamWidth = irVideoMode.getResolutionX();
		streamHeight = irVideoMode.getResolutionY();

		texMap = new openni::RGB888Pixel[streamHeight * streamWidth];

		logger.log("Infrared stream initialized succesful");
		return true;
	}
	return false;
}

void OpenNITracking::draw()
{
	color.readFrame(&colorFrame);
	if (colorFrame.isValid())
	{
		streamWidth = colorFrame.getWidth();
		streamHeight = colorFrame.getHeight();
		int size = streamWidth * streamHeight;
		const openni::RGB888Pixel* source = reinterpret_cast<const openni::RGB888Pixel*>(colorFrame.getData());

		for (int i = 0; i < size; i++)
		{
			*(texMap + i) = *(source + i);
		}
		
	}
	display->setImage(streamWidth, streamHeight, texMap);
}

void OpenNITracking::drawDepth()
{
	depth.readFrame(&depthFrame);
	if (depthFrame.isValid())
	{
		streamWidth = depthFrame.getWidth();
		streamHeight = depthFrame.getHeight();
		int size = streamWidth * streamHeight * 3;
		//int size = depthFrame.getDataSize();
		unsigned char* iterator = (unsigned char*)texMap;
		const openni::DepthPixel* source = reinterpret_cast<const openni::DepthPixel*>(depthFrame.getData());

		for (int i = 0, j = 0; i < size; i+=3, j++)
		{
			unsigned char intensity = (*(source + j) * 256) / 10000;
			*(iterator + i) = intensity;
			*(iterator + i + 1) = intensity;
			*(iterator + i + 2) = intensity;
		}
	}
	display->setImage(streamWidth, streamHeight, texMap);
}

void OpenNITracking::drawIr()
{
	ir.readFrame(&irFrame);
	if (irFrame.isValid())
	{
		streamWidth = irFrame.getWidth();
		streamHeight = irFrame.getHeight();
		int size = streamWidth * streamHeight * 3;
		unsigned char* iterator = (unsigned char*)texMap;
		const unsigned char* source = reinterpret_cast<const unsigned char*>(irFrame.getData());

		for (int i = 0, j = 0; i < size; i += 3, j++)
		{
			*(iterator + i) = *(source + j);
			*(iterator + i + 1) = *(source + j);
			*(iterator + i + 2) = *(source + j);
		}
	}
	display->setImage(streamWidth, streamHeight, texMap);
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
		if (initColor())
		{
			while (isRunning)
				draw();
			memset(texMap, 0, streamWidth*streamHeight*sizeof(openni::RGB888Pixel));
			color.destroy();
		}
		break;
	case TrackingMethod::DEPTH:
		if (initDepth())
		{
			while (isRunning)
				drawDepth();
			memset(texMap, 0, streamWidth*streamHeight*sizeof(openni::RGB888Pixel));
			depth.destroy();
		}
		break;
	case TrackingMethod::IR:
		if (initIr())
		{
			while (isRunning)
				drawIr();
			memset(texMap, 0, streamWidth*streamHeight*sizeof(openni::RGB888Pixel));
			ir.destroy();
		}
		break;
	default:
		break;
	}
	
	device.close();
	openni::OpenNI::shutdown();
	logger.log("OpenNI thread is stoped");
}

void OpenNITracking::close()
{
	isRunning = false;
	QThread::wait();
}