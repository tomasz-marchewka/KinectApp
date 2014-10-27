#include "OpenNITracking.h"
#include "Logger.h"

static Logger &logger = Logger::getInstance();

const char* OpenNITracking::methodName = "OpenNI";

OpenNITracking::OpenNITracking(GLDisplay *display) : TrackingMethod(QString(methodName), display)
{
}


OpenNITracking::~OpenNITracking()
{
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
	
	return true;
}