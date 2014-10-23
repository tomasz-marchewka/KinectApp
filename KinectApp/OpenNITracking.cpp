#include "OpenNITracking.h"

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
	
	status = openni::OpenNI::initialize();
	if (status != openni::STATUS_OK)
	{
		errorMessage = "Couldn't initialize openNI!\n";
		errorMessage += openni::OpenNI::getExtendedError();
		return false;
	}

	status = device.open(openni::ANY_DEVICE);
	if (status != openni::STATUS_OK)
	{
		errorMessage = "Device open failed\n";
		errorMessage += openni::OpenNI::getExtendedError();
		openni::OpenNI::shutdown();
		return false;
	}

	status = color.create(device, openni::SENSOR_COLOR);
	if (status != openni::STATUS_OK)
	{
		errorMessage = "Couldn't find color stream\n";
		errorMessage += openni::OpenNI::getExtendedError();
		openni::OpenNI::shutdown();
		return false;
	}

	status = color.start();
	if (status != openni::STATUS_OK)
	{
		errorMessage = "Couldn't start color stream\n";
		errorMessage += openni::OpenNI::getExtendedError();
		color.destroy();
		openni::OpenNI::shutdown();
		return false;
	}
	
	return true;
}

QString OpenNITracking::getErrorMessage()
{
	return errorMessage;
}