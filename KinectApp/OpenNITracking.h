#pragma once

#include "TrackingMethod.h"
#include <OpenNI\OpenNI.h>

class OpenNITracking : public TrackingMethod
{
	Q_OBJECT
public:
	OpenNITracking(GLDisplay *display);
	~OpenNITracking();

	virtual bool init();
	virtual void draw() = 0;
	virtual QList<QPushButton *> getFunctionList() = 0;
private:
	static const char * methodName;

	openni::Device device;
	openni::VideoStream color;

};

