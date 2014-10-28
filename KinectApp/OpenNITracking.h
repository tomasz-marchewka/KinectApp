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
	virtual void draw();

public slots:
	void startVideo();
	void stopTracking();

protected:
	void run();

private:
	static const char * methodName;

	void createButtons();

	openni::RGB888Pixel* texMap;
	unsigned int texMapX;
	unsigned int texMapY;

	int colorWidth;
	int colorHeight;

	openni::Device device;
	openni::VideoStream color;
	openni::VideoFrameRef colorFrame;
	
};

