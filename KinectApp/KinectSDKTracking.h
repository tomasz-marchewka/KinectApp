#pragma once

#include "TrackingMethod.h"
#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>

class KinectSDKTracking : public TrackingMethod
{
	Q_OBJECT
public:
	KinectSDKTracking(GLDisplay *display);
	~KinectSDKTracking();

	virtual bool init();
	virtual void draw();
	virtual void close();

public slots:
	void startVideo();
	void stopVideo();

protected:
	void run();

private:
	static const char* methodName;

	unsigned char* data;
	unsigned int textureId;
	void* rgbStream;
	void* depthStream;
	INuiSensor* sensor;

	void createButtons();
};

