#pragma once

#include "TrackingMethod.h"
#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>

class KinectSDKTracking : public TrackingMethod
{
	Q_OBJECT
public:
	KinectSDKTracking(QString name, GLDisplay *display);
	~KinectSDKTracking();

	virtual bool init();
	virtual void draw();
	virtual void close();

	void drawDepth();
	void drawIr();
	void drawSkeleton();
	void draw3dPoints();

public slots:
	void startVideo();
	void startDepth();
	void startIr();
	void startSkeletonTracking();
	void start3dPoints();
	void stop();

protected:
	void run();

private:
	bool initSensor(NUI_IMAGE_TYPE sensorType, QString sensorName);
	unsigned char* data;
	float *data3D;
	float *skeletonData;
	unsigned int textureId;

	void* stream;
	INuiSensor* sensor;

	void createButtons();
};

