#pragma once

#include "TrackingMethod.h"
#include <OpenNI.h>

class OpenNITracking : public TrackingMethod
{
	Q_OBJECT
public:
	OpenNITracking(QString name, GLDisplay *display);
	~OpenNITracking();

	virtual bool init();
	virtual void draw();
	virtual void close();

	void drawDepth();
	void drawIr();
	void draw3dPoints();

public slots:
	void startVideo();
	void startDepth();
	void startIr();
	void start3dPoints();
	void stopTracking();

protected:
	void run();
	void createButtons();

private:
	bool initStream(openni::SensorType sensorType, QString sensorName);

	openni::RGB888Pixel* texMap;
	float *data3d;

	int streamWidth;
	int streamHeight;

	openni::Device device;
	openni::VideoStream videoStream;
	openni::VideoFrameRef videoFrame;
	
};

