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
	virtual void close();

	void drawDepth();
	void drawIr();

public slots:
	void startVideo();
	void startDepth();
	void startIr();
	void stopTracking();

protected:
	void run();

private:
	static const char* methodName;

	bool initColor();
	bool initDepth();
	bool initIr();
	void createButtons();

	openni::RGB888Pixel* texMap;

	int streamWidth;
	int streamHeight;

	openni::Device device;
	openni::VideoStream color;
	openni::VideoStream depth;
	openni::VideoStream ir;
	
	openni::VideoFrameRef depthFrame;
	openni::VideoFrameRef colorFrame;
	openni::VideoFrameRef irFrame;
	
};

