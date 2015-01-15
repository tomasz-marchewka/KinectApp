#pragma once

#include "TrackingMethod.h"
#include <OpenNI.h>
#include <qcheckbox.h>

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
	void record();

public slots:
	void startVideo();
	void startDepth();
	void startIr();
	void start3dPoints();
	void stopTracking();
	void startRecord();

	void colorCheckBoxChange(bool state)
	{
		colorRecord = state;
	}
	void depthCheckBoxChange(bool state)
	{
		depthRecord = state;
	}
	void irCheckBoxChange(bool state)
	{
		irRecord = state;
	}

protected:
	void run();
	void createGUI();

private:
	bool initStream(openni::SensorType sensorType, QString sensorName, openni::VideoStream* videoStream);
	bool initRecord();
	void finalizeRecord();
	void finalize();
	bool colorRecord;
	bool depthRecord;
	bool irRecord;
	QString fileName;

	QCheckBox * colorCheck;
	QCheckBox * depthCheck;
	QCheckBox * irCheck;

	openni::RGB888Pixel* texMap;
	float *data3d;

	int streamWidth;
	int streamHeight;

	openni::Recorder recorder;
	openni::Device device;
	//openni::VideoStream videoStream;
	openni::VideoStream colorVideoStream;
	openni::VideoStream depthVideoStream;
	openni::VideoStream irVideoStream;
	openni::VideoFrameRef videoFrame;
	
};

