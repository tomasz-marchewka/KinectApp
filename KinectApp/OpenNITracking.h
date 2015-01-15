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

	virtual bool init(const char* dev_uri = openni::ANY_DEVICE);
	virtual void draw();
	virtual void close();

	void drawDepth();
	void drawIr();
	void draw3dPoints();
	void drawSelected();

public slots:
	void startVideo();
	void startDepth();
	void startIr();
	void start3dPoints();
	void stopTracking();
	void startRecord();
	void openFile();

	void colorCheckBoxChange(bool state)
	{
		colorSelected = state;
	}
	void depthCheckBoxChange(bool state)
	{
		depthSelected = state;
	}
	void irCheckBoxChange(bool state)
	{
		irSelected = state;
	}

protected:
	void run();
	void createGUI();

private:
	bool initStream(openni::SensorType sensorType, QString sensorName, openni::VideoStream* videoStream);
	bool initRecord();
	bool initFromFile(const char* file_name);
	void finalizeRecord();
	void finalize();

	bool colorSelected;
	bool depthSelected;
	bool irSelected;

	bool colorFromFile;
	bool depthFromFile;
	bool irFromFile;
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

