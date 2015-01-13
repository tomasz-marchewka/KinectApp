#pragma once
#include "TrackingMethod.h"
#include <kinect_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#define DATA_SIZE 307200

class PLCTracking : public TrackingMethod
{
	Q_OBJECT
public:
	PLCTracking(QString name, GLDisplay *display);
	~PLCTracking();

	virtual bool init();
	virtual void draw();
	virtual void close();

public slots:
	void startCaptureCloud();
	void stopCapture();

protected:
	void run();
	void createButtons();

private:
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> function;
	//pcl::visualization::CloudViewer viewer;
	pcl::Grabber* grabber;
	float *data;
};

