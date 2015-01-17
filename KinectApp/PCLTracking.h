#pragma once
#include "TrackingMethod.h"
#include <kinect_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include "qcheckbox.h"

#define DATA_SIZE 307200

class PCLTracking : public TrackingMethod
{
	Q_OBJECT
public:
	PCLTracking(QString name, GLDisplay *display);
	~PCLTracking();

	virtual bool init();
	virtual void close();

public slots:
	void startCaptureCloud();
	void stopCapture();
	void openFile();
	void saveFile();

protected:
	void run();
	void createButtons();

private:
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> function;
	QString fileName;

	//pcl::visualization::CloudViewer viewer;
	pcl::Grabber* grabber;
	float *data;
};

