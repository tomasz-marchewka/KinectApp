#pragma once
#include "TrackingMethod.h"
#include <NiTE.h>

class NiTETracking : public TrackingMethod
{
	Q_OBJECT
public:
	NiTETracking(QString name, GLDisplay *display);
	~NiTETracking();

	virtual bool init(const char* file = openni::ANY_DEVICE);
	virtual void draw();
	virtual void close();

	void drawSkeleton();

public slots:
	void startSkeletonTracking();
	void stopTracking();
	void openFile();

protected:
	void run();
	void createButtons();
private:
	QString fileName;
	openni::Device device;
	nite::UserTracker* userTracker;
	nite::UserData* userData;
	float* skeletonData;
};

