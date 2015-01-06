#pragma once
#include "TrackingMethod.h"
#include <NiTE.h>

class NiTETracking : public TrackingMethod
{
	Q_OBJECT
public:
	NiTETracking(QString name, GLDisplay *display);
	~NiTETracking();

	virtual bool init();
	virtual void draw();
	virtual void close();

	void drawSkeleton();

public slots:
	void startSkeletonTracking();
	void stopTracking();

protected:
	void run();
	void createButtons();
private:

	openni::Device device;
	nite::UserTracker* userTracker;
	nite::UserData* userData;
	float* skeletonData;
};

