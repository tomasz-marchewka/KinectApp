#pragma once

#include "TrackingMethod.h"

class MockTracking : public TrackingMethod
{
	Q_OBJECT
public:
	MockTracking(GLDisplay *display = NULL);
	MockTracking(const char *name, GLDisplay *display = NULL);
	virtual ~MockTracking();

	virtual bool init();
	virtual void draw();
	virtual QList<QPushButton*> getFunctionList();
	virtual QString getErrorMessage();

public slots:
	void button1();
};

