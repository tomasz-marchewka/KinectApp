#include "TrackingMethod.h"

TrackingMethod::TrackingMethod(const QString &name, GLDisplay *display)
{
	this->display = display;
	this->name = name;
	isRunning = false;
}


TrackingMethod::~TrackingMethod()
{
}

void TrackingMethod::setDisplay(GLDisplay *display)
{
	this->display = display;
}

QString TrackingMethod::getName()
{
	return name;
}

QList<QPushButton *> TrackingMethod::getFunctionList()
{
	return options;
}