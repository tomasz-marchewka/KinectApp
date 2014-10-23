#include "TrackingMethod.h"

TrackingMethod::TrackingMethod(const QString &name, GLDisplay *display)
{
	this->display = display;
	this->name = name;
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