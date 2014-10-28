#pragma once
#include <QList>
#include <QPushButton>
#include <QString>
#include <qthread.h>
#include <gldisplay.h>

class TrackingMethod : public QThread
{
public:
	TrackingMethod(const QString &name, GLDisplay *display = NULL);
	virtual ~TrackingMethod();

	virtual bool init() = 0;
	virtual void draw() = 0;
	virtual QList<QPushButton *> getFunctionList();

	virtual void setDisplay(GLDisplay *display);
	virtual QString getName();

protected:
	GLDisplay *display;
	QString name;
	QList<QPushButton *> options;

	bool isRunning;
};

