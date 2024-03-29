#pragma once
#include <QList>
#include <QString>
#include <qthread.h>
#include <gldisplay.h>
#include <qpushbutton.h>

class TrackingMethod : public QThread
{
public:
	TrackingMethod(const QString &name, GLDisplay *display = NULL);
	virtual ~TrackingMethod();

	//virtual bool init() = 0;
	//virtual void draw() = 0;
	virtual void close() = 0;
	virtual QList<QWidget *> getFunctionList();
	virtual QList<QWidget *> getAdditionalFunctionList();

	virtual void setDisplay(GLDisplay *display);
	virtual QString getName();

protected:
	GLDisplay *display;
	QString name;
	QList<QWidget *> options;
	QList<QWidget *> additionalOptions;

	enum TYPES_OF_STREAM { COLOR = 1, DEPTH = 2, IR = 3, SKELETON = 4, POINTS_3D = 5, RECORD = 6, FROM_FILE = 7};
	TYPES_OF_STREAM streamType;

	bool isRunning;
};

