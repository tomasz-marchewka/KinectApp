#include "MockTracking.h"
#include "Logger.h"
#include <qmessagebox.h>

static Logger &logger = Logger::getInstance();

MockTracking::MockTracking(GLDisplay *display) : TrackingMethod(QString("Mock Tracking"), display)
{
	QPushButton *test1 = new QPushButton("Test 1");
	options << test1 << new QPushButton("Test 2");
	connect(test1, SIGNAL(clicked()), SLOT(button1()));
}

MockTracking::MockTracking(const char *name, GLDisplay *display) : TrackingMethod(name, display)
{
	options << new QPushButton("Mock 1") << new QPushButton("Mock 2") << new QPushButton("Mock 3");
}


MockTracking::~MockTracking()
{
}

bool MockTracking::init()
{
	return true;
}

void MockTracking::draw()
{
		glBegin(GL_QUADS);
		glNormal3f(0, 0, -1);
		glVertex3f(-1, -1, 0);
		glVertex3f(-1, 1, 0);
		glVertex3f(1, 1, 0);
		glVertex3f(1, -1, 0);
		glEnd();
}

QString MockTracking::getErrorMessage()
{
	return QString("MockTracking don't have error :)");
}

QList<QPushButton*> MockTracking::getFunctionList()
{
	return options;
}

void MockTracking::button1()
{
	logger.log("Test log ");
	QMessageBox::information(0, tr("test"), tr("testtestets"), QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel, QMessageBox::Save);

}

void MockTracking::close()
{

}