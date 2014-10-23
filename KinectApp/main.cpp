#include "kinectapp.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	KinectApp w;
	w.show();
	return a.exec();
}
