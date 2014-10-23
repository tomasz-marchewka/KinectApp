#ifndef KINECTAPP_H
#define KINECTAPP_H

#include <QtWidgets/QMainWindow>
#include <QList>
#include "ui_kinectapp.h"
#include "TrackingMethod.h"

class KinectApp : public QMainWindow
{
	Q_OBJECT
public:
	KinectApp(QWidget *parent = 0);
	~KinectApp();
	
	void addTrackingMethod(TrackingMethod *method);

public slots:
	void selectMethod(int index);
	void showConsole(int state);
	void printOnConsole(QString message);

private:
	Ui::KinectAppClass ui;
	QList<TrackingMethod*> methods;
	TrackingMethod * selectedMethod;

	void showButtons(QList<QPushButton *> buttons);
	void hideButtons();
	void setButtons();


};

#endif // KINECTAPP_H
