#include "kinectapp.h"
#include "MockTracking.h"
#include "Logger.h"
#include "OpenNITracking.h"


static Logger &logger = Logger::getInstance();

KinectApp::KinectApp(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	addTrackingMethod(new OpenNITracking(ui.glDisplay));
	addTrackingMethod(new MockTracking(ui.glDisplay));
	addTrackingMethod(new MockTracking("Test mock", ui.glDisplay));
	
	connect(ui.comboBox, SIGNAL(activated(int)), SLOT(selectMethod(int)));
	connect(ui.checkBox, SIGNAL(stateChanged(int)), SLOT(showConsole(int)));
	connect(&logger, SIGNAL(logMessage(QString)), SLOT(printOnConsole(QString)));


	setButtons();
	selectMethod(0);
}

KinectApp::~KinectApp()
{
	selectedMethod->close();
}

void KinectApp::addTrackingMethod(TrackingMethod * method)
{
	methods << method;
	ui.comboBox->addItem(method->getName());
}

void KinectApp::selectMethod(int index)
{
	if (selectedMethod != NULL)
	{
		hideButtons();
	}
	selectedMethod = methods.at(index);
	if (selectedMethod != NULL)
	{
		showButtons(selectedMethod->getFunctionList());
	}
	else
	{
		//TODO: Display an error.
	}

}

void KinectApp::hideButtons()
{
	for each (TrackingMethod * method in methods)
	{
		for each (QPushButton * button in method->getFunctionList())
		{
			button->setVisible(false);
		}
	}
}

void KinectApp::showButtons(QList<QPushButton *> buttons)
{
	for each (QPushButton * button in buttons)
	{
		button->setVisible(true);
	}
}

void KinectApp::setButtons()
{
	static bool isButtonSet = false;
	if (!isButtonSet)
	{
		for each (TrackingMethod * method in methods)
		{
			for each (QPushButton * button in method->getFunctionList())
			{
				ui.horizontalLayout->addWidget(button);
				button->setVisible(false);
			}
		}
		isButtonSet = true;
	}
}

void KinectApp::showConsole(int state)
{
	if (state == Qt::Checked)
	{
		ui.console->setVisible(true);
	} 
	else if (state == Qt::Unchecked)
	{
		ui.console->setVisible(false);
	}
}

void KinectApp::printOnConsole(QString message)
{
	ui.console->appendPlainText(message);
}