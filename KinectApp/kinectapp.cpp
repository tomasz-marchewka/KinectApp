#include "kinectapp.h"
#include "Logger.h"
#include "OpenNITracking.h"
#include "KinectSDKTracking.h"
#include "NiTETracking.h"
#include "PCLTracking.h"


static Logger &logger = Logger::getInstance();

KinectApp::KinectApp(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	addTrackingMethod(new OpenNITracking("OpenNI", ui.glDisplay));
	addTrackingMethod(new NiTETracking("NiTE", ui.glDisplay));
	addTrackingMethod(new KinectSDKTracking("KinectSDK", ui.glDisplay));
	addTrackingMethod(new PCLTracking("PCL", ui.glDisplay));
	
	connect(ui.comboBox, SIGNAL(activated(int)), SLOT(selectMethod(int)));
	connect(ui.checkBox, SIGNAL(stateChanged(int)), SLOT(showConsole(int)));
	bool succes = connect(ui.resetButton, SIGNAL(clicked()), SLOT(resetButton()));
	connect(&logger, SIGNAL(logMessage(QString)), SLOT(printOnConsole(QString)));

	selectedMethod = NULL;
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
		selectedMethod->close();
	}
	selectedMethod = methods.at(index);
	if (selectedMethod != NULL)
	{
		showButtons(selectedMethod->getFunctionList());
		showButtons(selectedMethod->getAdditionalFunctionList());
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
		for each (QWidget * element in method->getFunctionList())
		{
			element->setVisible(false);
		}
		for each (QWidget * element in method->getAdditionalFunctionList())
		{
			element->setVisible(false);
		}
	}
}

void KinectApp::showButtons(QList<QWidget *> buttons)
{
	for each (QWidget * button in buttons)
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
			for each (QWidget * element in method->getFunctionList())
			{
				ui.horizontalLayout->addWidget(element);
				element->setVisible(false);
			}
			for each (QWidget * element in method->getAdditionalFunctionList())
			{
				ui.verticalLayout_4->addWidget(element);
				element->setVisible(false);
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

void KinectApp::resetButton()
{
	ui.glDisplay->resetRotation();
}