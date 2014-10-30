#include "KinectSDKTracking.h"
#include "Logger.h"
#include "KinectSDKParameters.h"

static Logger& logger = Logger::getInstance();
static KinectSDKParameters& videoResolution = KinectSDKParameters::getInstance(NUI_IMAGE_RESOLUTION_640x480);

const char* KinectSDKTracking::methodName = "KinectSDK";

KinectSDKTracking::KinectSDKTracking(GLDisplay* display) : TrackingMethod(methodName, display)
{
	data = NULL;
	sensor = NULL;
	createButtons();
}


KinectSDKTracking::~KinectSDKTracking()
{
	delete data;
	data = NULL;
}

void KinectSDKTracking::createButtons()
{
	//start button
	QPushButton* startButton = new QPushButton("Start video");
	connect(startButton, SIGNAL(clicked()), SLOT(startVideo()));
	//stop button 
	QPushButton* stopButton = new QPushButton("Stop video");
	connect(stopButton, SIGNAL(clicked()), SLOT(stopVideo()));

	options << startButton << stopButton;
}

bool KinectSDKTracking::init()
{
	int numSensors = 0;
	long status;

	status = NuiGetSensorCount(&numSensors);
	if (status != S_OK)
	{
		logger.log("Can't get sensors count!\nError code: ");
		logger.log(QString::number(status));
		return false;
	}

	if (numSensors < 1)
	{
		logger.log("KinectSDK sensor not found!");
		return false;
	}

	status = NuiCreateSensorByIndex(0, &sensor);
	if (status != S_OK)
	{
		logger.log("Couldn't create sensor by id 0\nError code: ");
		logger.log(QString::number(status));
		return false;
	}

	status = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR);
	if (status != S_OK)
	{
		logger.log("Couldn't initialize nui!\nError code: ");
		logger.log(QString::number(status));
		return false;
	}

	status = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, videoResolution.getResolutionType(), 0, 2, NULL, &rgbStream);
	if (status != S_OK)
	{
		logger.log("Couldn't open image stream!\nError code: ");
		logger.log(QString::number(status));
		return false;
	}

	
	data = new unsigned char[videoResolution.getWidth() * videoResolution.getHeight() * 3]; //multiply by 3 because we use rgb (3 bytes)
	return true;
}

void KinectSDKTracking::startVideo()
{
	isRunning = true;
	QThread::start();
}

void KinectSDKTracking::stopVideo()
{
	isRunning = false;
}

void KinectSDKTracking::run()
{
	if (init())
	{
		logger.log("KinectSDK thread is running...");
		while (isRunning)
		{
			draw();
		}
	}
	else
	{
		logger.log("Can't initialize openNI");
	}
	if (sensor)
	{
		sensor->NuiShutdown();
	}
	logger.log("KinectSDK thread is stoped");
}

void KinectSDKTracking::draw()
{
	NUI_IMAGE_FRAME imageFrame;
	NUI_LOCKED_RECT lockedRect;
	if (sensor->NuiImageStreamGetNextFrame(rgbStream, 0, &imageFrame) < 0)
	{
		return;
	}
	INuiFrameTexture* texture = imageFrame.pFrameTexture;
	
	texture->LockRect(0, &lockedRect, 0, NULL);

	if (lockedRect.Pitch != 0)
	{
		const BYTE* curr = (const BYTE*)lockedRect.pBits;
		const BYTE* dataEnd = curr + (videoResolution.getWidth() * videoResolution.getHeight() * 4);
		for (int i = 0, j = 0; i < lockedRect.size; i+=4, j +=3) 
		{
			*(data + j) = *(curr + i + 2);
			*(data + j + 1) = *(curr + i + 1);
			*(data + j + 2) = *(curr + i);
		}
	}
	texture->UnlockRect(0);
	sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);

	display->setImage(videoResolution.getWidth(), videoResolution.getHeight(), data);
}


void KinectSDKTracking::close()
{
	isRunning = false;
	QThread::wait();
}