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

	//start depth button
	QPushButton* startDepthButton = new QPushButton("Start depth");
	connect(startDepthButton, SIGNAL(clicked()), SLOT(startDepth()));

	options << startButton << stopButton << startDepthButton;
}

bool KinectSDKTracking::init()
{
	int numSensors = 0;
	long status;

	logger.log("KinectSDK initializing...");
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

	status = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
	if (status != S_OK)
	{
		logger.log("Couldn't initialize nui!\nError code: ");
		logger.log(QString::number(status));
		return false;
	}

	status = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, videoResolution.getResolutionType(), 0, 2, NULL, &rgbStream);
	if (status != S_OK)
	{
		logger.log("Couldn't open color stream!\nError code: ");
		logger.log(QString::number(status));
		return false;
	}

	status = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, videoResolution.getResolutionType(), 0, 2, NULL, &depthStream);
	if (status != S_OK)
	{
		logger.log("Couldn't open depth stream!\nError code: ");
		logger.log(QString::number(status));
		return false;
	}

	data = new unsigned char[videoResolution.getWidth() * videoResolution.getHeight() * 3]; //multiply by 3 because we use rgb (3 bytes)
	return true;
}

void KinectSDKTracking::startVideo()
{
	streamType = COLOR;
	QThread::start();
}

void KinectSDKTracking::startDepth()
{
	streamType = DEPTH;
	QThread::start();
}

void KinectSDKTracking::stopVideo()
{
	isRunning = false;
}

void KinectSDKTracking::run()
{
	isRunning = true;
	logger.log("KinectSDK thread is running...");
	if (init())
	{
		switch (streamType)
		{
		case KinectSDKTracking::COLOR:
			while (isRunning)
			{
				draw();
			}
			break;
		case KinectSDKTracking::DEPTH:
			while (isRunning)
			{
				drawDepth();
			}
			break;
		default:
			logger.log("Don't select stream type!");
			break;
		}
		memset(data, 0, videoResolution.getWidth() * videoResolution.getHeight() * 3);
	}
	else
	{
		logger.log("Can't initialize kinectSDK");
	}
	if (sensor)
	{
		sensor->NuiShutdown();
	}
	isRunning = false;
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
		const unsigned char* curr = (const unsigned char*)lockedRect.pBits;
		const unsigned char* dataEnd = curr + (videoResolution.getWidth() * videoResolution.getHeight() * 4);
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

void KinectSDKTracking::drawDepth()
{
	NUI_IMAGE_FRAME imageFrame;
	INuiFrameTexture* texture;
	int nearMode;

	if (sensor->NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame) < 0)
	{
		return;
	}

	if (!(sensor->NuiImageFrameGetDepthImagePixelFrameTexture(depthStream, &imageFrame, &nearMode, &texture) < 0))
	{
		NUI_LOCKED_RECT lockedRect;
		texture->LockRect(0, &lockedRect, NULL, 0);

		if (lockedRect.Pitch != 0)
		{
			int minDepth = (nearMode ? NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MINIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
			int maxDepth = (nearMode ? NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MAXIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

			unsigned char* dataIterator = data;
			const NUI_DEPTH_IMAGE_PIXEL* currPixel = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL*>(lockedRect.pBits);
			const NUI_DEPTH_IMAGE_PIXEL* endPixel = currPixel + (videoResolution.getWidth() * videoResolution.getHeight());

			while (currPixel < endPixel)
			{
				int depth = currPixel->depth;

				//unsigned char intensity = static_cast<unsigned char>(depth >= minDepth && depth <= maxDepth ? depth % 256 : 0);
				unsigned char intensity;
				if (depth < minDepth)
					intensity = 0;
				else if (depth > maxDepth)
					intensity = 255;
				else
					intensity = (depth * 256) / maxDepth;

				*(dataIterator++) = intensity;
				*(dataIterator++) = intensity;
				*(dataIterator++) = intensity;

				++currPixel;
			}
		}
		texture->UnlockRect(0);
		texture->Release();
	}

	sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);

	display->setImage(videoResolution.getWidth(), videoResolution.getHeight(), data);
}


void KinectSDKTracking::close()
{
	isRunning = false;
	QThread::wait();
}