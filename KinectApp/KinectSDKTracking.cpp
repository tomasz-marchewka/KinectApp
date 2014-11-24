#include "KinectSDKTracking.h"
#include "Logger.h"
#include "KinectSDKParameters.h"

static Logger& logger = Logger::getInstance();
static KinectSDKParameters& videoResolution = KinectSDKParameters::getInstance(NUI_IMAGE_RESOLUTION_640x480);

const char* KinectSDKTracking::methodName = "KinectSDK";

KinectSDKTracking::KinectSDKTracking(GLDisplay* display) : TrackingMethod(methodName, display)
{
	sensor = NULL;
	data = new unsigned char[videoResolution.getWidth() * videoResolution.getHeight() * 3]; //multiply by 3 because we use rgb (3 bytes)
	data3D = new float[videoResolution.getWidth() * videoResolution.getHeight() * 6];
	createButtons();

}


KinectSDKTracking::~KinectSDKTracking()
{
	delete data;
	delete data3D;
	data3D = NULL;
	data = NULL;
}

void KinectSDKTracking::createButtons()
{
	//start button
	QPushButton* startButton = new QPushButton("Start color");
	connect(startButton, SIGNAL(clicked()), SLOT(startVideo()));

	//start depth button
	QPushButton* startDepthButton = new QPushButton("Start depth");
	connect(startDepthButton, SIGNAL(clicked()), SLOT(startDepth()));

	//start ir button
	QPushButton* startIrButton = new QPushButton("Start infrared");
	connect(startIrButton, SIGNAL(clicked()), SLOT(startIr()));

	//start skeleton tracking button
	QPushButton* start3dPoints = new QPushButton("Start 3d ");
	connect(start3dPoints, SIGNAL(clicked()), SLOT(start3dPoints()));

	//stop button 
	QPushButton* stopButton = new QPushButton("Stop");
	connect(stopButton, SIGNAL(clicked()), SLOT(stop()));

	options << startButton << startDepthButton << startIrButton  << start3dPoints << stopButton;
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

	status = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_SKELETON);
	if (status != S_OK)
	{
		logger.log("Couldn't initialize nui!\nError code: ");
		logger.log(QString::number(status));
		return false;
	}
	return true;
}

bool KinectSDKTracking::initSensor(NUI_IMAGE_TYPE sensorType, QString sensorName)
{
	if (init())
	{
		long status;
		status = sensor->NuiImageStreamOpen(sensorType, videoResolution.getResolutionType(), 0, 2, NULL, &stream);
		if (status != S_OK)
		{
			logger.log("Couldn't open " + sensorName + " stream!\nError code: ");
			logger.log(QString::number(status));
			return false;
		}
		
		return true;
	}
	return false;
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

void KinectSDKTracking::startIr()
{
	streamType = IR;
	QThread::start();
}

void KinectSDKTracking::startSkeletonTracking()
{
	streamType = SKELETON;
	QThread::start();
}

void KinectSDKTracking::start3dPoints()
{
	streamType = POINTS_3D;
	QThread::start();
}


void KinectSDKTracking::stop()
{
	isRunning = false;
}

void KinectSDKTracking::run()
{
	isRunning = true;
	logger.log("KinectSDK thread is running...");
	switch (streamType)
	{
	case TrackingMethod::COLOR:
		if (initSensor(NUI_IMAGE_TYPE_COLOR, "color"))
		{
			while (isRunning)
				draw();
			memset(data, 0, videoResolution.getWidth() * videoResolution.getHeight() * 3);
		}
		break;
	case TrackingMethod::DEPTH:
		if (initSensor(NUI_IMAGE_TYPE_DEPTH, "depth"))
		{
			while (isRunning)
				drawDepth();
			memset(data, 0, videoResolution.getWidth() * videoResolution.getHeight() * 3);
		}
		break;
	case TrackingMethod::IR:
		if (initSensor(NUI_IMAGE_TYPE_COLOR_INFRARED, "infrared"))
		{
			while (isRunning)
				drawIr();
			memset(data, 0, videoResolution.getWidth() * videoResolution.getHeight() * 3);
		}
		break;
	case TrackingMethod::SKELETON:
		if (init())
		{
			while (isRunning)
				drawSkeleton();
		}
		break;
	case TrackingMethod::POINTS_3D:
		if (initSensor(NUI_IMAGE_TYPE_DEPTH, "depth"))
		{
			logger.log("3d initialized successfull.");
			while (isRunning)
				draw3dPoints();
			memset(data3D, 0, sizeof(float)* videoResolution.getWidth() * videoResolution.getHeight() * 6);
		}
		break;
	default:
		break;
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
	if (sensor->NuiImageStreamGetNextFrame(stream, 0, &imageFrame) < 0)
	{
		return;
	}
	INuiFrameTexture* texture = imageFrame.pFrameTexture;

	texture->LockRect(0, &lockedRect, 0, NULL);

	if (lockedRect.Pitch != 0)
	{
		const unsigned char* curr = (const unsigned char*)lockedRect.pBits;
		const unsigned char* dataEnd = curr + (videoResolution.getWidth() * videoResolution.getHeight() * 4);
		for (int i = 0, j = 0; i < lockedRect.size; i += 4, j += 3)
		{
			*(data + j) = *(curr + i + 2);
			*(data + j + 1) = *(curr + i + 1);
			*(data + j + 2) = *(curr + i);
		}
	}
	texture->UnlockRect(0);
	sensor->NuiImageStreamReleaseFrame(stream, &imageFrame);

	display->setImage(videoResolution.getWidth(), videoResolution.getHeight(), data);
}

void KinectSDKTracking::drawDepth()
{
	NUI_IMAGE_FRAME imageFrame;
	INuiFrameTexture* texture;
	int nearMode;

	if (sensor->NuiImageStreamGetNextFrame(stream, 0, &imageFrame) < 0)
	{
		return;
	}

	if (!(sensor->NuiImageFrameGetDepthImagePixelFrameTexture(stream, &imageFrame, &nearMode, &texture) < 0))
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

				unsigned char intensity;
				if (depth < minDepth)
					intensity = 0;
				else if (depth > maxDepth)
					intensity = 255;
				else
					intensity = ((depth)* 256) / (maxDepth);

				*(dataIterator++) = intensity;
				*(dataIterator++) = intensity;
				*(dataIterator++) = intensity;

				++currPixel;
			}
		}
		texture->UnlockRect(0);
		texture->Release();
	}

	sensor->NuiImageStreamReleaseFrame(stream, &imageFrame);

	display->setImage(videoResolution.getWidth(), videoResolution.getHeight(), data);
}

void KinectSDKTracking::draw3dPoints()
{
	NUI_IMAGE_FRAME imageFrame;
	INuiFrameTexture* texture;
	int nearMode;

	if (sensor->NuiImageStreamGetNextFrame(stream, 0, &imageFrame) < 0)
	{
		return;
	}

	if (!(sensor->NuiImageFrameGetDepthImagePixelFrameTexture(stream, &imageFrame, &nearMode, &texture) < 0))
	{
		NUI_LOCKED_RECT lockedRect;
		texture->LockRect(0, &lockedRect, NULL, 0);

		if (lockedRect.Pitch != 0)
		{
			int minDepth = (nearMode ? NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MINIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
			int maxDepth = (nearMode ? NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MAXIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

			float* dataIterator = data3D;
			const NUI_DEPTH_IMAGE_PIXEL* currPixel = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL*>(lockedRect.pBits);
			const NUI_DEPTH_IMAGE_PIXEL* endPixel = currPixel + (videoResolution.getWidth() * videoResolution.getHeight());
			int x = videoResolution.getWidth();
			int y = videoResolution.getHeight();
			for (int i = 0; i < y; i++)
			{
				for (int j = 0; j < x; j++)
				{
					int numPix = j + i*x;
					int depth = (currPixel + numPix)->depth;
					float intensity;
					if (depth < minDepth)
						intensity = minDepth / maxDepth;
					else if (depth > maxDepth)
						intensity = 1.0f;
					else
						intensity = depth / (float)maxDepth;

					*(dataIterator++) = intensity;
					*(dataIterator++) = intensity;
					*(dataIterator++) = intensity;
					*(dataIterator++) = (2.0f * j / x) - 1.0f;
					*(dataIterator++) = -(2.0f * i / y) + 1.0f;
					*(dataIterator++) = -intensity;
				}
			}
		}
		texture->UnlockRect(0);
		texture->Release();
	}

	sensor->NuiImageStreamReleaseFrame(stream, &imageFrame);

	display->setData3D(videoResolution.getWidth(), videoResolution.getHeight(), data3D);
}

void KinectSDKTracking::drawIr()
{
	NUI_IMAGE_FRAME imageFrame;
	NUI_LOCKED_RECT lockedRect;
	if (sensor->NuiImageStreamGetNextFrame(stream, 0, &imageFrame) < 0)
	{
		return;
	}
	INuiFrameTexture* texture = imageFrame.pFrameTexture;

	texture->LockRect(0, &lockedRect, 0, NULL);

	if (lockedRect.Pitch != 0)
	{
		int size = videoResolution.getHeight() * videoResolution.getWidth();
		for (int i = 0, j = 0; j < size; i += 3, j++)
		{
			unsigned char intensity = reinterpret_cast<unsigned short*>(lockedRect.pBits)[j] >> 8;
			*(data + i) = intensity;
			*(data + i + 1) = intensity;
			*(data + i + 2) = intensity;
		}
	}
	texture->UnlockRect(0);
	sensor->NuiImageStreamReleaseFrame(stream, &imageFrame);

	display->setImage(videoResolution.getWidth(), videoResolution.getHeight(), data);
}

void KinectSDKTracking::drawSkeleton()
{
	NUI_SKELETON_FRAME skeletonFrame = { 0 };
	if (sensor->NuiSkeletonGetNextFrame(0, &skeletonFrame) < 0)
	{
		return;
	}

	NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[0].eTrackingState;
	if (NUI_SKELETON_TRACKED == trackingState)
	{
		
	}

}

void KinectSDKTracking::close()
{
	isRunning = false;
	QThread::wait();
}