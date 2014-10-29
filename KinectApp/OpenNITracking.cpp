#include "OpenNITracking.h"
#include "Logger.h"

#define TEXTURE_SIZE 512
#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

static Logger &logger = Logger::getInstance();
const char* OpenNITracking::methodName = "OpenNI";

OpenNITracking::OpenNITracking(GLDisplay *display) : TrackingMethod(QString(methodName), display)
{
	isRunning = false;
	createButtons();
}


OpenNITracking::~OpenNITracking()
{
}

void OpenNITracking::createButtons()
{
	//start button
	QPushButton* startButton = new QPushButton("Start");
	connect(startButton, SIGNAL(clicked()), SLOT(startVideo()));
	//stop button 
	QPushButton* stopButton = new QPushButton("Stop");
	connect(stopButton, SIGNAL(clicked()), SLOT(stopTracking()));
	options  << startButton << stopButton;
}

bool OpenNITracking::init()
{
	openni::Status status = openni::STATUS_OK;
	QString message;
	status = openni::OpenNI::initialize();
	if (status != openni::STATUS_OK)
	{
		message = "Couldn't initialize openNI!\n";
		message += openni::OpenNI::getExtendedError();
		logger.log(message);
		return false;
	}

	logger.log("OpenNI initalizing...");

	status = device.open(openni::ANY_DEVICE);
	if (status != openni::STATUS_OK)
	{
		message = "Device open failed\n";
		message += openni::OpenNI::getExtendedError();
		logger.log(message);
		openni::OpenNI::shutdown();
		logger.log("OpenNI shutdown");
		return false;
	}

	status = color.create(device, openni::SENSOR_COLOR);
	if (status != openni::STATUS_OK)
	{
		message = "Couldn't find color stream\n";
		message += openni::OpenNI::getExtendedError();
		logger.log(message);
		openni::OpenNI::shutdown();
		logger.log("OpenNI shutdown");
		return false;
	}

	status = color.start();
	if (status != openni::STATUS_OK)
	{
		message = "Couldn't start color stream\n";
		message += openni::OpenNI::getExtendedError();
		logger.log(message);
		color.destroy();
		openni::OpenNI::shutdown();
		logger.log("OpenNI shutdown");
		return false;
	}

	if (!color.isValid())
	{
		message = "Color stream is invalid\n";
		message += openni::OpenNI::getExtendedError();
		logger.log(message);
		color.destroy();
		openni::OpenNI::shutdown();
		logger.log("OpenNI shutdown");
		return false;
	}
	openni::VideoMode colorVideoMode = color.getVideoMode();
	colorWidth = colorVideoMode.getResolutionX();
	colorHeight = colorVideoMode.getResolutionY();
	//texMapX = MIN_CHUNKS_SIZE(colorWidth, TEXTURE_SIZE);
	//texMapY = MIN_CHUNKS_SIZE(colorHeight, TEXTURE_SIZE);
	texMapX = colorWidth;
	texMapY = colorHeight;
	texMap = new openni::RGB888Pixel[texMapX * texMapY];
	
	logger.log("OpenNI initialized succesful");
	return true;
}

void OpenNITracking::draw()
{
	color.readFrame(&colorFrame);
	if (colorFrame.isValid())
	{
		const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)colorFrame.getData();
		openni::RGB888Pixel* pTexRow = texMap + colorFrame.getCropOriginY() * texMapX;
		int rowSize = colorFrame.getStrideInBytes() / sizeof(openni::RGB888Pixel);

		for (int y = 0; y < colorFrame.getHeight(); ++y)
		{
			const openni::RGB888Pixel* pImage = pImageRow;
			openni::RGB888Pixel* pTex = pTexRow + colorFrame.getCropOriginX();

			for (int x = 0; x < colorFrame.getWidth(); ++x, ++pImage, ++pTex)
			{
				*pTex = *pImage;
			}

			pImageRow += rowSize;
			pTexRow += texMapX;
		}
	}

	display->setImage(texMapX, texMapY, texMap);
}

void OpenNITracking::startVideo()
{
	isRunning = true;
	QThread::start();
}

void OpenNITracking::stopTracking()
{
	isRunning = false;
}

void OpenNITracking::run()
{
	static int counter = 0;
	if (init())
	{
		logger.log("OpenNI thread is running...");
		while (isRunning)
		{
			draw();
		}
	}
	else
	{
		logger.log("Can't initialize openNI");
	}
	openni::OpenNI::shutdown();
	logger.log("OpenNI thread is stoped");
}

void OpenNITracking::close()
{
	isRunning = false;
	QThread::wait();
}