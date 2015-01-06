#include "NiTETracking.h"
#include "Logger.h"

static Logger &logger = Logger::getInstance();


NiTETracking::NiTETracking(QString name, GLDisplay *display) : TrackingMethod(name, display)
{
	skeletonData = new float[3 * NITE_JOINT_COUNT];
	userData = NULL;
	userTracker = new nite::UserTracker();
	createButtons();
}


NiTETracking::~NiTETracking()
{
	delete skeletonData;
	skeletonData = NULL;
	delete userData;
	userData = NULL;
	delete userTracker;
	userTracker = NULL;
}


void NiTETracking::createButtons()
{
	//start skeleton button
	QPushButton* startSkeletonButton = new QPushButton("Start skeleton tracking");
	connect(startSkeletonButton, SIGNAL(clicked()), SLOT(startSkeletonTracking()));

	//stop button 
	QPushButton* stopButton = new QPushButton("Stop");
	connect(stopButton, SIGNAL(clicked()), SLOT(stopTracking()));

	options << startSkeletonButton << stopButton;
}

bool NiTETracking::init()
{
	openni::Status OpenNIstatus = openni::STATUS_OK;
	nite::Status NITEstatus = nite::STATUS_OK;
	QString message;

	OpenNIstatus = openni::OpenNI::initialize();
	if (OpenNIstatus != openni::STATUS_OK)
	{
		message = "Couldn't initialize openNI!\n";
		message += openni::OpenNI::getExtendedError();
		logger.log(message);
		return false;
	}

	logger.log("OpenNI initalizing...");

	OpenNIstatus = device.open(openni::ANY_DEVICE);
	if (OpenNIstatus != openni::STATUS_OK)
	{
		message = "Device open failed\n";
		message += openni::OpenNI::getExtendedError();
		logger.log(message);
		openni::OpenNI::shutdown();
		logger.log("OpenNI shutdown");
		return false;
	}

	NITEstatus = nite::NiTE::initialize();
	if (NITEstatus != nite::STATUS_OK)
	{
		message = "NITE: Couldn't initialize NiTE!\n";
		logger.log(message);
		return false;
	}

	NITEstatus = userTracker->create(&device);
	if (NITEstatus != nite::STATUS_OK)
	{
		message = "NITE: Couldn't create user tracker!";
		logger.log(message);
		return false;
	}

	logger.log("NiTE initialized succesful");
	return true;
}

void NiTETracking::draw()
{

}

void NiTETracking::drawSkeleton()
{
	nite::UserTrackerFrameRef userTrackerFrame;
	userTracker->readFrame(&userTrackerFrame);

	bool isUser = false;
	const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
	for (int i = 0; i < users.getSize(); i++)
	{
		const nite::UserData& user = users[i];
		if (user.isNew())
		{
			userTracker->startSkeletonTracking(user.getId());
		}
		if (!user.isVisible())
		{
			break;
		}
		if (nite::SkeletonState::SKELETON_TRACKED == user.getSkeleton().getState())
		{
			isUser = true;
			nite::JointType jType;
			for (int j = 0; j < NITE_JOINT_COUNT; j++)
			{
				jType = static_cast<nite::JointType>(j);
				*(skeletonData + j * 3) = user.getSkeleton().getJoint(jType).getPosition().x / 1000;
				*(skeletonData + j * 3 + 1) = user.getSkeleton().getJoint(jType).getPosition().y / 1000;
				*(skeletonData + j * 3 + 2) = user.getSkeleton().getJoint(jType).getPosition().z / 1000;
				
 			}
			display->setPoints(NITE_JOINT_COUNT, skeletonData);
			break;
		}
	}
	if (!isUser)
	{
		display->setPoints(0, NULL);
	}
}

void NiTETracking::startSkeletonTracking()
{
	streamType = SKELETON;
	QThread::start();
}

void NiTETracking::stopTracking()
{
	isRunning = false;
}

void NiTETracking::run()
{
	logger.log("NiTE thread is running...");
	isRunning = true;
	switch (streamType)
	{
	case SKELETON:
		if (init())
		{
			logger.log("NiTE: skeleton tracking work.");
			while (isRunning)
				drawSkeleton();
			memset(skeletonData, 0, sizeof(float)* NITE_JOINT_COUNT * 3);
		}
		break;
	default:
		break;
	}
	userTracker->destroy();
	device.close();
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
	logger.log("NiTE thread is stoped");
}

void NiTETracking::close()
{
	isRunning = false;
	QThread::wait();
}