#pragma once
#include <NuiApi.h>
class KinectSDKParameters
{
public:

	static KinectSDKParameters& getInstance(NUI_IMAGE_RESOLUTION res)
	{
		static KinectSDKParameters instance1(res, 80, 60);
		static KinectSDKParameters instance2(res, 320, 240);
		static KinectSDKParameters instance3(res, 640, 480);
		static KinectSDKParameters instance4(res, 1280, 960);
		switch (res)
		{
		case NUI_IMAGE_RESOLUTION_80x60 :
			return instance1;
			break;
		case NUI_IMAGE_RESOLUTION_320x240:		
			return instance2;
			break;
		case NUI_IMAGE_RESOLUTION_640x480:
			return instance3;
			break;
		case NUI_IMAGE_RESOLUTION_1280x960:
			return instance4;
			break;
		default:
			return instance3;
			break;
		}
	}

	int getWidth()
	{
		return width;
	}
	int getHeight()
	{
		return height;
	}
	NUI_IMAGE_RESOLUTION getResolutionType()
	{
		return resolutionType;
	}

private:
	int width;
	int height;
	NUI_IMAGE_RESOLUTION resolutionType;

	KinectSDKParameters(NUI_IMAGE_RESOLUTION resType, int width, int height)
	{
		this->width = width;
		this->height = height;
		this->resolutionType = resType;
	}

	~KinectSDKParameters()
	{
	}
};

