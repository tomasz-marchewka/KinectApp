// KinectGrabber is pcl::Grabber to retrieve the point cloud data from Kinect v1 using Kinect for Windows SDK v1.x.
// This source code is licensed under the MIT license. Please see the License in License.txt.

#ifndef KINECT_GRABBER
#define KINECT_GRABBER

#define NOMINMAX
#include <Windows.h>
#include <NuiApi.h>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace pcl
{
	struct pcl::PointXYZ;
	struct pcl::PointXYZRGB;
	template <typename T> class pcl::PointCloud;

	class KinectGrabber : public pcl::Grabber
	{
	public:
		KinectGrabber(const int index = 0);
		virtual ~KinectGrabber() throw ();
		virtual void start();
		virtual void stop();
		virtual bool isRunning() const;
		virtual std::string getName() const;
		virtual float getFramesPerSecond() const;

		typedef void (signal_Kinect_PointXYZ)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>&);
		typedef void (signal_Kinect_PointXYZRGB)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>&);

	protected:
		boost::signals2::signal<signal_Kinect_PointXYZ>* signal_PointXYZ;
		boost::signals2::signal<signal_Kinect_PointXYZRGB>* signal_PointXYZRGB;

		pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ(NUI_LOCKED_RECT* depthLockedRect);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB(NUI_LOCKED_RECT* colorLockedRect, NUI_LOCKED_RECT* depthLockedRect);

		boost::thread thread;
		mutable boost::mutex mutex;

		void threadFunction();

		bool quit;
		bool running;

		HRESULT result;
		INuiSensor* sensor;
		INuiCoordinateMapper* mapper;
		HANDLE colorHandle;
		HANDLE depthHandle;

		int width;
		int height;
	};
}
#endif KINECT_GRABBER

