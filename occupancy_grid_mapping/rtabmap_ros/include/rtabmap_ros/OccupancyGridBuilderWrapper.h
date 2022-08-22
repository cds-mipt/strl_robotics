#ifndef OCCUPANCYGRIDBUILDERNODE_H_
#define OCCUPANCYGRIDBUILDERNODE_H_

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rtabmap/core/OccupancyGrid.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UMutex.h>
#include "rtabmap_ros/CommonDataSubscriber.h"
#include "rtabmap_ros/MsgConversion.h"
#include <colored_occupancy_grid/ColoredOccupancyGrid.h>
#include "time_measurer/time_measurer.h"

#include <memory>
#include <list>
#include <vector>
#include <string>
#include <fstream>
#include <map>

namespace rtabmap_ros {

class OccupancyGridBuilder : public CommonDataSubscriber {
public:
	OccupancyGridBuilder(int argc, char** argv);
	~OccupancyGridBuilder();

private:
	rtabmap::ParametersMap readRtabmapParameters(int argc, char** argv, const ros::NodeHandle& pnh);
	void readParameters(const ros::NodeHandle& pnh);

	void load();
	void loadAssembledOccupancyGrid();
	void loadOccupancyGridCache();
	void save();
	void saveAssembledOccupancyGrid();
	void saveOccupancyGridCache();

	virtual void commonDepthCallback(
				const nav_msgs::OdometryConstPtr& odomMsg,
				const rtabmap_ros::UserDataConstPtr& userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr>& depthMsgs,
				const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
				const std::vector<rtabmap_ros::GlobalDescriptor>& globalDescriptorMsgs = std::vector<rtabmap_ros::GlobalDescriptor>(),
				const std::vector<std::vector<rtabmap_ros::KeyPoint>>& localKeyPoints = std::vector<std::vector<rtabmap_ros::KeyPoint>>(),
				const std::vector<std::vector<rtabmap_ros::Point3f>>& localPoints3d = std::vector<std::vector<rtabmap_ros::Point3f>>(),
				const std::vector<cv::Mat>& localDescriptors = std::vector<cv::Mat>());
	virtual void commonStereoCallback(
				const nav_msgs::OdometryConstPtr& odomMsg,
				const rtabmap_ros::UserDataConstPtr& userDataMsg,
				const cv_bridge::CvImageConstPtr& leftImageMsg,
				const cv_bridge::CvImageConstPtr& rightImageMsg,
				const sensor_msgs::CameraInfo& leftCamInfoMsg,
				const sensor_msgs::CameraInfo& rightCamInfoMsg,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
				const std::vector<rtabmap_ros::GlobalDescriptor>& globalDescriptorMsgs = std::vector<rtabmap_ros::GlobalDescriptor>(),
				const std::vector<rtabmap_ros::KeyPoint>& localKeyPoints = std::vector<rtabmap_ros::KeyPoint>(),
				const std::vector<rtabmap_ros::Point3f>& localPoints3d = std::vector<rtabmap_ros::Point3f>(),
				const cv::Mat& localDescriptors = cv::Mat()) {};
	virtual void commonLaserScanCallback(
				const nav_msgs::OdometryConstPtr& odomMsg,
				const rtabmap_ros::UserDataConstPtr& userDataMsg,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
				const rtabmap_ros::GlobalDescriptor& globalDescriptor = rtabmap_ros::GlobalDescriptor());
	virtual void commonOdomCallback(
				const nav_msgs::OdometryConstPtr& odomMsg,
				const rtabmap_ros::UserDataConstPtr& userDataMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg) {};

	std::unique_ptr<rtabmap::Signature> createSignature(const nav_msgs::OdometryConstPtr& odomMsg,
									   const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
									   const std::vector<cv_bridge::CvImageConstPtr>& depthMsgs,
									   const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
									   const std::vector<std::vector<rtabmap_ros::KeyPoint>>& localKeyPointsMsgs,
									   const std::vector<std::vector<rtabmap_ros::Point3f>>& localPoints3dMsgs,
									   const std::vector<cv::Mat>& localDescriptorsMsgs);
	std::unique_ptr<rtabmap::Signature> createSignature(const nav_msgs::OdometryConstPtr& odomMsg,
									   const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
									   const std::vector<cv_bridge::CvImageConstPtr>& depthMsgs,
									   const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
									   const sensor_msgs::PointCloud2& scan3dMsg,
									   const std::vector<std::vector<rtabmap_ros::KeyPoint>>& localKeyPointsMsgs,
									   const std::vector<std::vector<rtabmap_ros::Point3f>>& localPoints3dMsgs,
									   const std::vector<cv::Mat>& localDescriptorsMsgs);
	std::unique_ptr<rtabmap::Signature> createSignature(const nav_msgs::OdometryConstPtr& odomMsg,
									   const sensor_msgs::PointCloud2& scan3dMsg);

	std::unique_ptr<rtabmap::LaserScan> addRGBToLaserScan(const rtabmap::LaserScan& scan, const cv::Mat& rgb,
										const std::vector<rtabmap::CameraModel>& cameraModels,
										pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud);

	void processNewSignature(const rtabmap::Signature& signature, ros::Time stamp, std::string frame_id);

	void addSignatureToOccupancyGrid(const rtabmap::Signature& signature);
	nav_msgs::OccupancyGrid getOccupancyGridMap();
	
private:
	ros::Publisher occupancyGridPub_;
	ros::Publisher coloredOccupancyGridPub_;
	ros::Publisher coloredCloudPub_;
	tf::TransformListener tfListener_;
	rtabmap::OccupancyGrid occupancyGrid_;
	int nodeId_ = 1;
	std::map<int, rtabmap::Transform> poses_;

	std::string mapPath_;
	bool loadMap_;
	bool saveMap_;
	bool saveAssembledMap_;

	float min_semantic_range_;
	float max_semantic_range_;
	float min_semantic_range_sqr_;
	float max_semantic_range_sqr_;
};

}

#endif /* OCCUPANCYGRIDBUILDERNODE_H_ */
