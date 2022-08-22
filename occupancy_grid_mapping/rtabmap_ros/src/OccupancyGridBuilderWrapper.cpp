#include "rtabmap_ros/OccupancyGridBuilderWrapper.h"

bool writeMatBinary(std::fstream& fs, const cv::Mat& out_mat)
{
	if(!fs.is_open()) {
		return false;
	}
	if (out_mat.rows == -1 || out_mat.cols == -1) {
		return false;
	}
	if(out_mat.empty()) {
		int s = 0;
		fs.write((const char*)(&s), sizeof(int));
		return true;
	}
	int type = out_mat.type();
	fs.write((const char*)(&out_mat.rows), sizeof(int));
	fs.write((const char*)(&out_mat.cols), sizeof(int));
	fs.write((const char*)(&type), sizeof(int));
	fs.write((const char*)(out_mat.data), out_mat.elemSize() * out_mat.total());

	return true;
}

bool readMatBinary(std::fstream& fs, cv::Mat& in_mat)
{
	if(!fs.is_open()) {
		return false;
	}
	
	int rows, cols, type;
	fs.read((char*)(&rows), sizeof(int));
	if(rows == 0) {
		return true;
	}
	fs.read((char*)(&cols), sizeof(int));
	fs.read((char*)(&type), sizeof(int));

	in_mat.release();
	in_mat.create(rows, cols, type);
	fs.read((char*)(in_mat.data), in_mat.elemSize() * in_mat.total());

	return true;
}

namespace rtabmap_ros {

rtabmap::ParametersMap OccupancyGridBuilder::readRtabmapParameters(int argc, char** argv, const ros::NodeHandle& pnh) {
	std::string configPath;
	pnh.param("config_path", configPath, configPath);

	//parameters
	rtabmap::ParametersMap parameters;
	uInsert(parameters, rtabmap::Parameters::getDefaultParameters());

	if(!configPath.empty())
	{
		if(UFile::exists(configPath.c_str()))
		{
			ROS_INFO( "%s: Loading parameters from %s", ros::this_node::getName().c_str(), configPath.c_str());
			rtabmap::ParametersMap allParameters;
			rtabmap::Parameters::readINI(configPath.c_str(), allParameters);
			// only update odometry parameters
			for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
			{
				rtabmap::ParametersMap::iterator jter = allParameters.find(iter->first);
				if(jter!=allParameters.end())
				{
					iter->second = jter->second;
				}
			}
		}
		else
		{
			ROS_ERROR( "Config file \"%s\" not found!", configPath.c_str());
		}
	}

	for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		std::string vStr;
		bool vBool;
		int vInt;
		double vDouble;
		if(pnh.getParam(iter->first, vStr))
		{
			ROS_INFO( "Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), vStr.c_str());
			iter->second = vStr;
		}
		else if(pnh.getParam(iter->first, vBool))
		{
			ROS_INFO( "Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), uBool2Str(vBool).c_str());
			iter->second = uBool2Str(vBool);
		}
		else if(pnh.getParam(iter->first, vDouble))
		{
			ROS_INFO( "Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), uNumber2Str(vDouble).c_str());
			iter->second = uNumber2Str(vDouble);
		}
		else if(pnh.getParam(iter->first, vInt))
		{
			ROS_INFO( "Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), uNumber2Str(vInt).c_str());
			iter->second = uNumber2Str(vInt);
		}

		if(iter->first.compare(rtabmap::Parameters::kVisMinInliers()) == 0 && atoi(iter->second.c_str()) < 8)
		{
			ROS_WARN( "Parameter min_inliers must be >= 8, setting to 8...");
			iter->second = uNumber2Str(8);
		}
	}

	rtabmap::ParametersMap argParameters = rtabmap::Parameters::parseArguments(argc, argv);
	for(rtabmap::ParametersMap::iterator iter=argParameters.begin(); iter!=argParameters.end(); ++iter)
	{
		rtabmap::ParametersMap::iterator jter = parameters.find(iter->first);
		if(jter!=parameters.end())
		{
			ROS_INFO( "Update %s parameter \"%s\"=\"%s\" from arguments", ros::this_node::getName().c_str(), iter->first.c_str(), iter->second.c_str());
			jter->second = iter->second;
		}
	}

	// Backward compatibility
	for(std::map<std::string, std::pair<bool, std::string> >::const_iterator iter=rtabmap::Parameters::getRemovedParameters().begin();
		iter!=rtabmap::Parameters::getRemovedParameters().end();
		++iter)
	{
		std::string vStr;
		if(pnh.getParam(iter->first, vStr))
		{
			if(iter->second.first && parameters.find(iter->second.second) != parameters.end())
			{
				// can be migrated
				parameters.at(iter->second.second) = vStr;
				ROS_WARN( "%s: Parameter name changed: \"%s\" -> \"%s\". Please update your launch file accordingly. Value \"%s\" is still set to the new parameter name.",
						ros::this_node::getName().c_str(), iter->first.c_str(), iter->second.second.c_str(), vStr.c_str());
			}
			else
			{
				if(iter->second.second.empty())
				{
					ROS_ERROR( "%s: Parameter \"%s\" doesn't exist anymore!",
							ros::this_node::getName().c_str(), iter->first.c_str());
				}
				else
				{
					ROS_ERROR( "%s: Parameter \"%s\" doesn't exist anymore! You may look at this similar parameter: \"%s\"",
							ros::this_node::getName().c_str(), iter->first.c_str(), iter->second.second.c_str());
				}
			}
		}
	}

	return parameters;
}

void OccupancyGridBuilder::readParameters(const ros::NodeHandle& pnh) {
	pnh.param("map_path", mapPath_, std::string(""));
	pnh.param("load_map", loadMap_, false);
	pnh.param("save_map", saveMap_, false);
	pnh.param("save_assembled_map", saveAssembledMap_, true);
	pnh.param("min_semantic_range", min_semantic_range_, (float)0);
	pnh.param("max_semantic_range", max_semantic_range_, (float)0);
	if (min_semantic_range_ > 0.)
	{
		min_semantic_range_sqr_ = min_semantic_range_ * min_semantic_range_;
	}
	else
	{
		min_semantic_range_sqr_ = 0.;
	}
	if (max_semantic_range_ > 0.)
	{
		max_semantic_range_sqr_ = max_semantic_range_ * max_semantic_range_;
	}
	else
	{
		max_semantic_range_sqr_ = 0.;
	}
}

OccupancyGridBuilder::OccupancyGridBuilder(int argc, char** argv) :
			CommonDataSubscriber(false),
			nodeId_(1) {
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	rtabmap::ParametersMap parameters = readRtabmapParameters(argc, argv, pnh);
	readParameters(pnh);

	occupancyGrid_.parseParameters(parameters);
	occupancyGridPub_ = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);
	coloredOccupancyGridPub_ = nh.advertise<colored_occupancy_grid::ColoredOccupancyGrid>("colored_grid_map", 1);
	coloredCloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("colored_cloud", 1);
	if (mapPath_.empty()) {
		loadMap_ = false;
		saveMap_ = false;
	}
	if (loadMap_) {
		load();
	}
	setupCallbacks(nh, pnh, "DataSubscriber");
}

OccupancyGridBuilder::~OccupancyGridBuilder() {
	if (saveMap_) {
		save();
	}
}

void OccupancyGridBuilder::load() {
	if (saveAssembledMap_) {
		loadAssembledOccupancyGrid();
	} else {
		loadOccupancyGridCache();
	}
}

void OccupancyGridBuilder::loadAssembledOccupancyGrid() {
	MEASURE_BLOCK_TIME(loadAssembledOccupancyGrid);
	std::fstream fs(mapPath_, std::fstream::in | std::fstream::binary | std::fstream::app);
	UASSERT(fs.is_open());
	if (fs.peek() != EOF) {
		float xMin, yMin, cellSize;
		fs.read((char*)(&xMin), sizeof(float));
		fs.read((char*)(&yMin), sizeof(float));
		fs.read((char*)(&cellSize), sizeof(float));
		UASSERT(cellSize == occupancyGrid_.getCellSize());
		cv::Mat map;
		readMatBinary(fs, map);
		rtabmap::Transform pose(xMin, yMin, 0);
		poses_[nodeId_] = pose;
		nodeId_++;
		occupancyGrid_.setMap(map, xMin, yMin, cellSize, poses_);
	}
	fs.close();
}

void OccupancyGridBuilder::loadOccupancyGridCache() {
	MEASURE_BLOCK_TIME(loadOccupancyGridCache);
	std::fstream fs(mapPath_, std::fstream::in | std::fstream::binary | std::fstream::app);
	UASSERT(fs.is_open());
	int maxNodeId = 0;
	while (fs.peek() != EOF) {
		int nodeId;
		fs.read((char*)(&nodeId), sizeof(int));
		if (nodeId > maxNodeId) maxNodeId = nodeId;
		float cellSize;
		fs.read((char*)(&cellSize), sizeof(float));
		UASSERT(cellSize == occupancyGrid_.getCellSize());
		cv::Mat poseMat;
		readMatBinary(fs, poseMat);
		rtabmap::Transform pose(poseMat);
		poses_[nodeId] = pose;
		cv::Mat groundCells, obstacleCells, emptyCells;
		readMatBinary(fs, groundCells);
		readMatBinary(fs, obstacleCells);
		readMatBinary(fs, emptyCells);
		occupancyGrid_.addToCache(nodeId, groundCells, obstacleCells, emptyCells);
	}
	occupancyGrid_.update(poses_);
	nodeId_ = maxNodeId + 1;
	fs.close();
}

void OccupancyGridBuilder::save() {
	if (saveAssembledMap_) {
		saveAssembledOccupancyGrid();
	} else {
		saveOccupancyGridCache();
	}
}

void OccupancyGridBuilder::saveAssembledOccupancyGrid() {
	MEASURE_BLOCK_TIME(saveAssembledOccupancyGrid);
	std::fstream fs(mapPath_, std::fstream::out | std::fstream::binary | std::fstream::trunc);
	UASSERT(fs.is_open());
	float xMin, yMin, cellSize;
	const cv::Mat& map = occupancyGrid_.getMap(xMin, yMin);
	cellSize = occupancyGrid_.getCellSize();
	fs.write((const char*)(&xMin), sizeof(float));
	fs.write((const char*)(&yMin), sizeof(float));
	fs.write((const char*)(&cellSize), sizeof(float));
	writeMatBinary(fs, map);
	fs.close();
}

void OccupancyGridBuilder::saveOccupancyGridCache() {
	MEASURE_BLOCK_TIME(saveOccupancyGridCache);
	std::fstream fs(mapPath_, std::fstream::out | std::fstream::binary | std::fstream::trunc);
	UASSERT(fs.is_open());
	const auto& cache = occupancyGrid_.getCache();
	for (const auto& nodeIdgridCells : cache) {
		int nodeId = nodeIdgridCells.first;
		const auto& gridCells = nodeIdgridCells.second;
		auto poseIt = poses_.find(nodeId);
		if (poseIt == poses_.end()) {
			continue;
		}
		fs.write((const char*)(&nodeId), sizeof(int));
		float cellSize = occupancyGrid_.getCellSize();
		fs.write((const char*)(&cellSize), sizeof(float));
		const cv::Mat& poseMat = poseIt->second.dataMatrix();
		const cv::Mat& groundCells = gridCells.first.first;
		const cv::Mat& obstacleCells = gridCells.first.second;
		const cv::Mat& emptyCells = gridCells.second;
		writeMatBinary(fs, poseMat);
		writeMatBinary(fs, groundCells);
		writeMatBinary(fs, obstacleCells);
		writeMatBinary(fs, emptyCells);
	}
	fs.close();
}

void OccupancyGridBuilder::commonDepthCallback(
				const nav_msgs::OdometryConstPtr& odomMsg,
				const rtabmap_ros::UserDataConstPtr& userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr>& depthMsgs,
				const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
				const std::vector<rtabmap_ros::GlobalDescriptor>& globalDescriptorMsgs /* std::vector<rtabmap_ros::GlobalDescriptor>() */,
				const std::vector<std::vector<rtabmap_ros::KeyPoint>>& localKeyPoints /* std::vector<std::vector<rtabmap_ros::KeyPoint>>() */,
				const std::vector<std::vector<rtabmap_ros::Point3f>>& localPoints3d /* std::vector<std::vector<rtabmap_ros::Point3f>>() */,
				const std::vector<cv::Mat>& localDescriptors /* std::vector<cv::Mat>() */) {
	MEASURE_BLOCK_TIME(commonDepthCallback);
	UDEBUG("\n\nReceived new data");
	UASSERT(isSubscribedToOdom());
	UASSERT(isSubscribedToRGB());
	UASSERT(isSubscribedToDepth());
	std::unique_ptr<rtabmap::Signature> signature_ptr;
	if (isSubscribedToScan3d()) {
		signature_ptr = createSignature(odomMsg, imageMsgs, depthMsgs, cameraInfoMsgs, scan3dMsg,
														  localKeyPoints, localPoints3d, localDescriptors);
	} else {
		signature_ptr = createSignature(odomMsg, imageMsgs, depthMsgs, cameraInfoMsgs,
														  localKeyPoints, localPoints3d, localDescriptors);
	}
	processNewSignature(*signature_ptr, odomMsg->header.stamp, odomMsg->header.frame_id);
}

void OccupancyGridBuilder::commonLaserScanCallback(
			const nav_msgs::OdometryConstPtr& odomMsg,
			const rtabmap_ros::UserDataConstPtr& userDataMsg,
			const sensor_msgs::LaserScan& scanMsg,
			const sensor_msgs::PointCloud2& scan3dMsg,
			const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
			const rtabmap_ros::GlobalDescriptor& globalDescriptor /* rtabmap_ros::GlobalDescriptor() */) {
	MEASURE_BLOCK_TIME(commonLaserScanCallback);
	UDEBUG("\n\nReceived new data");
	UASSERT(isSubscribedToOdom());
	UASSERT(isSubscribedToScan3d());
	std::unique_ptr<rtabmap::Signature> signature_ptr = createSignature(odomMsg, scan3dMsg);
	processNewSignature(*signature_ptr, odomMsg->header.stamp, odomMsg->header.frame_id);
}

std::unique_ptr<rtabmap::Signature> OccupancyGridBuilder::createSignature(const nav_msgs::OdometryConstPtr& odomMsg,
														 const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
														 const std::vector<cv_bridge::CvImageConstPtr>& depthMsgs,
														 const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
														 const std::vector<std::vector<rtabmap_ros::KeyPoint>>& localKeyPointsMsgs,
														 const std::vector<std::vector<rtabmap_ros::Point3f>>& localPoints3dMsgs,
														 const std::vector<cv::Mat>& localDescriptorsMsgs) {
	cv::Mat rgb;
	cv::Mat depth;
	std::vector<rtabmap::CameraModel> cameraModels;
	std::vector<cv::KeyPoint> keypoints;
	std::vector<cv::Point3f> points;
	cv::Mat descriptors;
	bool convertionOk = rtabmap_ros::convertRGBDMsgs(imageMsgs, depthMsgs, cameraInfoMsgs, odomMsg->child_frame_id, "", ros::Time(0),
													 rgb, depth, cameraModels, tfListener_, 0,
													 localKeyPointsMsgs, localPoints3dMsgs, localDescriptorsMsgs,
													 &keypoints, &points, &descriptors);
	UASSERT(convertionOk);
	rtabmap::SensorData data;
	data.setRGBDImage(rgb, depth, cameraModels);
	std::unique_ptr<rtabmap::Signature> signature_ptr(new rtabmap::Signature(data));
	signature_ptr->setPose(rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose));
	return signature_ptr;
}

std::unique_ptr<rtabmap::Signature> OccupancyGridBuilder::createSignature(const nav_msgs::OdometryConstPtr& odomMsg,
														 const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
														 const std::vector<cv_bridge::CvImageConstPtr>& depthMsgs,
														 const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
														 const sensor_msgs::PointCloud2& scan3dMsg,
														 const std::vector<std::vector<rtabmap_ros::KeyPoint>>& localKeyPointsMsgs,
														 const std::vector<std::vector<rtabmap_ros::Point3f>>& localPoints3dMsgs,
														 const std::vector<cv::Mat>& localDescriptorsMsgs) {
	rtabmap::LaserScan scan;
	bool convertionOk = rtabmap_ros::convertScan3dMsg(scan3dMsg, odomMsg->child_frame_id, "", ros::Time(0), scan, tfListener_, 0);
	UASSERT(convertionOk);

	cv::Mat rgb;
	cv::Mat depth;
	std::vector<rtabmap::CameraModel> cameraModels;
	std::vector<cv::KeyPoint> keypoints;
	std::vector<cv::Point3f> points;
	cv::Mat descriptors;
	convertionOk = rtabmap_ros::convertRGBDMsgs(imageMsgs, depthMsgs, cameraInfoMsgs, odomMsg->child_frame_id, "", ros::Time(0),
													 rgb, depth, cameraModels, tfListener_, 0,
													 localKeyPointsMsgs, localPoints3dMsgs, localDescriptorsMsgs,
													 &keypoints, &points, &descriptors);
	UASSERT(convertionOk);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	std::unique_ptr<rtabmap::LaserScan> scanRGB = addRGBToLaserScan(scan, rgb, cameraModels, coloredCloud);
	sensor_msgs::PointCloud2 coloredCloudMsg;
	pcl::toROSMsg(*coloredCloud, coloredCloudMsg);
	coloredCloudMsg.header = scan3dMsg.header;
	coloredCloudPub_.publish(coloredCloudMsg);

	rtabmap::SensorData data;
	data.setRGBDImage(rgb, depth, cameraModels);
	data.setLaserScan(*scanRGB);
	std::unique_ptr<rtabmap::Signature> signature_ptr(new rtabmap::Signature(data));
	signature_ptr->setPose(rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose));
	return signature_ptr;
}

std::unique_ptr<rtabmap::Signature> OccupancyGridBuilder::createSignature(const nav_msgs::OdometryConstPtr& odomMsg,
														 const sensor_msgs::PointCloud2& scan3dMsg) {
	rtabmap::LaserScan scan;
	bool convertionOk = rtabmap_ros::convertScan3dMsg(scan3dMsg, odomMsg->child_frame_id, "", ros::Time(0), scan, tfListener_, 0);
	UASSERT(convertionOk);
	rtabmap::SensorData data;
	data.setId(nodeId_);
	data.setLaserScan(scan);
	std::unique_ptr<rtabmap::Signature> signature_ptr(new rtabmap::Signature(data));
	signature_ptr->setPose(rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose));
	return signature_ptr;
}

std::unique_ptr<rtabmap::LaserScan> OccupancyGridBuilder::addRGBToLaserScan(const rtabmap::LaserScan& scan, const cv::Mat& rgb,
											 const std::vector<rtabmap::CameraModel>& cameraModels,
											 pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud) {
	cv::Mat scanRGB_data = cv::Mat(1, scan.size(),
		CV_32FC(rtabmap::LaserScan::channels(rtabmap::LaserScan::Format::kXYZRGB)));
	UASSERT(scan.format() == rtabmap::LaserScan::Format::kXYZ || scan.format() == rtabmap::LaserScan::Format::kXYZI);
	UASSERT(rgb.type() == CV_8UC3);
	rtabmap::Transform camera2LaserScan = cameraModels[0].localTransform().inverse() * scan.localTransform();
	coloredCloud->clear();
	for (int i = 0; i < scan.size(); i++) {
		float* ptr = scanRGB_data.ptr<float>(0, i);
		ptr[0] = scan.field(i, 0);
		ptr[1] = scan.field(i, 1);
		ptr[2] = scan.field(i, 2);

		cv::Point3f camera_point = rtabmap::util3d::transformPoint(*(cv::Point3f*)(scan.data().ptr<float>(0, i)), camera2LaserScan);
		int u, v;
		cameraModels[0].reproject(camera_point.x, camera_point.y, camera_point.z, u, v);
		float camera_point_range_sqr = camera_point.x * camera_point.x + camera_point.y * camera_point.y +
			camera_point.z * camera_point.z;
		if (cameraModels[0].inFrame(u, v) && camera_point.z > 0 &&
			(min_semantic_range_sqr_ == 0. || camera_point_range_sqr > min_semantic_range_sqr_) &&
			(max_semantic_range_sqr_ == 0. || camera_point_range_sqr < max_semantic_range_sqr_)) {

			int* ptrInt = (int*)ptr;
			std::uint8_t b, g, r;
			const std::uint8_t* bgr_color = rgb.ptr<std::uint8_t>(v, u);
			b = std::max(bgr_color[0], (std::uint8_t)1);
			g = std::max(bgr_color[1], (std::uint8_t)1);
			r = std::max(bgr_color[2], (std::uint8_t)1);
			ptrInt[3] = int(b) | (int(g) << 8) | (int(r) << 16);

			pcl::PointXYZRGB colored_point;
			colored_point.x = ptr[0];
			colored_point.y = ptr[1];
			colored_point.z = ptr[2];
			colored_point.r = r;
			colored_point.g = g;
			colored_point.b = b;
			coloredCloud->push_back(colored_point);
		} else {
			int* ptrInt = (int*)ptr;
			ptrInt[3] = 0;
		}
	}

	std::unique_ptr<rtabmap::LaserScan> scanRGB =
		std::unique_ptr<rtabmap::LaserScan>(new rtabmap::LaserScan(scanRGB_data, scan.maxPoints(), scan.rangeMax(),
		rtabmap::LaserScan::Format::kXYZRGB, scan.localTransform()));
	return scanRGB;
}

void OccupancyGridBuilder::processNewSignature(const rtabmap::Signature& signature, ros::Time stamp, std::string frame_id) {
	addSignatureToOccupancyGrid(signature);
	nav_msgs::OccupancyGrid map = getOccupancyGridMap();
	map.header.stamp = stamp;
	map.header.frame_id = frame_id;
	occupancyGridPub_.publish(map);
	nodeId_++;

	colored_occupancy_grid::ColoredOccupancyGrid colored_map;
	colored_map.header = map.header;
	colored_map.info = map.info;
	colored_map.data = map.data;
	float xMin, yMin;
	cv::Mat colors = occupancyGrid_.getColors(xMin, yMin);
	for (int h = 0; h < colors.rows; h++)
	{
		for (int w = 0; w < colors.cols; w++)
		{
			cv::Vec3b color = colors.at<cv::Vec3b>(h, w);
			colored_map.b.push_back(color[0]);
			colored_map.g.push_back(color[1]);
			colored_map.r.push_back(color[2]);
		}
	}
	coloredOccupancyGridPub_.publish(colored_map);
}

void OccupancyGridBuilder::addSignatureToOccupancyGrid(const rtabmap::Signature& signature) {
	cv::Mat groundCells, obstacleCells, emptyCells;
	cv::Point3f viewPoint;
	occupancyGrid_.createLocalMap(signature, groundCells, obstacleCells, emptyCells, viewPoint);
	occupancyGrid_.addToCache(nodeId_, groundCells, obstacleCells, emptyCells);
	poses_[nodeId_] = signature.getPose();
	occupancyGrid_.update(poses_);
}

nav_msgs::OccupancyGrid OccupancyGridBuilder::getOccupancyGridMap() {
	float gridCellSize = occupancyGrid_.getCellSize();
	float xMin, yMin;
	cv::Mat pixels = occupancyGrid_.getMap(xMin, yMin);
	UASSERT(!pixels.empty());

	nav_msgs::OccupancyGrid map;
	map.info.resolution = gridCellSize;
	map.info.origin.position.x = 0.0;
	map.info.origin.position.y = 0.0;
	map.info.origin.position.z = 0.0;
	map.info.origin.orientation.x = 0.0;
	map.info.origin.orientation.y = 0.0;
	map.info.origin.orientation.z = 0.0;
	map.info.origin.orientation.w = 1.0;

	map.info.width = pixels.cols;
	map.info.height = pixels.rows;
	map.info.origin.position.x = xMin;
	map.info.origin.position.y = yMin;
	map.data.resize(map.info.width * map.info.height);

	memcpy(map.data.data(), pixels.data, map.info.width * map.info.height);
	return map;
}

}
