#pragma once

#include <string>
#include <map>
#include <fstream>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "defs.h"

class BagExtractor
{
public:
	BagExtractor(std::string bag_file_path);
	~BagExtractor();

	void Extract();
	void LoadBag(std::string& bag_file_path, std::map<double, cv::Mat>& rgbs, std::map<double, cv::Mat>& depths, std::map<double, CameraPose>& poses);

private:
	std::string mBagFilePath;

	rosbag::Bag mBag;

	// The topics
	std::string mRgbTopic;
	std::string mDepthTopic;
	std::string mPoseTopic;
};
