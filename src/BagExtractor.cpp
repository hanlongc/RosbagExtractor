#include "BagExtractor.h"

BagExtractor::BagExtractor(std::string bag_file_path)
: mBagFilePath( bag_file_path )
, mRgbTopic( "/asus/rgb/image_raw" )
, mDepthTopic( "/asus/depth/image" )
, mPoseTopic( "/asus/odometry" )
{
}

BagExtractor::~BagExtractor()
{
}

void BagExtractor::Extract()
{
	// First, load bag info
	ros::NodeHandle nodeHandle;

	std::map<double, cv::Mat> rgbImgs;
	std::map<double, cv::Mat> depthImgs;
	std::map<double, CameraPose> poses;

	std::cout << "Loading bag file ..." << std::endl;
	LoadBag(mBagFilePath, rgbImgs, depthImgs, poses);
	std::cout << "Load bag file done" << std::endl;

	if (rgbImgs.empty() || depthImgs.empty() || poses.empty())
	{
		std::cerr << "No images or poses in the bag file" << std::endl;
		return;
	}//end if

	// Save the result
	// Save rgb image to rgb/ and depth image to depth/
	// images are name with the timestamp, and need to save rgb.txt and depth.txt file
	// and the format are the same as tum
	std::cout << "Saving the rgb images..." << std::endl;
	std::ofstream outfile;
	outfile.open("rgb.txt");
	if (!outfile.is_open())
	{
		std::cerr << "Cannot open the rgb.txt file to write" << std::endl;
		return;
	}//end if
	for (std::map<double, cv::Mat>::iterator it=rgbImgs.begin(); it!=rgbImgs.end(); ++it)
	{
		char saveImgPath[100];
		sprintf(saveImgPath, "rgb/%f.png", it->first);
		cv::imwrite(saveImgPath, it->second);
		outfile << std::fixed << std::setprecision(6) << " " << saveImgPath << std::endl;
	}//end for
	outfile.close();
	std::cout << "Save rgb images done" << std::endl;

	std::cout << "Saving the depth images..." << std::endl;
	std::ofstream outfileDepth;
	outfileDepth.open("depth.txt");
	if (!outfileDepth.is_open())
	{
		std::cerr << "Cannot open the depth.txt file to write" << std::endl;
		return;
	}//end if
	for (std::map<double, cv::Mat>::iterator it=depthImgs.begin(); it!=depthImgs.end(); ++it)
	{
		char saveImgPath[100];
		sprintf(saveImgPath, "depth/%f.png", it->first);
		cv::imwrite(saveImgPath, it->second);
		outfileDepth << std::fixed << std::setprecision(6) << " " << saveImgPath << std::endl;
	}//end for
	outfileDepth.close();
	std::cout << "Save depth images done" << std::endl;

	std::cout << "Saving the camera pose ..." << std::endl;
	std::ofstream outfilePose;
	outfilePose.open("camerapose.txt");
	if (!outfilePose.is_open())
	{
		std::cerr << "Cannot open the camerapose.txt file to write" << std::endl;
		return;
	}//end if
	for (std::map<double, CameraPose>::iterator it=poses.begin(); it!=poses.end(); ++it)
	{
		outfilePose << std::fixed << std::setprecision(6) <<  it->first << " " << it->second.mTx << " " << it->second.mTy << " " << it->second.mTz << " ";
		outfilePose << it->second.mQw << " " << it->second.mQx << " " << it->second.mQy << " " << it->second.mQz << std::endl;
	}//end for
	outfilePose.close();
	std::cout << "Save camera pose done" << std::endl;
}

void BagExtractor::LoadBag(std::string& bag_file_path, std::map<double, cv::Mat>& rgbs, std::map<double, cv::Mat>& depths, std::map<double, CameraPose>& poses)
{
	std::cout << "Opening bag file ..." << std::endl;
	try
	{
		mBag.open(bag_file_path, rosbag::bagmode::Read);
	}catch(rosbag::BagIOException ex)
	{
		std::cerr << "Open the bag file: " << bag_file_path << " failed" << std::endl;
		return;
	}//end catch
	std::cout << "Open done" << std::endl;

	std::vector<std::string> topics;
	topics.push_back( mRgbTopic );
	topics.push_back( mDepthTopic );
	topics.push_back( mPoseTopic );

	rosbag::View view(mBag, rosbag::TopicQuery(topics));
	//for (rosbag::View::iterator it=view.begin(); it!=view.end(); ++it)
	BOOST_FOREACH(rosbag::MessageInstance const msgInstance, view)
	{
		if (!ros::ok())
			return;
		
		//rosbag::MessageInstance msgInstance = (*it);
		
		// Deal with /asus/rgb/image_raw topic
		if (msgInstance.getTopic() == mRgbTopic || ("/" + msgInstance.getTopic() == mRgbTopic))
		{
			sensor_msgs::ImageConstPtr msgRgb = msgInstance.instantiate<sensor_msgs::Image>();
			//sensor_msgs::Image::ConstPtr msgRgb = msgInstance.instantiate<sensor_msgs::Image>();
			double tsRgb = msgRgb->header.stamp.toSec();
			cv_bridge::CvImagePtr rgbImgPtr;
			try
			{
				rgbImgPtr = cv_bridge::toCvCopy(*msgRgb, sensor_msgs::image_encodings::TYPE_8UC3);
			}catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}//end catch

			cv::Mat rgbImg = rgbImgPtr->image;
			if (rgbImg.empty())
			{
				std::cerr << "No image in rgbImg" << std::endl;
				return;
			}//end if

			rgbs.insert( std::pair<double, cv::Mat>(tsRgb, rgbImg) );
		}//end if

		// Deal with /asus/depth/image topic
		if (msgInstance.getTopic() == mDepthTopic || ("/" + msgInstance.getTopic() == mDepthTopic))
		{
			sensor_msgs::ImageConstPtr msgDepth = msgInstance.instantiate<sensor_msgs::Image>();
			double tsDepth = msgDepth->header.stamp.toSec(); 
			cv_bridge::CvImagePtr depthImgPtr;
			try
			{
				if (msgDepth->encoding == "16UC1")
					depthImgPtr = cv_bridge::toCvCopy(*msgDepth, sensor_msgs::image_encodings::TYPE_16UC1);
				else if (msgDepth->encoding == "8UC1")
					depthImgPtr = cv_bridge::toCvCopy(*msgDepth, sensor_msgs::image_encodings::TYPE_8UC1);
				else if (msgDepth->encoding == "32FC1")
				{
					depthImgPtr = cv_bridge::toCvCopy(*msgDepth, sensor_msgs::image_encodings::TYPE_32FC1);
				}//end else if
				else
					std::cerr << "Maybe an invalid depth map format" << std::endl;

				if (depths.empty())
					std::cout << "depth encoding: " << msgDepth->encoding << std::endl;
			}catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}//end catch

			cv::Mat depthImg;
			if (msgDepth->encoding == "32FC1")
			{
				cv::Mat tmpImg = depthImgPtr->image;
				tmpImg.convertTo(depthImg, CV_16UC1, 1000.0, 0.0);
			}//end if
			else
				depthImg = depthImgPtr->image;

			if (depthImg.empty())
			{
				std::cerr << "No image in depthImg" << std::endl;
				return;
			}//end if

			depths.insert( std::pair<double, cv::Mat>(tsDepth, depthImg) );
		}//end if

		// Deal with /asus/odometry topic
		if (msgInstance.getTopic() == mPoseTopic || ("/" + msgInstance.getTopic() == mPoseTopic))
		{
			nav_msgs::Odometry::ConstPtr msgOdometry = msgInstance.instantiate<nav_msgs::Odometry>();
			double tsOdometry = msgOdometry->header.stamp.toSec(); 
			
			CameraPose cp;
			cp.mTx = msgOdometry->pose.pose.position.x;
			cp.mTy = msgOdometry->pose.pose.position.y;
			cp.mTz = msgOdometry->pose.pose.position.z;
			cp.mQw = msgOdometry->pose.pose.orientation.w;
			cp.mQx = msgOdometry->pose.pose.orientation.x;
			cp.mQy = msgOdometry->pose.pose.orientation.y;
			cp.mQz = msgOdometry->pose.pose.orientation.z;

			poses.insert( std::pair<double, CameraPose>(tsOdometry, cp) );
		}//end if
	}//end for

	mBag.close();
}
