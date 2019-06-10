#pragma once

#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>

namespace BLK360
{
	class HDRIScan
	{
	public:
		Eigen::Matrix4d worldToScan;
		float fovy;
		cv::Mat image;

		HDRIScan(const unsigned int width = 0, const unsigned int height = 0, const float fovy = 0, const Eigen::Matrix4d& worldToScan = Eigen::Matrix4d::Identity(), const boost::filesystem::path& fileName = "", const std::string& format = "rgb32f");
	};

	class HDRI
	{
	public:
		std::vector<HDRIScan> scans;

		HDRI(const boost::filesystem::path& filePath);
	};
}