#pragma once

#include "Common.h"

#include <opencv2/opencv.hpp>

namespace e57
{
	class BLK360HDRIScan
	{
	public:
		Eigen::Matrix4d worldToScan;
		float fovy;
		cv::Mat image;

		BLK360HDRIScan(const unsigned int width = 0, const unsigned int height = 0, const float fovy = 0, const Eigen::Matrix4d& worldToScan = Eigen::Matrix4d::Identity(), const boost::filesystem::path& fileName = "", const std::string& format = "rgb32f");
	};

	class BLK360HDRI
	{
	public:
		std::vector<BLK360HDRIScan> scans;

		BLK360HDRI(const boost::filesystem::path& filePath);
	};
}