#pragma once

#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/pcl_macros.h>
#include <boost/filesystem.hpp>
#include "nlohmann/json.hpp"
#include "half.hpp"

namespace BLK360
{
	class HDRIScan
	{
	public:
		Eigen::Matrix4d worldToScan;
		unsigned int width;
		unsigned int height;
		float fovy;
		std::shared_ptr<float> data;

		HDRIScan(const unsigned int width=0, const unsigned int height=0, const float fovy = 0, const Eigen::Matrix4d& worldToScan = Eigen::Matrix4d::Identity())
			: width(width), height(height), fovy(fovy), worldToScan(worldToScan)
		{ 
			if ((width * height) > 0)
				data = std::shared_ptr<float>(new float[width * height]);
		}
	};

	class HDRI
	{
	public:
		std::vector<HDRIScan> scans;

		HDRI(const boost::filesystem::path& filePath)
		{
			try
			{
				std::ifstream inFile((filePath / boost::filesystem::path("photos.json")).string());
				if (inFile)
				{
					nlohmann::json j;
					inFile >> j;
					scans.resize(j.size());
					{
						std::stringstream ss;
						ss << "[BLK360::%s::HDRI] number of HDRI scans - " << j.size() << ".\n";
						PCL_INFO(ss.str().c_str(), "HDRI");
					}

					for (int i = 0 ; i < j.size(); ++i)
					{
						Eigen::Matrix4d worldToScan;
						unsigned int width = j[i]["width"];
						unsigned int height = j[i]["height"];
						std::string format = j[i]["format"];
						if (format != "rgb16f" &&  format != "rgb32f")
							throw pcl::PCLException("format: " + format + "is not support");

						for (int m = 0; m < j[i]["calibration"].size(); ++m)
							worldToScan(m%4, m/4) = j[i]["calibration"][m];

						worldToScan = worldToScan.inverse();
						{
							std::stringstream ss;
							ss << "[BLK360::%s::HDRI] scans" << i << " - width: " << width << " - height: " << height << " - format: " << format << " - calibration: " << worldToScan;
							PCL_INFO(ss.str().c_str(), "HDRI");
						}

						//
						float fonvy = 53.333333333; // Just get from trying, the original spec is 60, but that is not sutable. 
						scans[i] = HDRIScan(width, height, fonvy, worldToScan);
					}

					
				}
				else
					throw pcl::PCLException("Cannot read file: " + (filePath / boost::filesystem::path("photos.json")).string());
			}
			catch (std::exception& ex)
			{
				std::stringstream ss;
				ss << "[BLK360::%s::HDRI] Got an std::exception, what=" << ex.what() << ".\n";
				PCL_INFO(ss.str().c_str(), "HDRI");
			}
			catch (...)
			{
				PCL_INFO("[BLK360::%s::HDRI] Got an unknown exception.\n", "HDRI");
			}
		}
	};
}