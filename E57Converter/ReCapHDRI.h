#pragma once


/*#include <fstream>
#include <iostream>
#include <memory>
#include <vector>*/
/*#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>*/
/*#include <pcl/io/io.h>
#include <pcl/pcl_macros.h>*/
/*#include <boost/assign.hpp>
#include "nlohmann/json.hpp"
#include "half.hpp"*/
/*
namespace pcl
{
	class ReCapHDRIScan
	{
	public:
		Eigen::Matrix4d worldToScan;
		unsigned int width;
		unsigned int height;
		std::shared_ptr<float> data;

		ReCapHDRIScan(unsigned int width=0, unsigned int height=0, const Eigen::Matrix4d& worldToScan = Eigen::Matrix4d::identity())
			: width(width), height(height), worldToScan(worldToScan)
		{ 
			if ((width * height) > 0)
				data = std::shared_ptr<float>(new float[width * height]);
		}
	};

	class ReCapHDRI
	{
	public:
		std::vector<ReCapHDRIScan> scans;

		ReCapHDRI(const boost::filesystem::path& rcpHDRIPath)
		{
			try
			{
				std::ifstream inFile((rcpHDRIPath / boost::filesystem::path("photos.json")).string());
				if (inFile)
				{
					nlohmann::json j;
					inFile >> j;
					scans.resize(j.size());
					{
						std::stringstream ss;
						ss << "[e57::%s::ReCapHDRI] numScans - " << j.size() << ".\n";
						PCL_INFO(ss.str().c_str(), "ReCapHDRI");
					}

					for (int i = 0 ; i < j.size(); ++i)
					{
						Eigen::Matrix4d worldToScan;
						unsigned int width = j[i]["width"];
						unsigned int height = j[i]["height"];
						std::string format = j[i]["format"];
						if (format != "rgb16f" &&  format != "rgb32f")
							throw PCLException("format: " + format + "is not support");

						for (int m = 0; m < j[i]["calibration"].size(); ++m)
							worldToScan(m%4, m/4) = j[i]["calibration"][m];
						worldToScan = worldToScan.inverse();
						{
							std::stringstream ss;
							ss << "[e57::%s::ReCapHDRI] scans" << i << " - width: " << width << " - height: " << width << " - format: " << format << " - calibration: " << worldToScan[i];
							PCL_INFO(ss.str().c_str(), "ReCapHDRI");
						}

						//
						float fonvy = 65.0f;



						scans[i] = ReCapHDRIScan(width, height, worldToScan);
					}

					
				}
				else
					throw PCLException("Cannot read file: " + (rcpHDRIPath / boost::filesystem::path("photos.json")).string());
			}
			catch (std::exception& ex)
			{
				std::stringstream ss;
				ss << "[e57::%s::ReCapHDRI] Got an std::exception, what=" << ex.what() << ".\n";
				PCL_INFO(ss.str().c_str(), "ReCapHDRI");
			}
			catch (...)
			{
				PCL_INFO("[e57::%s::ReCapHDRI] Got an unknown exception.\n", "ReCapHDRI");
			}
		}
	};
}*/