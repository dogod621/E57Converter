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
#include <opencv2/photo.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui.hpp>

namespace BLK360
{
	class HDRIScan
	{
	public:
		Eigen::Matrix4d worldToScan;
		float fovy;
		cv::Mat image;

		HDRIScan(const unsigned int width=0, const unsigned int height=0, const float fovy = 0, const Eigen::Matrix4d& worldToScan = Eigen::Matrix4d::Identity(), const boost::filesystem::path& fileName = "", const std::string& format = "rgb32f")
			: fovy(fovy), worldToScan(worldToScan)
		{ 
			if ((width * height) > 0)
			{
				std::ifstream file(fileName.string(), std::ios_base::in | std::ios_base::binary);
				if (!file)
					throw pcl::PCLException("Cannot read file: " + fileName.string());

				cv::Mat rawImage;
				if (format == "rgb32f")
				{
					rawImage = cv::Mat(width, height, CV_32FC3);
					file.read(reinterpret_cast<char *>(&rawImage.data), width * height * 3 * sizeof(float));
				}
				else if (format == "rgb16f")
				{
					rawImage = cv::Mat(width, height, CV_32FC3);
					std::vector<half_float::half> rawData(width * height * 3);
					float* data = reinterpret_cast<float *>(rawImage.data);
					file.read(reinterpret_cast<char *>(&rawData[0]), rawData.size() * sizeof(half_float::half));
					for (std::size_t i = 0; i < rawData.size(); ++i)
						data[i] = (float)rawData[i];
				}
				else
					throw pcl::PCLException("format: " + format + "is not support");

				// The image raw data is rotated 90 degrees, and fip left-right
				cv::cvtColor(rawImage, rawImage, cv::COLOR_BGR2RGB);
				cv::rotate(rawImage, rawImage, cv::ROTATE_90_COUNTERCLOCKWISE);
				cv::flip(rawImage, image, 1);

				file.close();
			}
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
						unsigned int width = j[i]["height"]; // The image raw data is rotated 90 degrees, and fip left-right
						unsigned int height = j[i]["width"]; // The image raw data is rotated 90 degrees, and fip left-right
						std::string format = j[i]["format"];

						for (int m = 0; m < j[i]["calibration"].size(); ++m)
							worldToScan(m%4, m/4) = j[i]["calibration"][m];

						worldToScan = worldToScan.inverse();
						{
							std::stringstream ss;
							ss << "[BLK360::%s::HDRI] scans" << i << " - width: " << width << " - height: " << height << " - format: " << format << " - calibration: " << worldToScan;
							PCL_INFO(ss.str().c_str(), "HDRI");
						}

						float fonvy = 53.333333333; // Just get from trying, the original spec is 60, but that is not sutable. 
						std::string url = j[i]["sourceURI"];
						scans[i] = HDRIScan(width, height, fonvy, worldToScan, (filePath / boost::filesystem::path(url).stem()), format);

						//
						cv::Mat ldr;
						cv::Ptr<cv::TonemapDrago> tonemapDrago = cv::createTonemapDrago(1.0, 0.7);
						tonemapDrago->process(scans[i].image, ldr);
						cv::imwrite((filePath / boost::filesystem::path(url).stem().replace_extension(".png") ).string(), ldr * 255);
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