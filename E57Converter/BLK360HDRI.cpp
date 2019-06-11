#include <fstream>
#include <iostream>
#include <vector>

#include "BLK360HDRI.h"

#include "nlohmann/json.hpp"
#include "half.hpp"

namespace e57
{
	BLK360HDRIScan::BLK360HDRIScan(const unsigned int width, const unsigned int height, const float fovy, const Eigen::Matrix4d& worldToScan, const boost::filesystem::path& fileName, const std::string& format)
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

	BLK360HDRI::BLK360HDRI(const boost::filesystem::path& filePath)
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
					ss << "[e57::%s::BLK360HDRI] number of HDRI scans - " << j.size() << ".\n";
					PCL_INFO(ss.str().c_str(), "BLK360HDRI");
				}

				for (int i = 0; i < j.size(); ++i)
				{
					Eigen::Matrix4d worldToScan;
					unsigned int width = j[i]["height"]; // The image raw data is rotated 90 degrees, and fip left-right
					unsigned int height = j[i]["width"]; // The image raw data is rotated 90 degrees, and fip left-right
					std::string format = j[i]["format"];

					for (int m = 0; m < j[i]["calibration"].size(); ++m)
						worldToScan(m % 4, m / 4) = j[i]["calibration"][m];

					worldToScan = worldToScan.inverse();
					{
						std::stringstream ss;
						ss << "[e57::%s::BLK360HDRI] scans" << i << " - width: " << width << " - height: " << height << " - format: " << format << " - calibration: " << worldToScan;
						PCL_INFO(ss.str().c_str(), "BLK360HDRI");
					}

					float fonvy = 53.333333333; // Just get from trying, the original spec is 60, but that is not sutable. 
					std::string url = j[i]["sourceURI"];
					scans[i] = BLK360HDRIScan(width, height, fonvy, worldToScan, (filePath / boost::filesystem::path(url).stem()), format);

					//
					cv::Mat ldr;
					cv::Ptr<cv::TonemapDrago> tonemapDrago = cv::createTonemapDrago(1.0, 0.7);
					tonemapDrago->process(scans[i].image, ldr);
					cv::imwrite((filePath / boost::filesystem::path(url).stem().replace_extension(".png")).string(), ldr * 255);
				}
			}
			else
				throw pcl::PCLException("Cannot read file: " + (filePath / boost::filesystem::path("photos.json")).string());
		}
		catch (std::exception& ex)
		{
			std::stringstream ss;
			ss << "[e57::%s::BLK360HDRI] Got an std::exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "BLK360HDRI");
		}
		catch (...)
		{
			PCL_INFO("[e57::%s::BLK360HDRI] Got an unknown exception.\n", "BLK360HDRI");
		}
	}
}