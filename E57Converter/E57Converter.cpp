#pragma once

#include <fstream>
#include <vector>

#include "E57Utils.h"
#include "E57Converter.h"

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/outofcore/outofcore_impl.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>


//#include "ReCapHDRI.h"

namespace e57
{
	Converter::Converter(const boost::filesystem::path& octPath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution, const std::string& coordSys) : octPath(octPath)
	{
		try
		{
			oct = OCT::Ptr(new OCT(min, max, resolution, octPath / boost::filesystem::path("octRoot.oct_idx"), coordSys));

			// Init scanInfo file
			scanInfo.clear();
			std::ofstream scanFile((octPath / boost::filesystem::path("scan.txt")).string(), std::ios_base::out);
			if (!scanFile)
				throw pcl::PCLException("Create scanFile " + (octPath / boost::filesystem::path("scan.txt")).string() + " failed.");
			scanFile << scanInfo;
			scanFile.close();
		}
		catch (std::exception& ex)
		{
			std::stringstream ss;
			ss << "[e57::%s::Converter] Got an std::exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "Converter");
		}
		catch (...)
		{
			PCL_INFO("[e57::%s::Converter] Got an unknown exception.\n", "Converter");
		}
	}

	Converter::Converter(const boost::filesystem::path& octPath) : octPath(octPath)
	{
		try
		{
			oct = OCT::Ptr(new OCT(octPath / boost::filesystem::path("octRoot.oct_idx"), true));

			// Load scanInfo file
			std::ifstream scanFile((octPath / boost::filesystem::path("scan.txt")).string(), std::ios_base::in);
			if (!scanFile)
				throw pcl::PCLException("Create scanFile " + (octPath / boost::filesystem::path("scan.txt")).string() + " failed.");
			scanFile >> scanInfo;
			scanFile.close();
		}
		catch (std::exception& ex)
		{
			std::stringstream ss;
			ss << "[e57::%s::Converter] Got an std::exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "Converter");
		}
		catch (...)
		{
			PCL_INFO("[e57::%s::Converter] Got an unknown exception.\n", "Converter");
		}
	}

	void Converter::LoadE57(const boost::filesystem::path& e57Path, const double LODSamplePercent, const uint8_t minRGB)
	{
		try
		{
			e57::ImageFile imf(e57Path.string().c_str(), "r");
			e57::VectorNode data3D(imf.root().get("data3D"));
			e57::VectorNode images2D(imf.root().get("images2D"));

			scanInfo.clear();
			for (int64_t scanID = 0; scanID < data3D.childCount(); ++scanID)
			{
				Scan scan;

				// Load E57
				{
					{
						std::stringstream ss;
						ss << "[e57::%s::Converter] Load - scann" << scanID << ".\n";
						PCL_INFO(ss.str().c_str(), "Converter");
					}
					scan.Load(imf, data3D, scanID);

					// Save scan info
					scanInfo[scan.ID]["coodSys"] = CoodSysToStr(scan.coodSys);
					scanInfo[scan.ID]["raeMode"] = RAEModeToStr(scan.raeMode);
					for (int r = 0; r < 4; ++r)
						for (int c = 0; c < 4; ++c)
							scanInfo[scan.ID]["transform"].push_back(scan.transform(r, c));
					scanInfo[scan.ID]["hasPointXYZ"] = scan.hasPointXYZ;
					scanInfo[scan.ID]["hasPointRGB"] = scan.hasPointRGB;
					scanInfo[scan.ID]["hasPointI"] = scan.hasPointI;
					scanInfo[scan.ID]["numPoints"] = scan.numPoints;
				}

				// To OCT
				{
					//
					{
						std::stringstream ss;
						ss << "[e57::%s::Converter] OutOfCoreOctree addPointCloud - scann" << scanID << ".\n";
						PCL_INFO(ss.str().c_str(), "Converter");
					}
					pcl::PointCloud<PointE57>::Ptr scanCloud = pcl::PointCloud<PointE57>::Ptr(new pcl::PointCloud<PointE57>);
					scan.ToPointCloud(*scanCloud);

					// Remove black scan noise
					pcl::PointCloud<PointE57>::Ptr scanCloud_removeBlack = pcl::PointCloud<PointE57>::Ptr(new pcl::PointCloud<PointE57>);
					scanCloud_removeBlack->reserve(scanCloud->size());
					for (std::size_t pi = 0; pi < scanCloud->size(); ++pi)
					{
						PointE57& inP = (*scanCloud)[pi];
						if (inP.r >= minRGB || inP.g >= minRGB || inP.b >= minRGB)
							scanCloud_removeBlack->push_back(inP);
					}
					oct->addPointCloud(scanCloud_removeBlack);

					// Save scan info
					scanInfo[scan.ID]["numValidPoints"] = scanCloud_removeBlack->size();
				}
			}

			// Save info
			{
				std::ofstream scanFile((octPath / boost::filesystem::path("scan.txt")).string(), std::ios_base::out);
				if (!scanFile)
					throw pcl::PCLException("Create scanFile " + (octPath / boost::filesystem::path("scan.txt")).string() + " failed.");
				scanFile << scanInfo;
				scanFile.close();
			}

			// OCT buildLOD
			{
				std::stringstream ss;
				ss << "[e57::%s::Converter] OutOfCoreOctree buildLOD - LODSamplePercent " << LODSamplePercent << ".\n";
				PCL_INFO(ss.str().c_str(), "Converter");
			}
			oct->setSamplePercent(LODSamplePercent);
			oct->buildLOD();
			imf.close();
		}
		catch (e57::E57Exception& ex)
		{
			std::stringstream ss;
			ss << "[e57::%s::Converter] Got an e57::E57Exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "Converter");
		}
		catch (std::exception& ex)
		{
			std::stringstream ss;
			ss << "[e57::%s::Converter] Got an std::exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "Converter");
		}
		catch (...)
		{
			PCL_INFO("[e57::%s::Converter] Got an unknown exception.\n", "Converter");
		}
	}

	void Converter::ReconstructScanImages(pcl::PointCloud<PointPCD>& cloud, const boost::filesystem::path& scanImagePath, const CoodSys coodSys, const RAEMode raeMode, const float fovy, const unsigned int width, const unsigned int height)
	{
		try
		{
			if (!E57_CAN_CONTAIN_SCANID)
				throw pcl::PCLException("You must compile the program with POINT_E57_WITH_SCANID definition to enable the function");

			if (!boost::filesystem::exists(scanImagePath))
			{
				boost::filesystem::create_directory(scanImagePath);
				std::stringstream ss;
				ss << "[e57::%s::ReconstructScanImages] OutOfCoreOctree create directory - " << scanImagePath << ".\n";
				PCL_INFO(ss.str().c_str(), "Converter");
			}

			for (nlohmann::json::iterator it = scanInfo.begin(); it != scanInfo.end(); ++it)
			{
				Eigen::Matrix4d transform;
				nlohmann::json::iterator it2 = (*it)["transform"].begin();
				for (int r = 0; r < 4; ++r)
				{
					for (int c = 0; c < 4; ++c)
					{
						transform(r, c) = *it2;
						++it2;
					}
				}
				std::cout << transform << '\n'; 

				//std::cout << *it << '\n'; 

				//for (nlohmann::json::iterator it2 = (*it)["transform"].begin(); it2 != (*it)["transform"].end(); ++it2)


				/*Eigen::Matrix4d transform;
				std::stringstream ss;
				std::string s = (*it)["transform"];
				ss << s;
				for (int r = 0; r < 4; ++r)
				{
					for (int c = 0; c < 4; ++c)
					{
						double v = 0.0;
						ss >> v;
						transform(r, c) = v;
					}
				}
				std::cout << transform << '\n';*/

				//pcl::PointCloud<PointPCD>::Ptr scanImage(new );
			}
		}
		catch (std::exception& ex)
		{
			std::stringstream ss;
			ss << "[e57::%s::ReconstructScanImages] Got an std::exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "Converter");
		}
		catch (...)
		{
			PCL_INFO("[e57::%s::ReconstructScanImages] Got an unknown exception.\n", "Converter");
		}
	}

	void Converter::BuildLOD(const double sample_percent_arg)
	{
		try
		{
			oct->setSamplePercent(sample_percent_arg);
			oct->buildLOD();
		}
		catch (std::exception& ex)
		{
			std::stringstream ss;
			ss << "[e57::%s::BuildLOD] Got an std::exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "Converter");
		}
		catch (...)
		{
			PCL_INFO("[e57::%s::BuildLOD] Got an unknown exception.\n", "Converter");
		}
	}

	void Converter::ExportToPCD(const double voxelUnit, const unsigned int searchRadiusNumVoxels, const int meanK, const int polynomialOrder, pcl::PointCloud<PointPCD>& out)
	{
		try
		{
			double searchRadius = voxelUnit * searchRadiusNumVoxels;
			pcl::VoxelGrid<PointE57> vf;
			pcl::StatisticalOutlierRemoval<PointE57> olr;
			pcl::MovingLeastSquares<PointE57, PointPCD> mls;
			pcl::NormalEstimation<PointE57, PointPCD> ne;
			pcl::CropBox<PointPCD> cb;
			
			//
			{
				vf.setLeafSize(voxelUnit, voxelUnit, voxelUnit);

				//
				if (meanK > 0)
				{
					olr.setMeanK(meanK);
					olr.setStddevMulThresh(1.0);
				}

				//
				pcl::search::KdTree<PointE57>::Ptr tree(new pcl::search::KdTree<PointE57>());
				if (polynomialOrder > 0)
				{
					mls.setComputeNormals(PCD_CAN_CONTAIN_NORMAL);
					mls.setPolynomialOrder(polynomialOrder);
					mls.setSearchMethod(tree);
					mls.setSearchRadius(searchRadius);
				}
				else
				{
					ne.setSearchMethod(tree);
					ne.setRadiusSearch(searchRadius);
				}
			}

			//
			std::size_t numLeaf = 0;
			{
				OCT::Iterator it(*oct);
				while (*it != nullptr)
				{
					if ((*it)->getNodeType() == pcl::octree::LEAF_NODE)
						numLeaf++;
					it++;
				}
			}

			//
			OCT::Iterator it(*oct);
			out.clear();
			std::size_t leafID = 0;
			while (*it != nullptr)
			{
				if ((*it)->getNodeType() == pcl::octree::LEAF_NODE)
				{
					{
						std::stringstream ss;
						ss << "[e57::%s::ExportToPCD] Process leaf " << leafID << "/" << numLeaf << ".\n";
						PCL_INFO(ss.str().c_str(), "Converter");
					}

					pcl::PointCloud<PointPCD>::Ptr pcdCloud(new pcl::PointCloud<PointPCD>);
					{
						pcl::PointCloud<PointE57>::Ptr e57Cloud(new pcl::PointCloud<PointE57>);

						// Query
						{
							// Query Points
							Eigen::Vector3d minBB;
							Eigen::Vector3d maxBB;
							Eigen::Vector3d extMinBB;
							Eigen::Vector3d extMaxBB;
							Eigen::Vector3d extXYZ(searchRadius, searchRadius, searchRadius);
							(*it)->getBoundingBox(minBB, maxBB);
							std::size_t depth = (*it)->getDepth();
							extMinBB = minBB - extXYZ;
							extMaxBB = maxBB + extXYZ;
							cb.setMin(Eigen::Vector4f(minBB.x(), minBB.y(), minBB.z(), 1.0));
							cb.setMax(Eigen::Vector4f(maxBB.x(), maxBB.y(), maxBB.z(), 1.0));

							std::stringstream ss;
							ss << "[e57::%s::ExportToPCD] Query - minBB, maxBB, depth, extMinBB, extMaxBB: " << minBB << ", " << maxBB << ", " << depth << ", " << extMinBB << ", " << extMaxBB << ".\n";
							PCL_INFO(ss.str().c_str(), "Converter");

							pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
							oct->queryBoundingBox(extMinBB, extMaxBB, depth, blob);
							pcl::fromPCLPointCloud2(*blob, *e57Cloud);
						}

						// Down Sampling
						{
							pcl::PointCloud<PointE57>::Ptr e57Cloud_VF(new pcl::PointCloud<PointE57>);

							PCL_INFO("[e57::%s::ExportToPCD] Down Sampling.\n", "Converter");
							vf.setInputCloud(e57Cloud);
							vf.filter(*e57Cloud_VF);

							std::stringstream ss;
							ss << "[e57::%s::ExportToPCD] Down Sampling - inSize, outSize: " << e57Cloud->size() << ", " << e57Cloud_VF->size() << ".\n";
							PCL_INFO(ss.str().c_str(), "Converter");

							e57Cloud = e57Cloud_VF;
						}

						//Outlier Removal
						if (meanK > 0)
						{
							pcl::PointCloud<PointE57>::Ptr e57Cloud_OLR(new pcl::PointCloud<PointE57>);

							PCL_INFO("[e57::%s::ExportToPCD] Outlier Removal.\n", "Converter");
							olr.setInputCloud(e57Cloud);
							olr.filter(*e57Cloud_OLR);

							std::stringstream ss;
							ss << "[e57::%s::ExportToPCD] Outlier Removal - inSize, outSize: " << e57Cloud->size() << ", " << e57Cloud_OLR->size() << ".\n";
							PCL_INFO(ss.str().c_str(), "Converter");

							e57Cloud = e57Cloud_OLR;
						}

						// Copy to pcdCloud
						pcdCloud->resize(e57Cloud->size());
						for (std::size_t pi = 0; pi < e57Cloud->size(); ++pi)
							(*pcdCloud)[pi].FromPointE57((*e57Cloud)[pi]);

						// Estimat Surface
						if (polynomialOrder > 0)
						{
							PCL_INFO("[e57::%s::ExportToPCD] Estimat Surface.\n", "Converter");
							mls.setInputCloud(e57Cloud);
							mls.process(*pcdCloud);
						}
						else
						{
							PCL_INFO("[e57::%s::ExportToPCD] Estimat Normal.\n", "Converter");
							ne.setInputCloud(e57Cloud);
							ne.compute(*pcdCloud);
						}
					}

					// Crop Box
					{
						pcl::PointCloud<PointPCD>::Ptr pcdCloud_CB(new pcl::PointCloud<PointPCD>);

						PCL_INFO("[e57::%s::ExportToPCD] Crop Box.\n", "Converter");
						cb.setInputCloud(pcdCloud);
						cb.filter(*pcdCloud_CB);

						std::stringstream ss;
						ss << "[e57::%s::ExportToPCD] Crop Box - inSize, outSize: " << pcdCloud->size() << ", " << pcdCloud_CB->size() << ".\n";
						PCL_INFO(ss.str().c_str(), "Converter");

						pcdCloud = pcdCloud_CB;
					}

					// Merge
					out += (*pcdCloud);
					{
						std::stringstream ss;
						ss << "[e57::%s::ExportToPCD] Process leaf " << leafID << " end, final cloud total " << out.size() << " points.\n";
						PCL_INFO(ss.str().c_str(), "Converter");
					}
					leafID++;
				}
				
				it++;
			}
		}
		catch (std::exception& ex)
		{
			std::stringstream ss;
			ss << "[e57::%s::ExportToPCD] Got an std::exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "Converter");
		}
		catch (...)
		{
			PCL_INFO("[e57::%s::ExportToPCD] Got an unknown exception.\n", "Converter");
		}
	}
}
