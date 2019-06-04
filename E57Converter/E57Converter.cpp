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
#include <pcl/pcl_macros.h>
#include <pcl/outofcore/outofcore_impl.h>
#include <pcl/features/normal_3d.h>

#include "nlohmann/json.hpp"

//#include "ReCapHDRI.h"

namespace e57
{
	Converter::Converter(const boost::filesystem::path& octPath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution, const std::string& coordSys) : octPath(octPath)
	{
		try
		{
			oct = OCT::Ptr(new OCT(min, max, resolution, octPath / boost::filesystem::path("octRoot.oct_idx"), coordSys));
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
			nlohmann::json scanJson;

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
					scanJson[scan.ID]["coodSys"] = CoodSysStr(scan.coodSys);
					std::stringstream ss;
					ss << scan.transform;
					scanJson[scan.ID]["transform"] = ss.str();
					scanJson[scan.ID]["hasPointXYZ"] = scan.hasPointXYZ;
					scanJson[scan.ID]["hasPointRGB"] = scan.hasPointRGB;
					scanJson[scan.ID]["hasPointI"] = scan.hasPointI;
					scanJson[scan.ID]["numPoints"] = scan.numPoints;
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
					scanJson[scan.ID]["numValidPoints"] = scanCloud_removeBlack->size();
				}
			}

			// Save info
			{
				std::ofstream scanFile((octPath / boost::filesystem::path("scan.txt")).string(), std::ios_base::out);
				if (!scanFile)
					throw pcl::PCLException("Create scanFile " + (e57Path / boost::filesystem::path("scan.txt")).string() + " failed.");
				scanFile << scanJson;
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

	void Converter::SaveScanImages(const boost::filesystem::path& scanImagePath)
	{
		if (!E57_CAN_CONTAIN_SCANID)
			throw pcl::PCLException("You must compile the program with POINT_E57_WITH_SCANID definition to enable the function");

		// not implement
		/*
		e57::ImageFile imf(filePath.string().c_str(), "r");
		e57::VectorNode data3D(imf.root().get("data3D"));
		e57::VectorNode images2D(imf.root().get("images2D"));

		for (int64_t scanID = 0; scanID < data3D.childCount(); ++scanID)
		{
			E57Scan scanData;
			{
				ReadScan(imf, data3D, scanID, scanData);
				{
					std::stringstream ss;
					ss << "[e57::%s::Converter] ReadScan - scann" << scanID << ".\n";
					PCL_INFO(ss.str().c_str(), "Converter");
				}

				if (scanImagePath.size() > 0)
				{
					if (!boost::filesystem::exists(scanImagePath))
					{
						boost::filesystem::create_directory(scanImagePath);
						std::stringstream ss;
						ss << "[e57::%s::Converter] OutOfCoreOctree create directory - " << scanImagePath << ".\n";
						PCL_INFO(ss.str().c_str(), "Converter");
					}

				}
			}
		}*/
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

	void Converter::ExportToPCD(const double voxelUnit, const unsigned int searchRadiusNumVoxels, pcl::PointCloud<PointPCD>& out)
	{
		try
		{
			double searchRadius = voxelUnit * searchRadiusNumVoxels;
			pcl::VoxelGrid<PointE57> vf;
			pcl::RadiusOutlierRemoval<PointE57> ror;
			pcl::NormalEstimation<PointE57, PointPCD> ne;

			pcl::StatisticalOutlierRemoval<PointPCD> sor;
			pcl::CropBox<PointPCD> cb;

			//
			{
				vf.setLeafSize(voxelUnit, voxelUnit, voxelUnit);

				//
				ror.setRadiusSearch(searchRadius);
				ror.setMinNeighborsInRadius((searchRadius * searchRadius)*0.1);

				//
				pcl::search::KdTree<PointE57>::Ptr tree(new pcl::search::KdTree<PointE57>());
				ne.setSearchMethod(tree);
				ne.setRadiusSearch(searchRadius);

				//
				sor.setMeanK(int(searchRadius * searchRadius));
				sor.setStddevMulThresh(1.0);
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

					// Query Points
					Eigen::Vector3d minBB;
					Eigen::Vector3d maxBB;
					Eigen::Vector3d extMinBB;
					Eigen::Vector3d extMaxBB;
					Eigen::Vector3d extXYZ (searchRadius, searchRadius, searchRadius);
					(*it)->getBoundingBox(minBB, maxBB);
					std::size_t depth = (*it)->getDepth();
					extMinBB = minBB - extXYZ;
					extMaxBB = maxBB + extXYZ;
					cb.setMin(Eigen::Vector4f(minBB.x(), minBB.y(), minBB.z(), 1.0));
					cb.setMax(Eigen::Vector4f(maxBB.x(), maxBB.y(), maxBB.z(), 1.0));

					pcl::PointCloud<PointPCD>::Ptr pcdCloud(new pcl::PointCloud<PointPCD>);
					{
						pcl::PointCloud<PointPCD>::Ptr pcdCloud_VF_ROR(new pcl::PointCloud<PointPCD>);
						{
							{
								std::stringstream ss;
								ss << "[e57::%s::ExportToPCD] Query - minBB, maxBB, depth, extMinBB, extMaxBB: " << minBB << ", " << maxBB << ", " << depth << ", " << extMinBB << ", " << extMaxBB << ".\n";
								PCL_INFO(ss.str().c_str(), "Converter");
							}
							pcl::PointCloud<PointE57>::Ptr e57Cloud(new pcl::PointCloud<PointE57>);
							{
								pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
								oct->queryBoundingBox(extMinBB, extMaxBB, depth, blob);
								pcl::fromPCLPointCloud2(*blob, *e57Cloud);
							}

							pcl::PointCloud<PointE57>::Ptr e57Cloud_VF(new pcl::PointCloud<PointE57>);
							pcl::PointCloud<PointE57>::Ptr e57Cloud_VF_ROR(new pcl::PointCloud<PointE57>);
							{
								// DownSampling
								PCL_INFO("[e57::%s::ExportToPCD] DownSampling.\n", "Converter");
								vf.setInputCloud(e57Cloud);
								vf.filter(*e57Cloud_VF);
								{
									std::stringstream ss;
									ss << "[e57::%s::ExportToPCD] DownSampling - inSize, outSize: " << e57Cloud->size() << ", " << e57Cloud_VF->size() << ".\n";
									PCL_INFO(ss.str().c_str(), "Converter");
								}

								//RadiusOutlierRemoval
								PCL_INFO("[e57::%s::ExportToPCD] RadiusOutlierRemoval.\n", "Converter");
								vf.setInputCloud(e57Cloud_VF);
								vf.filter(*e57Cloud_VF_ROR);
								{
									std::stringstream ss;
									ss << "[e57::%s::ExportToPCD] DownSampling - inSize, outSize: " << e57Cloud_VF->size() << ", " << e57Cloud_VF_ROR->size() << ".\n";
									PCL_INFO(ss.str().c_str(), "Converter");
								}
							}

							// Estimat Normal
							pcdCloud_VF_ROR->resize(e57Cloud_VF_ROR->size());
							for (std::size_t pi = 0; pi < e57Cloud_VF_ROR->size(); ++pi)
								(*pcdCloud_VF_ROR)[pi].FromPointE57((*e57Cloud_VF_ROR)[pi]);

							if (PCD_CAN_CONTAIN_NORMAL)
							{
								PCL_INFO("[e57::%s::ExportToPCD] NormalEstimation.\n", "Converter");
								ne.setInputCloud(e57Cloud_VF_ROR);
								ne.setSearchSurface(e57Cloud_VF);
								ne.compute(*pcdCloud_VF_ROR);
							}
						}

						// Outlier Removal
						pcl::PointCloud<PointPCD>::Ptr pcdCloud_VF_ROR_SOR(new pcl::PointCloud<PointPCD>);
						{
							PCL_INFO("[e57::%s::ExportToPCD] Outlier Removal.\n", "Converter");
							sor.setInputCloud(pcdCloud_VF_ROR);
							sor.filter(*pcdCloud_VF_ROR_SOR);
							{
								std::stringstream ss;
								ss << "[e57::%s::ExportToPCD] Outlier Removal - inSize, outSize: " << pcdCloud_VF_ROR->size() << ", " << pcdCloud_VF_ROR_SOR->size() << ".\n";
								PCL_INFO(ss.str().c_str(), "Converter");
							}
						}

						// Crop Box
						{
							PCL_INFO("[e57::%s::ExportToPCD] Crop Box.\n", "Converter");
							cb.setInputCloud(pcdCloud_VF_ROR_SOR);
							cb.filter(*pcdCloud);
							{
								std::stringstream ss;
								ss << "[e57::%s::ExportToPCD] Crop Box - inSize, outSize: " << pcdCloud_VF_ROR_SOR->size() << ", " << pcdCloud->size() << ".\n";
								PCL_INFO(ss.str().c_str(), "Converter");
							}
						}
					}

					// Merge
					{
						out += (*pcdCloud);
						{
							std::stringstream ss;
							ss << "[e57::%s::ExportToPCD] Process leaf " << leafID << " end, final cloud total " << out.size() << " points.\n";
							PCL_INFO(ss.str().c_str(), "Converter");
						}
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
