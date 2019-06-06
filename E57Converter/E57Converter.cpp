#pragma once

#include <fstream>
#include <vector>
#include <limits>
#include <algorithm> 

#include "E57Utils.h"
#include "E57Converter.h"
#include "BLK360HDRI.h"

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
#include <pcl/conversions.h>
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
			if (!E57_CAN_CONTAIN_LABEL)
				throw pcl::PCLException("You must compile the program with POINT_E57_WITH_LABEL definition to enable the function");

			if (!boost::filesystem::exists(scanImagePath))
			{
				boost::filesystem::create_directory(scanImagePath);
				std::stringstream ss;
				ss << "[e57::%s::ReconstructScanImages] OutOfCoreOctree create directory - " << scanImagePath << ".\n";
				PCL_INFO(ss.str().c_str(), "Converter");
			}

			std::size_t scanID = 0;
			for (nlohmann::json::iterator it = scanInfo.begin(); it != scanInfo.end(); ++it)
			{
				{
					std::stringstream ss;
					ss << "[e57::%s::ReconstructScanImages] Reconstruct scan" << scanID << ".\n";
					PCL_INFO(ss.str().c_str(), "Converter");
				}


				//
				pcl::PointCloud<PointPCD>::Ptr scanImage(new pcl::PointCloud<PointPCD>);
				scanImage->width = width;
				scanImage->height = height;
				scanImage->is_dense = true;
				scanImage->resize(width*height);
				for (pcl::PointCloud<PointPCD>::iterator cloudIT = scanImage->begin(); cloudIT != scanImage->end(); ++cloudIT)
				{
					cloudIT->x = NAN;
					cloudIT->y = NAN;
					cloudIT->z = NAN;
					cloudIT->data[3] = std::numeric_limits<float>::infinity(); // use as depth buffer
#ifdef POINT_PCD_WITH_NORMAL
					cloudIT->normal_x = NAN;
					cloudIT->normal_y = NAN;
					cloudIT->normal_z = NAN;
					cloudIT->curvature = NAN;
#endif
#ifdef POINT_PCD_WITH_RGB
					cloudIT->r = 0;
					cloudIT->g = 0;
					cloudIT->b = 0;
#endif
				}

				//
				Eigen::Matrix4d scanToWord;
				nlohmann::json::iterator it2 = (*it)["transform"].begin();
				for (int r = 0; r < 4; ++r)
				{
					for (int c = 0; c < 4; ++c)
					{
						scanToWord(r, c) = *it2;
						++it2;
					}
				}
				Eigen::Matrix4d wordToScan = scanToWord.inverse();

				for (pcl::PointCloud<PointPCD>::iterator cloudIT = cloud.begin(); cloudIT != cloud.end(); ++cloudIT)
				{
					Eigen::Vector4d scanPos(cloudIT->x, cloudIT->y, cloudIT->z, 1.0);
					scanPos = wordToScan * scanPos;

					Eigen::Vector2d uv;
					double depth = 0;
					switch (coodSys)
					{
					case CoodSys::RAE:
					{
						Eigen::Vector3d rae = e57::XYZToRAE(raeMode, Eigen::Vector3d(scanPos.x(), scanPos.y(), scanPos.z()));
						depth = rae.x();
						uv = e57::RAEToUV(raeMode, rae);
					}
					break;

					default:
						throw pcl::PCLException("coodSys is not support.");
						break;
					}

					std::size_t col = uv.x() * (width - 1);
					std::size_t row = (1.0-uv.y()) * (height - 1);
					std::size_t index = row * width + col;
					PointPCD& scanImageP = (*scanImage)[index];
					if (depth < scanImageP.data[3])
					{
						scanImageP = *cloudIT;
						scanImageP.x = uv.x();
						scanImageP.y = uv.y();
						scanImageP.z = depth;
						scanImageP.data[3] = depth;
					}
				}

				// Z
				{
					std::stringstream fileName;
					fileName << "scan" << scanID << "_Depth.png";
					std::string filePath = (scanImagePath / boost::filesystem::path(fileName.str())).string();

					pcl::PCLImage image;
					pcl::io::PointCloudImageExtractorFromZField<PointPCD> pcie;
					pcie.setPaintNaNsWithBlack(true);
					pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
					if (!pcie.extract(*scanImage, image))
						throw pcl::PCLException("Failed to extract an image from Depth field .");
					pcl::io::savePNGFile(filePath, image);
				}

#ifdef POINT_PCD_WITH_NORMAL
				// Normal
				{
					std::stringstream fileName;
					fileName << "scan" << scanID << "_Normal.png";
					std::string filePath = (scanImagePath / boost::filesystem::path(fileName.str())).string();

					pcl::PCLImage image;
					pcl::io::PointCloudImageExtractorFromNormalField<PointPCD> pcie;
					pcie.setPaintNaNsWithBlack(true);
					if (!pcie.extract(*scanImage, image))
						throw pcl::PCLException("Failed to extract an image from Normal field .");
					pcl::io::savePNGFile(filePath, image);
				}

				// Curvature
				{
					std::stringstream fileName;
					fileName << "scan" << scanID << "_Curvature.png";
					std::string filePath = (scanImagePath / boost::filesystem::path(fileName.str())).string();

					pcl::PCLImage image;
					pcl::io::PointCloudImageExtractorFromCurvatureField<PointPCD> pcie;
					pcie.setPaintNaNsWithBlack(true);
					pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
					if (!pcie.extract(*scanImage, image))
						throw pcl::PCLException("Failed to extract an image from Curvature field .");
					pcl::io::savePNGFile(filePath, image);
				}
#endif
#ifdef POINT_PCD_WITH_RGB
				{
					std::stringstream fileName;
					fileName << "scan" << scanID << "_RGB.png";
					std::string filePath = (scanImagePath / boost::filesystem::path(fileName.str())).string();

					pcl::PCLImage image;
					pcl::io::PointCloudImageExtractorFromRGBField<PointPCD> pcie;
					pcie.setPaintNaNsWithBlack(true);
					if (!pcie.extract(*scanImage, image))
						throw pcl::PCLException("Failed to extract an image from RGB field .");
					pcl::io::savePNGFile(filePath, image);
				}
#endif
#ifdef POINT_PCD_WITH_INTENSITY
				{
					std::stringstream fileName;
					fileName << "scan" << scanID << "_Intensity.png";
					std::string filePath = (scanImagePath / boost::filesystem::path(fileName.str())).string();

					pcl::PCLImage image;
					pcl::io::PointCloudImageExtractorFromIntensityField<PointPCD> pcie;
					pcie.setPaintNaNsWithBlack(true);
					pcie.setScalingMethod(pcie.SCALING_FULL_RANGE);
					if (!pcie.extract(*scanImage, image))
						throw pcl::PCLException("Failed to extract an image from Intensity field .");
					pcl::io::savePNGFile(filePath, image);
				}
#endif
#ifdef POINT_PCD_WITH_LABEL
				{
					std::stringstream fileName;
					fileName << "scan" << scanID << "_Label.png";
					std::string filePath = (scanImagePath / boost::filesystem::path(fileName.str())).string();

					pcl::PCLImage image;
					pcl::io::PointCloudImageExtractorFromLabelField<PointPCD> pcie;
					pcie.setPaintNaNsWithBlack(true);
					if (!pcie.extract(*scanImage, image))
						throw pcl::PCLException("Failed to extract an image from Label field .");
					pcl::io::savePNGFile(filePath, image);
				}
#endif
				//
				scanID++;
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

	struct ScanDataDir
	{
		int number;
		boost::filesystem::path dir;

		ScanDataDir(int number = -1, const boost::filesystem::path& dir = boost::filesystem::path(""))
			: number(number), dir(dir)
		{}
	};

	bool ScanDataDirCompare(const ScanDataDir& i, const ScanDataDir& j) { return (i.number < j.number); }

	void Converter::LoadScanHDRI(const boost::filesystem::path& filePath, const Scanner& scanner)
	{
		try
		{
			if (!E57_CAN_CONTAIN_HDR)
				throw pcl::PCLException("You must compile the program with POINT_E57_WITH_HDR definition to enable the function");

			switch (scanner)
			{
			case Scanner::BLK360:
			{
				if (!(ToUpper(filePath.extension().string()) == ".RCP"))
					throw pcl::PCLException("Scanner BLK360 need a .RCP file.");

				boost::filesystem::path fileName = filePath.stem();
				boost::filesystem::path fileDir = filePath.parent_path();
				boost::filesystem::path dataDir = fileDir / boost::filesystem::path(fileName.string() + " Support") / boost::filesystem::path("sourcedata");

				if (!boost::filesystem::is_directory(dataDir))
					throw pcl::PCLException("Scanner BLK360 didnot save scanned sourcedata.");

				std::vector<ScanDataDir> scanDataDirs;
				for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(dataDir), {}))
				{
					if (boost::filesystem::is_directory(entry.path()))
					{
						std::string baseName = entry.path().stem().string();
						if (baseName.find(fileName.string()) == 0)
						{
							int id = IsUnsignedInt(baseName.substr(fileName.string().size()));
							if (id > 0)
								scanDataDirs.push_back(ScanDataDir(id, entry.path()));
						}
					}
				}

				//
				if(scanDataDirs.size() == 0)
					throw pcl::PCLException("Do not find any scanned data.");

				std::sort(scanDataDirs.begin(), scanDataDirs.end(), ScanDataDirCompare);
				for (int i = 0; i < scanDataDirs.size(); i++)
				{
					std::stringstream ss;
					ss << "[e57::%s::LoadScanHDRI] Find scanData for scan: " << i << ", which is from " << scanDataDirs[i].number << "-th scanning process.\n";
					PCL_INFO(ss.str().c_str(), "Converter");

					BLK360::HDRI hdri(scanDataDirs[i].dir);
				}
			}
			break;

			default:
				throw pcl::PCLException("Read scanned HDRI from the Scanner type is not support.");
				break;
			}
		}
		catch (std::exception& ex)
		{
			std::stringstream ss;
			ss << "[e57::%s::LoadScanHDRI] Got an std::exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "Converter");
		}
		catch (...)
		{
			PCL_INFO("[e57::%s::LoadScanHDRI] Got an unknown exception.\n", "Converter");
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
