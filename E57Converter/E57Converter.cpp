#pragma once

#include <fstream>
#include <limits>
#include <algorithm> 

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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>

#include "E57Utils.h"
#include "E57Converter.h"
#include "E57AlbedoEstimation.h"
#include "E57BLK360HDRI.h"

namespace e57
{
	void Converter::LoadScanInfo(const boost::filesystem::path& octPath)
	{
		nlohmann::json scanInfoJson;
		std::ifstream file((octPath / boost::filesystem::path("scanInfo.txt")).string(), std::ios_base::in);
		if (!file)
			throw pcl::PCLException("Load file " + (octPath / boost::filesystem::path("scanInfo.txt")).string() + " failed.");
		file >> scanInfoJson;
		file.close();

		for (nlohmann::json::const_iterator it = scanInfoJson.begin(); it != scanInfoJson.end(); ++it)
			scanInfo.push_back(ScanInfo::LoadFromJson(*it));
	}

	void Converter::DumpScanInfo(const boost::filesystem::path& octPath)
	{
		std::ofstream file((octPath / boost::filesystem::path("scanInfo.txt")).string(), std::ios_base::out);
		if (!file)
			throw pcl::PCLException("Create file " + (octPath / boost::filesystem::path("scanInfo.txt")).string() + " failed.");

		nlohmann::json scanInfoJson;
		for (std::size_t i = 0; i < scanInfo.size(); ++i)
			scanInfoJson[i] = scanInfo[i].DumpToJson();

		file << scanInfoJson;
		file.close();
	}

	Converter::Converter(const boost::filesystem::path& octPath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution, const std::string& coordSys) : octPath(octPath)
	{
		try
		{
			oct = OCT::Ptr(new OCT(min, max, resolution, octPath / boost::filesystem::path("octRoot.oct_idx"), coordSys));

			// Init scanInfo file
			DumpScanInfo(octPath);
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
			LoadScanInfo(octPath);
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

	void Converter::LoadE57(const boost::filesystem::path& e57Path, const double LODSamplePercent, const uint8_t minRGB, const Scanner& scanner)
	{
		try
		{
			e57::ImageFile imf(e57Path.string().c_str(), "r");
			e57::VectorNode data3D(imf.root().get("data3D"));
			e57::VectorNode images2D(imf.root().get("images2D"));

			scanInfo.clear();
			for (int64_t scanID = 0; scanID < data3D.childCount(); ++scanID)
			{
				Scan scan (scanner);

				// Load E57
				{
					{
						std::stringstream ss;
						ss << "[e57::%s::Converter] Load - scann" << scanID << ".\n";
						PCL_INFO(ss.str().c_str(), "Converter");
					}
					scan.Load(imf, data3D, scanID);
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
					scan.ToPointCloud(*scanCloud, minRGB);
					oct->addPointCloud(scanCloud);
				}

				scanInfo.push_back(scan);
			}

			// Save scanInfo
			DumpScanInfo(octPath);

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

			for (std::vector<ScanInfo>::const_iterator it = scanInfo.begin(); it != scanInfo.end(); ++it)
			{
				{
					std::stringstream ss;
					ss << "[e57::%s::ReconstructScanImages] Reconstruct scan" << it->ID << ".\n";
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
				Eigen::Matrix4d wordToScan = it->transform;

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
						Eigen::Vector3d rae = XYZToRAE(raeMode, Eigen::Vector3d(scanPos.x(), scanPos.y(), scanPos.z()));
						depth = rae.x();
						uv = RAEToUV(raeMode, rae);
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
					fileName << "scan" << it->ID << "_Depth.png";
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
					fileName << "scan" << it->ID << "_Normal.png";
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
					fileName << "scan" << it->ID << "_Curvature.png";
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
					fileName << "scan" << it->ID << "_RGB.png";
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
					fileName << "scan" << it->ID << "_Intensity.png";
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
					fileName << "scan" << it->ID << "_Label.png";
					std::string filePath = (scanImagePath / boost::filesystem::path(fileName.str())).string();

					pcl::PCLImage image;
					pcl::io::PointCloudImageExtractorFromLabelField<PointPCD> pcie;
					pcie.setPaintNaNsWithBlack(true);
					if (!pcie.extract(*scanImage, image))
						throw pcl::PCLException("Failed to extract an image from Label field .");
					pcl::io::savePNGFile(filePath, image);
				}
#endif
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

	void Converter::LoadScanHDRI(const boost::filesystem::path& filePath)
	{
		try
		{
			if (!E57_CAN_CONTAIN_HDR)
				throw pcl::PCLException("You must compile the program with POINT_E57_WITH_HDR definition to enable the function");

			if (scanInfo.size () == 0)
				throw pcl::PCLException("scanInfo.size () == 0");

			Scanner scanner = scanInfo.begin()->scanner;
			for (std::vector<ScanInfo>::const_iterator it = scanInfo.begin() + 1; it != scanInfo.end(); ++it)
			{
				if ( scanner != it->scanner )
					throw pcl::PCLException("Hybrid scanner is not support.");
			}

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

					BLK360HDRI hdri(scanDataDirs[i].dir);
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

	void Converter::ExportToPCD(const double voxelUnit, const unsigned int searchRadiusNumVoxels, const int meanK, const int polynomialOrder, bool reconstructAlbedo, pcl::PointCloud<PointPCD>& out)
	{
		if (reconstructAlbedo)
		{
			if (!E57_CAN_CONTAIN_LABEL)
				throw pcl::PCLException("You must compile the program with POINT_E57_WITH_LABEL definition to enable reconstructAlbedo");
			if (!E57_CAN_CONTAIN_INTENSITY)
				throw pcl::PCLException("You must compile the program with POINT_E57_WITH_INTENSITY definition to enable reconstructAlbedo");
			if (!PCD_CAN_CONTAIN_INTENSITY)
				throw pcl::PCLException("You must compile the program with POINT_PCD_WITH_INTENSITY definition to enable reconstructAlbedo");
		}

		try
		{
			double searchRadius = voxelUnit * searchRadiusNumVoxels;
			pcl::VoxelGrid<PointE57> vf;
			pcl::StatisticalOutlierRemoval<PointE57> olr;
			pcl::MovingLeastSquares<PointE57, PointPCD> mls;
			pcl::CropBox<PointE57> cbE57;
			pcl::CropBox<PointPCD> cbPCD;
			pcl::NormalEstimationOMP<PointE57, PointPCD> ne;
			AlbedoEstimationOMP ae;

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
				if (reconstructAlbedo)
				{
					ae.setSearchMethod(tree);
					ae.setRadiusSearch(searchRadius);
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

					pcl::PointCloud<PointE57>::Ptr e57Cloud_RAW(new pcl::PointCloud<PointE57>);
					pcl::PointCloud<PointE57>::Ptr e57Cloud(new pcl::PointCloud<PointE57>);
					pcl::PointCloud<PointPCD>::Ptr pcdCloud(new pcl::PointCloud<PointPCD>);
					pcl::PointCloud<PointE57>::Ptr e57Cloud_CB(new pcl::PointCloud<PointE57>);
					pcl::PointCloud<PointPCD>::Ptr pcdCloud_CB(new pcl::PointCloud<PointPCD>);

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
						cbPCD.setMin(Eigen::Vector4f(minBB.x(), minBB.y(), minBB.z(), 1.0));
						cbE57.setMin(Eigen::Vector4f(minBB.x(), minBB.y(), minBB.z(), 1.0));
						cbPCD.setMax(Eigen::Vector4f(maxBB.x(), maxBB.y(), maxBB.z(), 1.0));
						cbE57.setMax(Eigen::Vector4f(maxBB.x(), maxBB.y(), maxBB.z(), 1.0));

						std::stringstream ss;
						ss << "[e57::%s::ExportToPCD] Query - minBB, maxBB, depth, extMinBB, extMaxBB: " << minBB << ", " << maxBB << ", " << depth << ", " << extMinBB << ", " << extMaxBB << ".\n";
						PCL_INFO(ss.str().c_str(), "Converter");

						pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
						oct->queryBoundingBox(extMinBB, extMaxBB, depth, blob);
						pcl::fromPCLPointCloud2(*blob, *e57Cloud_RAW);
					}

					// Down Sampling
					{
						PCL_INFO("[e57::%s::ExportToPCD] Down Sampling.\n", "Converter");
						vf.setInputCloud(e57Cloud_RAW);
						vf.filter(*e57Cloud);

						std::stringstream ss;
						ss << "[e57::%s::ExportToPCD] Down Sampling - inSize, outSize: " << e57Cloud_RAW->size() << ", " << e57Cloud->size() << ".\n";
						PCL_INFO(ss.str().c_str(), "Converter");
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

					// Crop Box
					{
						{
							PCL_INFO("[e57::%s::ExportToPCD] PCD Crop Box.\n", "Converter");
							cbPCD.setInputCloud(pcdCloud);
							cbPCD.filter(*pcdCloud_CB);
							std::stringstream ss;
							ss << "[e57::%s::ExportToPCD] PCD Crop Box - inSize, outSize: " << pcdCloud->size() << ", " << pcdCloud_CB->size() << ".\n";
							PCL_INFO(ss.str().c_str(), "Converter");
						}
						{
							PCL_INFO("[e57::%s::ExportToPCD] E57 Crop Box.\n", "Converter");
							cbE57.setInputCloud(e57Cloud);
							cbE57.filter(*e57Cloud_CB);
							std::stringstream ss;
							ss << "[e57::%s::ExportToPCD] E57 Crop Box - inSize, outSize: " << e57Cloud->size() << ", " << e57Cloud_CB->size() << ".\n";
							PCL_INFO(ss.str().c_str(), "Converter");
						}
					}

					// Estimat Normal if not Estimat Surface
					if (polynomialOrder <= 0)
					{
						PCL_INFO("[e57::%s::ExportToPCD] Estimat Normal.\n", "Converter");
						ne.setSearchSurface(e57Cloud);
						ne.setInputCloud(e57Cloud_CB);
						ne.compute(*pcdCloud_CB);
					}

					// Estimate albedo
					if (reconstructAlbedo)
					{
						PCL_INFO("[e57::%s::ExportToPCD] Estimat Albedo.\n", "Converter");
						ae.setSearchSurface(e57Cloud);
						ae.setInputCloud(e57Cloud_RAW);
						ae.compute(*pcdCloud_CB);
					}

					// Merge
					out += (*pcdCloud_CB);
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
