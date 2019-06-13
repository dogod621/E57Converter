#pragma once

#include <future>
#include <fstream>
#include <limits>
#include <algorithm> 

#include <pcl/common/common.h>
#include <pcl/common/io.h>
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
//#include <pcl/segmentation/supervoxel_clustering.h>
#include "SupervoxelClustering.h"

#include "E57Utils.h"
#include "E57Converter.h"
//#include "E57AlbedoEstimation.h"
#include "E57BLK360HDRI.h"

namespace e57
{
	bool ScanInfoDirCompare(const ScanInfo& i, const ScanInfo& j) { return (i.ID < j.ID); }

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
		std::sort(scanInfo.begin(), scanInfo.end(), ScanInfoDirCompare);
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

	// for asyc
	int LoadE57_LoadScan(const e57::ImageFile* imf, const e57::VectorNode* data3D, int64_t scanID, const Scanner& scanner, std::vector<std::shared_ptr<Scan>>* scanBuffer, bool p)
	{
		if (scanID >= data3D->childCount())
			return 0;
		
		std::stringstream ss;
		ss << "[e57::LoadE57_LoadScan] Start - scann" << scanID << ".\n";
		PCL_INFO(ss.str().c_str());
		(*scanBuffer)[p] = std::shared_ptr<Scan>(new Scan(scanner));
		(*scanBuffer)[p]->Load(*imf, *data3D, scanID);
		PCL_INFO("[e57::LoadE57_LoadScan] End.\n");
		return 0;
	}

	int LoadE57_MergeScan(const Converter::OCT::Ptr* oct, const uint8_t minRGB, std::vector<std::shared_ptr<Scan>>* scanBuffer, bool p)
	{
		if (!(*scanBuffer)[p])
			return 1;

		PCL_INFO("[e57::LoadE57_MergeScan] Start.\n");

		pcl::PointCloud<PointE57>::Ptr scanCloud = pcl::PointCloud<PointE57>::Ptr(new pcl::PointCloud<PointE57>);
		(*scanBuffer)[p]->ExtractValidPointCloud(*scanCloud, minRGB);
		(*oct)->addPointCloud(scanCloud);

		//
		PCL_INFO("[e57::LoadE57_MergeScan] End.\n");
		return 0;
	}

	void Converter::LoadE57(const boost::filesystem::path& e57Path, const double LODSamplePercent, const uint8_t minRGB, const Scanner& scanner)
	{
		try
		{
			e57::ImageFile imf(e57Path.string().c_str(), "r");
			e57::VectorNode data3D(imf.root().get("data3D"));
			e57::VectorNode images2D(imf.root().get("images2D"));

			scanInfo.clear();

			bool p = false;
			std::vector<std::shared_ptr<Scan>> scanBuffer (2);
			{
				int rLoadScan = LoadE57_LoadScan(&imf, &data3D, 0, scanner, &scanBuffer, p);
				if (rLoadScan != 0) throw pcl::PCLException("LoadE57_LoadScan failed - " + std::to_string(rLoadScan));
			}
			for (int64_t scanID = 0; scanID < data3D.childCount(); ++scanID)
			{
				std::future<int> loadScan = std::async(LoadE57_LoadScan, &imf, &data3D, scanID + 1, scanner, &scanBuffer, !p);
				std::future<int> mergeScan = std::async(LoadE57_MergeScan, &oct, minRGB, &scanBuffer, p);
				int rLoadScan = loadScan.get();
				int rMergeScan = mergeScan.get();
				if (rLoadScan != 0) throw pcl::PCLException("LoadE57_LoadScan failed - " + std::to_string(rLoadScan));
				if (rMergeScan != 0) throw pcl::PCLException("LoadE57_MergeScan failed - " + std::to_string(rMergeScan));

				scanInfo.push_back(*scanBuffer[p]);
				p = !p;
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

	struct Color
	{
		unsigned char r;
		unsigned char g;
		unsigned char b;
	};

	void Converter::ReconstructScanImages(pcl::PointCloud<PointPCD>& cloud, const boost::filesystem::path& scanImagePath, const CoodSys coodSys, const RAEMode raeMode, const float fovy, const unsigned int width, const unsigned int height)
	{
		try
		{
			if (!E57_CAN_CONTAIN_LABEL)
				throw pcl::PCLException("You must compile the program with POINT_E57_WITH_LABEL definition to enable the function");

			//
			std::vector<Color> colorTable;
			colorTable.resize(10000);
			srand(time(NULL));
			for (std::size_t i = 0; i < colorTable.size(); ++i)
			{
				colorTable[i].r = rand() % 255;
				colorTable[i].g = rand() % 255;
				colorTable[i].b = rand() % 255;
			}

			//
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
				Eigen::Matrix4d wordToScan = it->transform.inverse();

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

						std::size_t colorIndex = cloudIT->label % colorTable.size();
						scanImageP.r = colorTable[colorIndex].r;
						scanImageP.g = colorTable[colorIndex].g;
						scanImageP.b = colorTable[colorIndex].b;
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
					pcie.setScalingMethod(pcie.SCALING_FIXED_FACTOR);
					pcie.setScalingFactor(300.f);
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

	//
	struct OCTQuery
	{
		double voxelUnit;
		int meanK;
		int polynomialOrder;
		bool reconstructAlbedo;
		bool reconstructNDF;
		Eigen::Vector3d minBB;
		Eigen::Vector3d maxBB;
		std::size_t depth;
		double searchRadius;
	};

	int ExportToPCD_Query(const Converter::OCT::Ptr* oct, const std::vector<OCTQuery>* querys, const int64_t queryID, std::vector<pcl::PointCloud<PointE57>::Ptr>* rawE57CloudBuffer, bool p)
	{
		if (queryID >= querys->size())
			return 0;

		std::stringstream ss;
		ss << "[e57::ExportToPCD_Query] Start - query" << queryID << "/" << querys->size() << ".\n";
		PCL_INFO(ss.str().c_str());

		//
		Eigen::Vector3d extMinBB;
		Eigen::Vector3d extMaxBB;
		Eigen::Vector3d extXYZ((*querys)[queryID].searchRadius, (*querys)[queryID].searchRadius, (*querys)[queryID].searchRadius);
		extMinBB = (*querys)[queryID].minBB - extXYZ;
		extMaxBB = (*querys)[queryID].maxBB + extXYZ;

		pcl::PCLPointCloud2::Ptr blob(new pcl::PCLPointCloud2);
		(*oct)->queryBoundingBox(extMinBB, extMaxBB, (*querys)[queryID].depth, blob);
		(*rawE57CloudBuffer)[p] = pcl::PointCloud<PointE57>::Ptr(new pcl::PointCloud<PointE57>());
		pcl::fromPCLPointCloud2(*blob, *(*rawE57CloudBuffer)[p]);
		PCL_INFO("[e57::ExportToPCD_Query] End. \n");
		return 0;
	}
	
	int ExportToPCD_Process(const std::vector<OCTQuery>* querys, const int64_t queryID, std::vector<pcl::PointCloud<PointE57>::Ptr>* rawE57CloudBuffer, bool p, const std::vector<ScanInfo>* scanInfos, pcl::PointCloud<PointPCD>::Ptr* outPointCloud)
	{
		if (queryID >= querys->size())
			return 0;
		if ((*rawE57CloudBuffer)[p].get() == nullptr)
			return 1;
		if (outPointCloud->get() == nullptr)
			return 2;
		(*outPointCloud)->clear();

		std::stringstream ss;
		ss << "[e57::ExportToPCD_Process] Start - query" << queryID << "/" << querys->size() << ".\n";
		PCL_INFO(ss.str().c_str());

		//
		pcl::PointCloud<PointExchange>::Ptr rawE57Cloud(new pcl::PointCloud<PointExchange>());
		rawE57Cloud->resize((*rawE57CloudBuffer)[p]->size());
		for (std::size_t pi = 0; pi < rawE57Cloud->size(); ++pi)
			(*rawE57Cloud)[pi] = (*(*rawE57CloudBuffer)[p])[pi];
		pcl::search::KdTree<PointExchange>::Ptr rawE57Cloud_tree(new pcl::search::KdTree<PointExchange>());

		pcl::PointCloud<PointExchange>::Ptr e57Cloud(new pcl::PointCloud<PointExchange>);
		pcl::search::KdTree<PointExchange>::Ptr e57Cloud_tree(new pcl::search::KdTree<PointExchange>());

		// DownSampling
		{
			PCL_INFO("[e57::ExportToPCD_Process] DownSampling.\n");
			pcl::VoxelGrid<PointExchange> vf;
			vf.setLeafSize((*querys)[queryID].voxelUnit, (*querys)[queryID].voxelUnit, (*querys)[queryID].voxelUnit);
			vf.setInputCloud(rawE57Cloud);
			vf.filter(*e57Cloud);
			std::stringstream ss;
			ss << "[e57::ExportToPCD_Process] DownSampling - inSize, outSize: " << (*rawE57CloudBuffer)[p]->size() << ", " << e57Cloud->size() << ".\n";
			PCL_INFO(ss.str().c_str());
		}

		//Outlier Removal
		if ((*querys)[queryID].meanK > 0)
		{
			PCL_INFO("[e57::ExportToPCD_Process] Outlier Removal.\n");

			pcl::PointCloud<PointExchange>::Ptr e57Cloud_OLR(new pcl::PointCloud<PointExchange>);
			pcl::StatisticalOutlierRemoval<PointExchange> olr;
			olr.setMeanK((*querys)[queryID].meanK);
			olr.setStddevMulThresh(1.0);
			olr.setInputCloud(e57Cloud);
			olr.filter(*e57Cloud_OLR);

			std::stringstream ss;
			ss << "[e57::ExportToPCD_Process] Outlier Removal - inSize, outSize: " << e57Cloud->size() << ", " << e57Cloud_OLR->size() << ".\n";
			PCL_INFO(ss.str().c_str());
			e57Cloud = e57Cloud_OLR;
		}

		
		// Estimat Surface
		if ((*querys)[queryID].polynomialOrder > 0)
		{
			PCL_INFO("[e57::ExportToPCD_Process] Estimat Surface.\n");

			//  Copy to pcdCloud
			pcl::MovingLeastSquares<PointExchange, PointExchange> mls;
			mls.setComputeNormals(PCD_CAN_CONTAIN_NORMAL);
			mls.setPolynomialOrder((*querys)[queryID].polynomialOrder);
			mls.setSearchMethod(e57Cloud_tree);
			mls.setSearchRadius((*querys)[queryID].searchRadius);
			mls.setInputCloud(e57Cloud);
			mls.process(*e57Cloud);			
		}
		else // Estimat Normal
		{
			if (PCD_CAN_CONTAIN_NORMAL)
			{
				PCL_INFO("[e57::ExportToPCD_Process] Estimat Normal.\n");

				//
				pcl::NormalEstimationOMP<PointExchange, PointExchange> ne;
				ne.setSearchMethod(e57Cloud_tree);
				ne.setRadiusSearch((*querys)[queryID].searchRadius);
				ne.setSearchSurface(e57Cloud);
				ne.setInputCloud(e57Cloud);
				ne.compute(*e57Cloud);
			}
		}

		// Crop Box
		pcl::PointCloud<PointExchange>::Ptr e57Cloud_CB(new pcl::PointCloud<PointExchange>);
		{
			PCL_INFO("[e57::ExportToPCD_Process] Crop Box.\n");

			pcl::CropBox<PointExchange> cb;
			cb.setMin(Eigen::Vector4f((*querys)[queryID].minBB.x(), (*querys)[queryID].minBB.y(), (*querys)[queryID].minBB.z(), 1.0));
			cb.setMax(Eigen::Vector4f((*querys)[queryID].maxBB.x(), (*querys)[queryID].maxBB.y(), (*querys)[queryID].maxBB.z(), 1.0));
			cb.setInputCloud(e57Cloud);
			cb.filter(*e57Cloud_CB);

			std::stringstream ss;
			ss << "[e57::ExportToPCD_Process] Crop Box - inSize, outSize: " << e57Cloud->size() << ", " << e57Cloud_CB->size() << ".\n";
			PCL_INFO(ss.str().c_str());
		}

		// Estimate albedo
		if ((*querys)[queryID].reconstructAlbedo)
		{
			PCL_INFO("[e57::ExportToPCD_Process] Estimat Albedo.\n");

			//
			PCL_INFO("[e57::ExportToPCD_Process] Estimat Albedo - Upsampling Normal.\n");
			{
				if (e57Cloud_tree->getInputCloud() != e57Cloud)
					e57Cloud_tree->setInputCloud(e57Cloud);

#ifdef _OPENMP
#pragma omp parallel for num_threads(omp_get_num_procs())
#endif
				// Iterating over the entire index vector
				for (int px = 0; px < static_cast<int> (rawE57Cloud->size()); ++px)
				{
					PointExchange& point = (*rawE57Cloud)[px];
					std::vector<int> ki;
					std::vector<float> kd;
					if (e57Cloud_tree->nearestKSearch(point, 1, ki, kd) > 0)
					{
						PointExchange& kPoint = (*e57Cloud)[ki[0]];
						point.normal_x = kPoint.normal_x;
						point.normal_y = kPoint.normal_y;
						point.normal_z = kPoint.normal_z;
						point.curvature = kPoint.curvature;
					}
					else
					{
						point.normal_x = 0.0f;
						point.normal_y = 0.0f;
						point.normal_z = 0.0f;
						point.curvature = 0.0f;
					}
				}
			}

			PCL_INFO("[e57::ExportToPCD_Process] Estimat Albedo - Estimat Albedo.\n");
			{
				/*AlbedoEstimationOMP ae(*scanInfos);
				ae.setSearchMethod(rawE57Cloud_tree);
				ae.setRadiusSearch((*querys)[queryID].searchRadius);
				ae.setSearchSurface(rawE57Cloud);
				ae.setInputCloud(e57Cloud_CB);
				ae.compute(*(*outPointCloud));*/

				if (rawE57Cloud_tree->getInputCloud() != rawE57Cloud)
					rawE57Cloud_tree->setInputCloud(rawE57Cloud);

				//
				LinearSolver linearSolver = LinearSolver::EIGEN_SVD;
				double distInterParm = 10.0;
				double angleInterParm = 20.0;
				double frontInterParm = 5.0;
				double cutFalloff = 0.33;
				double cutGrazing = 0.86602540378;
				double radius = (*querys)[queryID].searchRadius;
				Eigen::Vector3d tempVec(1.0, 1.0, 1.0);
				tempVec /= tempVec.norm();

#ifdef _OPENMP
#pragma omp parallel for shared (e57Cloud_CB) num_threads(omp_get_num_procs())
#endif
				for (int px = 0; px < static_cast<int> (e57Cloud_CB->size()); ++px)
				{
					bool success = false;
					std::vector<int> ki;
					std::vector<float> kd;
					PointExchange& point = (*e57Cloud_CB)[px];
					Eigen::Vector3d pointNormal(point.normal_x, point.normal_y, point.normal_z);
					if (std::abs(pointNormal.norm() - 1.0f) > 0.05f)
					{
						PCL_WARN("[e57::ExportToPCD_Process] pointNormal is not valid!!?.\n");
					}
					else
					{
						std::vector<ScannLaserInfo> scannLaserInfos;
						int numk = rawE57Cloud_tree->radiusSearch(point, radius, ki, kd);
						if (numk > 0)
						{
							scannLaserInfos.reserve(numk);
							for (int k = 0; k < numk; ++k)
							{
								PointExchange& kPoint = (*rawE57Cloud)[ki[k]];
								double d = std::sqrt(kd[k]);
								if (d > radius)
								{
									PCL_WARN("[e57::ExportToPCD_Process] distance is larger then radius!!? Ignore.\n");
								}
								else
								{
									ScannLaserInfo scannLaserInfo;
									const ScanInfo& scanScanInfo = (*scanInfos)[kPoint.label];
									scannLaserInfo.hitNormal = Eigen::Vector3d(kPoint.normal_x, kPoint.normal_y, kPoint.normal_z);
									if (std::abs(scannLaserInfo.hitNormal.norm() - 1.0) > 0.05)
									{
										PCL_WARN("[e57::ExportToPCD_Process] scannLaserInfo.hitNormal is not valid!!? Ignore.\n");
									}
									else
									{
										double dotNN = scannLaserInfo.hitNormal.dot(pointNormal);
										if (dotNN > cutGrazing)
										{
											scannLaserInfo.hitPosition = Eigen::Vector3d(kPoint.x, kPoint.y, kPoint.z);
											switch (scanScanInfo.scanner)
											{
											case Scanner::BLK360:
											{
												scannLaserInfo.incidentDirection = scanScanInfo.position - scannLaserInfo.hitPosition;
												scannLaserInfo.hitDistance = scannLaserInfo.incidentDirection.norm();
												scannLaserInfo.incidentDirection /= scannLaserInfo.hitDistance;
												if (scannLaserInfo.incidentDirection.dot(pointNormal) < 0)
													scannLaserInfo.incidentDirection *= -1.0;
												scannLaserInfo.reflectedDirection = scannLaserInfo.incidentDirection; // BLK360 

												// Ref - BLK 360 Spec - laser wavelength & Beam divergence : https://lasers.leica-geosystems.com/global/sites/lasers.leica-geosystems.com.global/files/leica_media/product_documents/blk/853811_leica_blk360_um_v2.0.0_en.pdf
												// Ref - Gaussian beam : https://en.wikipedia.org/wiki/Gaussian_beam
												// Ref - Beam divergence to Beam waist(w0) : http://www2.nsysu.edu.tw/optics/laser/angle.htm
												double temp = scannLaserInfo.hitDistance / 26.2854504782;
												scannLaserInfo.beamFalloff = 1.0f / (1 + temp * temp);
												if ((scannLaserInfo.beamFalloff > cutFalloff))
												{
													scannLaserInfo.hitTangent = scannLaserInfo.hitNormal.cross(tempVec);
													double hitTangentNorm = scannLaserInfo.hitTangent.norm();
													if (hitTangentNorm > 0.0)
													{
														scannLaserInfo.hitTangent /= hitTangentNorm;
														scannLaserInfo.hitBitangent = scannLaserInfo.hitNormal.cross(scannLaserInfo.hitTangent);
														scannLaserInfo.hitBitangent /= scannLaserInfo.hitBitangent.norm();
														scannLaserInfo.weight = std::pow((radius - d) / radius, distInterParm) * std::pow(dotNN, angleInterParm);
														scannLaserInfo.intensity = (double)kPoint.intensity;
														scannLaserInfos.push_back(scannLaserInfo);
													}
													else
													{
														PCL_WARN("[e57::ExportToPCD_Process] scannLaserInfo.hitTangent is not valid!!? Ignore.\n");
													}
												}
											}
											break;

											default:
												PCL_WARN("[e57::ExportToPCD_Process] Scan data Scanner type is not support!!? Ignore.\n");
												break;
											}
										}
									}
								}
							}
						}
						if (scannLaserInfos.size() > 0)
						{
							Eigen::MatrixXf A;
							Eigen::MatrixXf B;
							A = Eigen::MatrixXf(scannLaserInfos.size() * 3, 3);
							B = Eigen::MatrixXf(scannLaserInfos.size() * 3, 1);

							std::size_t shifter = 0;
							for (std::vector<ScannLaserInfo>::const_iterator it = scannLaserInfos.begin(); it != scannLaserInfos.end(); ++it)
							{
								A(shifter, 0) = it->weight * it->incidentDirection.x();
								A(shifter, 1) = it->weight * it->incidentDirection.y();
								A(shifter, 2) = it->weight * it->incidentDirection.z();
								B(shifter, 0) = it->weight * (it->intensity / it->beamFalloff);

								A(shifter + 1, 0) = it->weight * it->hitTangent.x();
								A(shifter + 1, 1) = it->weight * it->hitTangent.y();
								A(shifter + 1, 2) = it->weight * it->hitTangent.z();
								B(shifter + 1, 0) = 0.0;

								A(shifter + 2, 0) = it->weight * it->hitBitangent.x();
								A(shifter + 2, 1) = it->weight * it->hitBitangent.y();
								A(shifter + 2, 2) = it->weight * it->hitBitangent.z();
								B(shifter + 2, 0) = 0.0;

								shifter += 3;
							}

							Eigen::MatrixXf X;
							switch (linearSolver)
							{
							case LinearSolver::EIGEN_QR:
							{
								X = A.colPivHouseholderQr().solve(B);
							}
							break;
							case LinearSolver::EIGEN_SVD:
							{
								X = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
							}
							break;
							case LinearSolver::EIGEN_NE:
							{
								Eigen::MatrixXf localAT = A.transpose();
								X = (localAT * A).ldlt().solve(localAT * B);
							}
							break;
							default:
							{
								throw pcl::PCLException("LinearSolver is not supported.");
							}
							break;
							}

							//
							Eigen::Vector3d xVec(X(0, 0), X(1, 0), X(2, 0));
							if (std::isfinite(xVec.x()) && std::isfinite(xVec.y()) && std::isfinite(xVec.z()))
							{
								double xVecNorm = xVec.norm();
								if (xVecNorm > 0.0)
								{
									point.intensity = xVecNorm;
									xVec /= xVecNorm;
									point.normal_x = xVec.x();
									point.normal_y = xVec.y();
									point.normal_z = xVec.z();
									success = true;
								}
								else
								{
									PCL_WARN("[e57::%s::ComputePointAlbedo] LinearSolver solve zero norm.\n", "AlbedoEstimation");
								}
							}
							else
							{
								PCL_WARN("[e57::%s::ComputePointAlbedo] LinearSolver solve non finite value.\n", "AlbedoEstimation");
							}
						}
					}
					if (!success)
					{
						PCL_WARN("[e57::ExportToPCD_Process] Estimat Albedo - failed, ignore.\n");
						point.intensity = std::numeric_limits<float>::quiet_NaN();
						e57Cloud_CB->is_dense = false;
					}
				}
			}

			//
			if (!e57Cloud_CB->is_dense)
			{
				PCL_INFO("[e57::ExportToPCD_Process] Estimat Albedo - Remove NAN.\n");
				(*outPointCloud)->reserve((*outPointCloud)->size());
				for (pcl::PointCloud<PointExchange>::iterator it = e57Cloud_CB->begin(); it != e57Cloud_CB->end(); ++it)
					if (std::isfinite(it->intensity))
						(*outPointCloud)->push_back(*it);

				std::stringstream ss;
				ss << "[e57::ExportToPCD_Process] Estimat Albedo - Remove NAN - inSize, outSize: " << e57Cloud_CB->size() << ", " << (*outPointCloud)->size() << ".\n";
				PCL_INFO(ss.str().c_str());
			}
			else
			{
				(*outPointCloud)->resize(e57Cloud_CB->size());
				for (std::size_t pi = 0; pi < e57Cloud_CB->size(); ++pi)
					(*(*outPointCloud))[pi] = (*e57Cloud_CB)[pi];
			}

		}

		//
		PCL_INFO("[e57::ExportToPCD_Process] End. \n");
		return 0;
	}

	void Converter::ExportToPCD(const double voxelUnit, const unsigned int searchRadiusNumVoxels, const int meanK, const int polynomialOrder, bool reconstructAlbedo, bool reconstructNDF, const pcl::PointCloud<PointPCD>::Ptr& out, std::vector<pcl::PointCloud<PointNDF>::Ptr>& NDFs)
	{
		try
		{		
			if (reconstructNDF)
			{
				reconstructAlbedo = true;
				if (!PCD_CAN_CONTAIN_LABEL)
					throw pcl::PCLException("You must compile the program with POINT_PCD_WITH_LABEL definition to enable reconstructNDF");
			}

			if (reconstructAlbedo)
			{
				if (!E57_CAN_CONTAIN_LABEL)
					throw pcl::PCLException("You must compile the program with POINT_E57_WITH_LABEL definition to enable reconstructAlbedo");
				if (!E57_CAN_CONTAIN_INTENSITY)
					throw pcl::PCLException("You must compile the program with POINT_E57_WITH_INTENSITY definition to enable reconstructAlbedo");
				if (!PCD_CAN_CONTAIN_INTENSITY)
					throw pcl::PCLException("You must compile the program with POINT_PCD_WITH_INTENSITY definition to enable reconstructAlbedo");
				if (!PCD_CAN_CONTAIN_NORMAL)
					throw pcl::PCLException("You must compile the program with POINT_PCD_WITH_NORMAL definition to enable reconstructAlbedo");
			}

			//
			out->clear();
			std::vector<OCTQuery> querys;
			OCT::Iterator it(*oct);
			while (*it != nullptr)
			{
				if ((*it)->getNodeType() == pcl::octree::LEAF_NODE)
				{
					OCTQuery query;
					query.voxelUnit = voxelUnit;
					query.meanK = meanK;
					query.polynomialOrder = polynomialOrder;
					query.reconstructAlbedo = reconstructAlbedo;
					query.reconstructNDF = reconstructNDF;
					(*it)->getBoundingBox(query.minBB, query.maxBB);
					query.depth = (*it)->getDepth();
					query.searchRadius = voxelUnit * searchRadiusNumVoxels;
					querys.push_back(query);
				}
				it++;
			}

			//
			bool p = false;
			std::vector<pcl::PointCloud<PointE57>::Ptr> rawE57CloudBuffer(2);
			{
				int rQuery = ExportToPCD_Query(&oct, &querys, 0, &rawE57CloudBuffer, p);
				if (rQuery != 0) throw pcl::PCLException("ExportToPCD_Query failed - " + std::to_string(rQuery));
			}
			for (int64_t queryID = 0; queryID < querys.size(); ++queryID)
			{
				pcl::PointCloud<PointPCD>::Ptr outPointCloud(new pcl::PointCloud<PointPCD>);

				//
				std::future<int> query = std::async(ExportToPCD_Query, &oct, &querys, queryID+1, &rawE57CloudBuffer, !p);
				std::future<int> process = std::async(ExportToPCD_Process, &querys, queryID, &rawE57CloudBuffer, p, &scanInfo,  &outPointCloud);
				int rQuery = query.get();
				int rProcess = process.get();
				if (rQuery != 0) throw pcl::PCLException("ExportToPCD_Query failed - " + std::to_string(rQuery));
				if (rProcess != 0) throw pcl::PCLException("ExportToPCD_Process failed - " + std::to_string(rProcess));

				// Merge
				(*out) += (*outPointCloud);
				{
					std::stringstream ss;
					ss << "[e57::%s::ExportToPCD] Final cloud size " << out->size() << " points.\n";
					PCL_INFO(ss.str().c_str(), "Converter");
				}
				p = !p;
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

		if (reconstructNDF)
		{
			//ExportToPCD_ReconstructNDF(voxelUnit, searchRadiusNumVoxels, out, NDFs);
		}
	}

	int ExportToPCD_ReconstructNDF_Process(const std::vector<OCTQuery>* querys, const int64_t queryID, std::vector<pcl::PointCloud<PointE57>::Ptr>* rawE57CloudBuffer, bool p, const pcl::PointCloud<PointPCD>::Ptr* cloud, const std::vector<ScanInfo>* scanInfos, std::vector<pcl::PointCloud<PointNDF>::Ptr>* NDFs)
	{
		if (queryID >= querys->size())
			return 0;
		if ((*rawE57CloudBuffer)[p].get() == nullptr)
			return 1;
		if (cloud->get() == nullptr)
			return 2;

		//
		pcl::PointCloud<PointExchange>::Ptr rawE57Cloud(new pcl::PointCloud<PointExchange>());
		rawE57Cloud->resize((*rawE57CloudBuffer)[p]->size());
		for (std::size_t pi = 0; pi < rawE57Cloud->size(); ++pi)
			(*rawE57Cloud)[pi] = (*(*rawE57CloudBuffer)[p])[pi];
		pcl::search::KdTree<PointExchange>::Ptr rawE57Cloud_tree(new pcl::search::KdTree<PointExchange>());

		pcl::PointCloud<PointExchange>::Ptr e57Cloud(new pcl::PointCloud<PointExchange>);
		pcl::search::KdTree<PointExchange>::Ptr e57Cloud_tree(new pcl::search::KdTree<PointExchange>());

		pcl::PointCloud<PointExchange>::Ptr e57Cloud_CB(new pcl::PointCloud<PointExchange>);

		//
		{
			PCL_INFO("[e57::ExportToPCD_ReconstructNDF_Process] Crop Box Ext.\n");

			Eigen::Vector3d extMinBB;
			Eigen::Vector3d extMaxBB;
			Eigen::Vector3d extXYZ((*querys)[queryID].searchRadius, (*querys)[queryID].searchRadius, (*querys)[queryID].searchRadius);
			extMinBB = (*querys)[queryID].minBB - extXYZ;
			extMaxBB = (*querys)[queryID].maxBB + extXYZ;

			pcl::PointCloud<PointPCD>::Ptr pcdCloud(new pcl::PointCloud<PointPCD>);
			pcl::CropBox<PointPCD> cb;
			cb.setMin(Eigen::Vector4f(extMinBB.x(), extMinBB.y(), extMinBB.z(), 1.0));
			cb.setMax(Eigen::Vector4f(extMaxBB.x(), extMaxBB.y(), extMaxBB.z(), 1.0));
			cb.setInputCloud(*cloud);
			cb.filter(*pcdCloud);

			std::stringstream ss;
			ss << "[e57::ExportToPCD_ReconstructNDF_Process] Crop Box Ext - inSize, outSize: " << (*cloud)->size() << ", " << pcdCloud->size() << ".\n";
			PCL_INFO(ss.str().c_str());

			e57Cloud->resize(pcdCloud->size());
			for (std::size_t pi = 0; pi < pcdCloud->size(); ++pi)
				(*e57Cloud)[pi] = (*pcdCloud)[pi];
		}

		{
			PCL_INFO("[e57::ExportToPCD_ReconstructNDF_Process] Crop Box.\n");

			pcl::CropBox<PointExchange> cb;
			cb.setMin(Eigen::Vector4f((*querys)[queryID].minBB.x(), (*querys)[queryID].minBB.y(), (*querys)[queryID].minBB.z(), 1.0));
			cb.setMax(Eigen::Vector4f((*querys)[queryID].maxBB.x(), (*querys)[queryID].maxBB.y(), (*querys)[queryID].maxBB.z(), 1.0));
			cb.setInputCloud(e57Cloud);
			cb.filter(*e57Cloud_CB);

			std::stringstream ss;
			ss << "[e57::ExportToPCD_ReconstructNDF_Process] Crop Box - inSize, outSize: " << e57Cloud->size() << ", " << e57Cloud_CB->size() << ".\n";
			PCL_INFO(ss.str().c_str());
		}

		PCL_INFO("[e57::ExportToPCD_ReconstructNDF_Process] Upsampling Normal and segmentLabel ID.\n");
		{
			if (e57Cloud_tree->getInputCloud() != e57Cloud)
				e57Cloud_tree->setInputCloud(e57Cloud);

#ifdef _OPENMP
#pragma omp parallel for num_threads(omp_get_num_procs())
#endif
			// Iterating over the entire index vector
			for (int px = 0; px < static_cast<int> (rawE57Cloud->size()); ++px)
			{
				PointExchange& point = (*rawE57Cloud)[px];
				std::vector<int> ki;
				std::vector<float> kd;
				if (e57Cloud_tree->nearestKSearch(point, 1, ki, kd) > 0)
				{
					PointExchange& kPoint = (*e57Cloud)[ki[0]];
					point.normal_x = kPoint.normal_x;
					point.normal_y = kPoint.normal_y;
					point.normal_z = kPoint.normal_z;
					point.curvature = kPoint.curvature;
					point.segmentLabel = kPoint.segmentLabel;
				}
				else
				{
					point.normal_x = 0.0f;
					point.normal_y = 0.0f;
					point.normal_z = 0.0f;
					point.curvature = 0.0f;
					point.hasSegmentLabel = -1;
				}
			}
		}

		PCL_INFO("[e57::ExportToPCD_ReconstructNDF_Process] Reconstruct NDF.\n");
		{
			double cutFalloff = 0.33;
			Eigen::Vector3d tempVec(1.0, 1.0, 1.0);
			tempVec /= tempVec.norm();

/*#ifdef _OPENMP
#pragma omp parallel for num_threads(omp_get_num_procs())
#endif*/
			for (int px = 0; px < static_cast<int> (rawE57Cloud->size()); ++px)
			{
				bool success = false;
				PointExchange& point = (*rawE57Cloud)[px];
				pcl::PointCloud<PointNDF>::Ptr& NDF = (*NDFs)[point.segmentLabel];
				ScannLaserInfo scannLaserInfo;
				scannLaserInfo.hitNormal = Eigen::Vector3d(point.normal_x, point.normal_y, point.normal_z);
				if (std::abs(scannLaserInfo.hitNormal.norm() - 1.0) > 0.05)
				{
					PCL_WARN("[e57::ExportToPCD_ReconstructNDF_Process] scannLaserInfo.hitNormal is not valid!!? Ignore.\n");
				}
				else
				{
					const ScanInfo& scanScanInfo = (*scanInfos)[point.label];
					switch (scanScanInfo.scanner)
					{
					case Scanner::BLK360:
					{
						scannLaserInfo.incidentDirection = scanScanInfo.position - scannLaserInfo.hitPosition;
						scannLaserInfo.hitDistance = scannLaserInfo.incidentDirection.norm();
						scannLaserInfo.incidentDirection /= scannLaserInfo.hitDistance;
						if (scannLaserInfo.incidentDirection.dot(scannLaserInfo.hitNormal) < 0)
							scannLaserInfo.incidentDirection *= -1.0;
						scannLaserInfo.reflectedDirection = scannLaserInfo.incidentDirection; // BLK360 

						// Ref - BLK 360 Spec - laser wavelength & Beam divergence : https://lasers.leica-geosystems.com/global/sites/lasers.leica-geosystems.com.global/files/leica_media/product_documents/blk/853811_leica_blk360_um_v2.0.0_en.pdf
						// Ref - Gaussian beam : https://en.wikipedia.org/wiki/Gaussian_beam
						// Ref - Beam divergence to Beam waist(w0) : http://www2.nsysu.edu.tw/optics/laser/angle.htm
						double temp = scannLaserInfo.hitDistance / 26.2854504782;
						scannLaserInfo.beamFalloff = 1.0f / (1 + temp * temp);
						if ((scannLaserInfo.beamFalloff > cutFalloff))
						{
							scannLaserInfo.hitTangent = scannLaserInfo.hitNormal.cross(tempVec);
							double hitTangentNorm = scannLaserInfo.hitTangent.norm();
							if (hitTangentNorm > 0.0)
							{
								scannLaserInfo.hitTangent /= hitTangentNorm;
								scannLaserInfo.hitBitangent = scannLaserInfo.hitNormal.cross(scannLaserInfo.hitTangent);
								scannLaserInfo.hitBitangent /= scannLaserInfo.hitBitangent.norm();
								scannLaserInfo.weight = 1.0;
								scannLaserInfo.intensity = (double)point.intensity;
								success = true;
							}
							else
							{
								PCL_WARN("[e57::ExportToPCD_ReconstructNDF_Process] scannLaserInfo.hitTangent is not valid!!? Ignore.\n");
							}
						}
					}
					break;

					default:
						PCL_WARN("[e57::ExportToPCD_ReconstructNDF_Process] Scan data Scanner type is not support, ignore.\n");
						break;
					}

					//
					if (success)
					{
						// To tan space
						Eigen::Matrix3d TBN;
						TBN(0, 0) = scannLaserInfo.hitTangent.x();
						TBN(0, 1) = scannLaserInfo.hitTangent.y();
						TBN(0, 2) = scannLaserInfo.hitTangent.z();

						TBN(1, 0) = scannLaserInfo.hitBitangent.x();
						TBN(1, 1) = scannLaserInfo.hitBitangent.y();
						TBN(1, 2) = scannLaserInfo.hitBitangent.z();

						TBN(2, 0) = scannLaserInfo.hitNormal.x();
						TBN(2, 1) = scannLaserInfo.hitNormal.y();
						TBN(2, 2) = scannLaserInfo.hitNormal.z();

						Eigen::Vector3d hitHalfway = scannLaserInfo.incidentDirection + scannLaserInfo.reflectedDirection;
						double hitHalfwayNorm = hitHalfway.norm();
						if (hitHalfwayNorm > 0.0)
						{
							hitHalfway /= hitHalfwayNorm;
							hitHalfway = TBN * hitHalfway;

							PointNDF dataNDF;
							dataNDF.x = hitHalfway.x();
							dataNDF.y = hitHalfway.y();
							dataNDF.z = hitHalfway.z();
							dataNDF.intensity = scannLaserInfo.intensity / scannLaserInfo.beamFalloff;
							NDF->push_back(dataNDF);
						}
						else
						{
							PCL_WARN("[e57::ExportToPCD_ReconstructNDF_Process] hitHalfway is not valid!!? Ignore.\n");
						}
					}
				}
			}
		}

		return 0;
	}

	void Converter::ExportToPCD_ReconstructNDF(const double voxelUnit, const unsigned int searchRadiusNumVoxels, float spatialImportance, float normalImportance, const pcl::PointCloud<PointPCD>::Ptr& cloud, std::vector<pcl::PointCloud<PointNDF>::Ptr>& NDFs)
	{
		try
		{
			if (!PCD_CAN_CONTAIN_LABEL)
				throw pcl::PCLException("You must compile the program with POINT_PCD_WITH_LABEL definition to enable ExportToPCD_ReconstructNDF");
			if (!E57_CAN_CONTAIN_LABEL)
				throw pcl::PCLException("You must compile the program with POINT_E57_WITH_LABEL definition to enable ExportToPCD_ReconstructNDF");
			if (!E57_CAN_CONTAIN_INTENSITY)
				throw pcl::PCLException("You must compile the program with POINT_E57_WITH_INTENSITY definition to enable ExportToPCD_ReconstructNDF");
			if (!PCD_CAN_CONTAIN_INTENSITY)
				throw pcl::PCLException("You must compile the program with POINT_PCD_WITH_INTENSITY definition to enable ExportToPCD_ReconstructNDF");
			if (!PCD_CAN_CONTAIN_NORMAL)
				throw pcl::PCLException("You must compile the program with POINT_PCD_WITH_NORMAL definition to enable ExportToPCD_ReconstructNDF");

			//
			NDFs.clear();
			
			// Segment
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudXYZRGBA(new pcl::PointCloud<pcl::PointXYZRGBA>());
			pcl::PointCloud<pcl::Normal>::Ptr cloudNormal(new pcl::PointCloud<pcl::Normal>());
			cloudXYZRGBA->resize(cloud->size());
			cloudNormal->resize(cloud->size());
			for (std::size_t px = 0; px < cloudXYZRGBA->size(); px++)
			{
				PointPCD& pcd = (*cloud)[px];
				pcl::PointXYZRGBA& point = (*cloudXYZRGBA)[px];
				pcl::Normal& normal = (*cloudNormal)[px];

				point.x = pcd.x;
				point.y = pcd.y;
				point.z = pcd.z;
				/*unsigned char color = (unsigned char)std::max(std::min((255.f * pcd.intensity), 255.f), 0.0f);
				point.r = color;
				point.g = color;
				point.b = color;*/
				point.rgb = pcd.intensity;
				normal.normal_x = pcd.normal_x;
				normal.normal_y = pcd.normal_y;
				normal.normal_z = pcd.normal_z;
			}

			//
			PCL_INFO(("[e57::%s::ExportToPCD_ReconstructNDF] Segment Start. voxel_resolution " + std::to_string((float)voxelUnit) + ", seed_resolution  " + std::to_string((float)(voxelUnit * searchRadiusNumVoxels)) + ".\n").c_str(), "Converter");
			e57::SupervoxelClustering<pcl::PointXYZRGBA> super((float)voxelUnit, (float)(voxelUnit * searchRadiusNumVoxels));
			//super.setUseSingleCameraTransform(false);
			super.setInputCloud(cloudXYZRGBA);
			super.setNormalCloud(cloudNormal);
			super.setColorImportance(255.0);
			super.setSpatialImportance(spatialImportance);
			super.setNormalImportance(normalImportance);
			std::map <uint32_t, e57::Supervoxel<pcl::PointXYZRGBA>::Ptr > clusters;
			super.extract(clusters);
			pcl::PointCloud<pcl::PointXYZL>::Ptr cloudXYZL = super.getLabeledCloud();
			PCL_INFO(("[e57::%s::ExportToPCD_ReconstructNDF] Segment End. Segment " + std::to_string(clusters.size()) + ", Size " + std::to_string(cloudXYZL->size()) + ".\n").c_str(), "Converter");
			pcl::search::KdTree<pcl::PointXYZL>::Ptr cloudXYZL_tree(new pcl::search::KdTree<pcl::PointXYZL>());
			if (cloudXYZL_tree->getInputCloud() != cloudXYZL)
				cloudXYZL_tree->setInputCloud(cloudXYZL);

			PCL_INFO("[e57::%s::ExportToPCD_ReconstructNDF] Upsampling Segment ID.\n", "Converter");
			{
#ifdef _OPENMP
#pragma omp parallel for num_threads(omp_get_num_procs())
#endif
				for (int px = 0; px < static_cast<int> (cloud->size()); ++px)
				{
					PointPCD& point = (*cloud)[px];
					std::vector<int> ki;
					std::vector<float> kd;
					pcl::PointXYZL c;
					c.x = point.x;
					c.y = point.y;
					c.z = point.z;
					if (cloudXYZL_tree->nearestKSearch(c, 1, ki, kd) > 0)
						point.label = (*cloudXYZL)[ki[0]].label;
					else
						point.hasLabel = -1;
				}
			}

			//
			for (std::size_t i = 0; i < clusters.size(); ++i)
			{
				pcl::PointCloud<PointNDF>::Ptr NDF (new pcl::PointCloud<PointNDF>());
				NDF->reserve(1000);
				NDFs.push_back(NDF);
			}
			std::vector<OCTQuery> querys;
			OCT::Iterator it(*oct);
			while (*it != nullptr)
			{
				if ((*it)->getNodeType() == pcl::octree::LEAF_NODE)
				{
					OCTQuery query;
					query.voxelUnit = voxelUnit;
					query.meanK = -1;
					query.polynomialOrder = -1;
					query.reconstructAlbedo = false;
					query.reconstructNDF = true;
					(*it)->getBoundingBox(query.minBB, query.maxBB);
					query.depth = (*it)->getDepth();
					query.searchRadius = voxelUnit * searchRadiusNumVoxels;
					querys.push_back(query);
				}
				it++;
			}
			//
			PCL_INFO("[e57::%s::ExportToPCD_ReconstructNDF] Reconstruct NDF.\n", "Converter");
			bool p = false;
			std::vector<pcl::PointCloud<PointE57>::Ptr> rawE57CloudBuffer(2);
			{
				int rQuery = ExportToPCD_Query(&oct, &querys, 0, &rawE57CloudBuffer, p);
				if (rQuery != 0) throw pcl::PCLException("ExportToPCD_ReconstructNDF_Query failed - " + std::to_string(rQuery));
			}
			for (int64_t queryID = 0; queryID < querys.size(); ++queryID)
			{
				std::future<int> query = std::async(ExportToPCD_Query, &oct, &querys, queryID + 1, &rawE57CloudBuffer, !p);
				std::future<int> process = std::async(ExportToPCD_ReconstructNDF_Process, &querys, queryID, &rawE57CloudBuffer, p, &cloud, &scanInfo, &NDFs);
				int rQuery = query.get();
				int rProcess = process.get();
				if (rQuery != 0) throw pcl::PCLException("ExportToPCD_ReconstructNDF_Query failed - " + std::to_string(rQuery));
				if (rProcess != 0) throw pcl::PCLException("ExportToPCD_ReconstructNDF_Process failed - " + std::to_string(rProcess));
				p = !p;
			}
		}
		catch (std::exception& ex)
		{
			std::stringstream ss;
			ss << "[e57::%s::ExportToPCD_ReconstructNDF] Got an std::exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "Converter");
		}
		catch (...)
		{
			PCL_INFO("[e57::%s::ExportToPCD_ReconstructNDF] Got an unknown exception.\n", "Converter");
		}
	}
}
