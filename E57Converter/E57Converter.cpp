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

#include "E57Utils.h"
#include "E57Converter.h"
#include "E57AlbedoEstimation.h"
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

	//
	struct OCTQuery
	{
		double voxelUnit;
		int meanK;
		int polynomialOrder;
		bool reconstructAlbedo;
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

	int ExportToPCD_Process(const std::vector<OCTQuery>* querys, const int64_t queryID, std::vector<pcl::PointCloud<PointE57>::Ptr>* rawE57CloudBuffer, bool p, const std::vector<ScanInfo>* scanInfo, pcl::PointCloud<PointPCD>::Ptr* outPointCloud)
	{
		if (queryID >= querys->size())
			return 0;
		if ((*rawE57CloudBuffer)[p].get() == nullptr)
			return 1;
		if (outPointCloud->get() == nullptr)
			return 2;

		std::stringstream ss;
		ss << "[e57::ExportToPCD_Process] Start - query" << queryID << "/" << querys->size() << ".\n";
		PCL_INFO(ss.str().c_str());

		//
		pcl::PointCloud<PointE57>::Ptr e57Cloud(new pcl::PointCloud<PointE57>);			

		// DownSampling
		{
			PCL_INFO("[e57::ExportToPCD_Process] DownSampling.\n");
			pcl::VoxelGrid<PointE57> vf;
			vf.setLeafSize((*querys)[queryID].voxelUnit, (*querys)[queryID].voxelUnit, (*querys)[queryID].voxelUnit);
			vf.setInputCloud((*rawE57CloudBuffer)[p]);
			vf.filter(*e57Cloud);
			std::stringstream ss;
			ss << "[e57::ExportToPCD_Process] DownSampling - inSize, outSize: " << (*rawE57CloudBuffer)[p]->size() << ", " << e57Cloud->size() << ".\n";
			PCL_INFO(ss.str().c_str());
		}

		//Outlier Removal
		if ((*querys)[queryID].meanK > 0)
		{
			PCL_INFO("[e57::ExportToPCD_Process] Outlier Removal.\n");

			pcl::PointCloud<PointE57>::Ptr e57Cloud_OLR(new pcl::PointCloud<PointE57>);
			pcl::StatisticalOutlierRemoval<PointE57> olr;
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
			(*outPointCloud)->resize(e57Cloud->size());
			for (std::size_t pi = 0; pi < e57Cloud->size(); ++pi)
				(*(*outPointCloud))[pi].FromPointE57((*e57Cloud)[pi]);

			pcl::search::KdTree<PointE57>::Ptr tree(new pcl::search::KdTree<PointE57>());
			pcl::MovingLeastSquares<PointE57, PointPCD> mls;
			mls.setComputeNormals(PCD_CAN_CONTAIN_NORMAL);
			mls.setPolynomialOrder((*querys)[queryID].polynomialOrder);
			mls.setSearchMethod(tree);
			mls.setSearchRadius((*querys)[queryID].searchRadius);
			mls.setInputCloud(e57Cloud);
			mls.process(*(*outPointCloud));

			// Crop Box
			pcl::PointCloud<PointPCD>::Ptr pcdCloud_CB(new pcl::PointCloud<PointPCD>);
			{
				PCL_INFO("[e57::ExportToPCD_Process] Estimat Surface - Crop Box.\n");

				pcl::CropBox<PointPCD> cb;
				cb.setMin(Eigen::Vector4f((*querys)[queryID].minBB.x(), (*querys)[queryID].minBB.y(), (*querys)[queryID].minBB.z(), 1.0));
				cb.setMax(Eigen::Vector4f((*querys)[queryID].maxBB.x(), (*querys)[queryID].maxBB.y(), (*querys)[queryID].maxBB.z(), 1.0));
				cb.setInputCloud(*outPointCloud);
				cb.filter(*pcdCloud_CB);

				std::stringstream ss;
				ss << "[e57::ExportToPCD_Process] Estimat Surface - Crop Box - inSize, outSize: " << (*outPointCloud)->size() << ", " << pcdCloud_CB->size() << ".\n";
				PCL_INFO(ss.str().c_str());
			}

			(*outPointCloud) = pcdCloud_CB;
		}
		else // Estimat Normal
		{
			PCL_INFO("[e57::ExportToPCD_Process] Estimat Normal.\n");

			// Crop Box
			pcl::PointCloud<PointE57>::Ptr e57Cloud_CB(new pcl::PointCloud<PointE57>);
			{
				PCL_INFO("[e57::ExportToPCD_Process] Estimat Normal - Crop Box.\n");

				pcl::CropBox<PointE57> cb;
				cb.setMin(Eigen::Vector4f((*querys)[queryID].minBB.x(), (*querys)[queryID].minBB.y(), (*querys)[queryID].minBB.z(), 1.0));
				cb.setMax(Eigen::Vector4f((*querys)[queryID].maxBB.x(), (*querys)[queryID].maxBB.y(), (*querys)[queryID].maxBB.z(), 1.0));
				cb.setInputCloud(e57Cloud);
				cb.filter(*e57Cloud_CB);

				std::stringstream ss;
				ss << "[e57::ExportToPCD_Process] Estimat Normal - Crop Box - inSize, outSize: " << e57Cloud->size() << ", " << e57Cloud_CB->size() << ".\n";
				PCL_INFO(ss.str().c_str());
			}

			// Copy to pcdCloud
			(*outPointCloud)->resize(e57Cloud_CB->size());
			for (std::size_t pi = 0; pi < e57Cloud_CB->size(); ++pi)
				(*(*outPointCloud))[pi].FromPointE57((*e57Cloud_CB)[pi]);

			//
			pcl::search::KdTree<PointE57>::Ptr tree(new pcl::search::KdTree<PointE57>());
			pcl::NormalEstimationOMP<PointE57, PointPCD> ne;
			ne.setSearchMethod(tree);
			ne.setRadiusSearch((*querys)[queryID].searchRadius);
			ne.setSearchSurface(e57Cloud);
			ne.setInputCloud(e57Cloud_CB);
			ne.compute(*(*outPointCloud));
		}

		// Estimate albedo
		if ((*querys)[queryID].reconstructAlbedo)
		{
			PCL_INFO("[e57::ExportToPCD_Process] Estimat Albedo.\n");

			pcl::search::KdTree<PointE57>::Ptr tree(new pcl::search::KdTree<PointE57>());
			AlbedoEstimationOMP ae(*scanInfo);
			ae.setSearchMethod(tree);
			ae.setRadiusSearch((*querys)[queryID].searchRadius);
			ae.setSearchSurface((*rawE57CloudBuffer)[p]);
			ae.setInputCloud(e57Cloud);
			ae.compute(*(*outPointCloud));
		}

		//
		PCL_INFO("[e57::ExportToPCD_Process] End. \n");
		return 0;
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
			//
			out.clear();
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
				out += (*outPointCloud);
				{
					std::stringstream ss;
					ss << "[e57::%s::ExportToPCD] Final cloud size " << out.size() << " points.\n";
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
	}
}
