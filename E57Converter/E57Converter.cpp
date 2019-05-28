#pragma once

#include "E57Converter.h"

#include <vector>
#include <iomanip>
#include <fstream>
#include <assert.h>
#include <cmath>

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>

namespace e57
{
#define TYPE_SPACE std::left << std::setw(6) 
#define NAME_SPACE std::left << std::setw(12) 
#define NUMBER_SPACE std::left << std::setprecision(6) << std::setw(9) 

	std::string NodeTypeStr(NodeType t)
	{
		switch (t)
		{
		case e57::NodeType::E57_STRUCTURE: return std::string("E57_STRUCTURE");
		case e57::NodeType::E57_VECTOR: return std::string("E57_VECTOR");
		case e57::NodeType::E57_COMPRESSED_VECTOR: return std::string("E57_COMPRESSED_VECTOR");
		case e57::NodeType::E57_INTEGER: return std::string("E57_INTEGER");
		case e57::NodeType::E57_SCALED_INTEGER: return std::string("E57_SCALED_INTEGER");
		case e57::NodeType::E57_FLOAT: return std::string("E57_FLOAT");
		case e57::NodeType::E57_STRING: return std::string("E57_STRING");
		case e57::NodeType::E57_BLOB: return std::string("E57_BLOB");
		default: return std::string("UNKNOWN");
		}
	}

	void ParseNode(std::size_t pDepth, const e57::StructureNode& pNode)
	{
		std::cout << std::string(pDepth, '\t') << TYPE_SPACE << "STRUCT" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.childCount() << std::endl;
		std::cout << std::string(pDepth, '\t') << "{" << std::endl;
		pDepth++;

		//
		for (int64_t i = 0; i < pNode.childCount(); ++i)
			ParseNode(pDepth, pNode.get(i));

		//
		std::cout << std::string(pDepth - 1, '\t') << "}" << std::endl;
	}

	void ParseNode(std::size_t pDepth, const e57::VectorNode& pNode)
	{
		std::cout << std::string(pDepth, '\t') << TYPE_SPACE << "VEC" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.childCount() << std::endl;
		std::cout << std::string(pDepth, '\t') << "{" << std::endl;
		pDepth++;

		//
		for (int64_t i = 0; i < pNode.childCount(); ++i)
			ParseNode(pDepth, pNode.get(i));

		//
		std::cout << std::string(pDepth - 1, '\t') << "}" << std::endl;
	}

	void ParseNode(std::size_t pDepth, const e57::CompressedVectorNode& pNode)
	{
		std::cout << std::string(pDepth, '\t') << TYPE_SPACE << "CP_VEV" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.childCount() << std::endl;
		std::cout << std::string(pDepth, '\t') << "{" << std::endl;
		pDepth++;

		//
		ParseNode(pDepth, pNode.prototype());
		ParseNode(pDepth, pNode.codecs());

		//
		std::cout << std::string(pDepth - 1, '\t') << "}" << std::endl;
	}

	void ParseNode(std::size_t pDepth, const e57::IntegerNode& pNode)
	{
		std::cout << std::string(pDepth, '\t') << TYPE_SPACE << "INT" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.value() << std::endl;
	}

	void ParseNode(std::size_t pDepth, const e57::ScaledIntegerNode& pNode)
	{
		std::cout << std::string(pDepth, '\t') << TYPE_SPACE << "SC_INT" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.scaledValue() << ", " << pNode.rawValue() << ", " << pNode.scale() << ", " << pNode.offset() << std::endl;
	}

	void ParseNode(std::size_t pDepth, const e57::FloatNode& pNode)
	{
		std::cout << std::string(pDepth, '\t') << TYPE_SPACE << "FLOAT" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.value() << std::endl;
	}

	void ParseNode(std::size_t pDepth, const e57::StringNode& pNode)
	{
		std::cout << std::string(pDepth, '\t') << TYPE_SPACE << "STR" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.value() << std::endl;
	}

	void ParseNode(std::size_t pDepth, const e57::BlobNode& pNode)
	{
		std::cout << std::string(pDepth, '\t') << TYPE_SPACE << "BLOB" << "*" << NAME_SPACE << pNode.elementName() << " : " << NUMBER_SPACE << pNode.byteCount() << std::endl;
		std::cout << std::string(pDepth, '\t') << "{" << std::endl;
		pDepth++;

		//
		std::cout << std::string(pDepth - 1, '\t') << "}" << std::endl;
	}

	void ParseNode(std::size_t pDepth, const e57::Node& pNode)
	{
		switch (pNode.type())
		{
		case e57::NodeType::E57_STRUCTURE: return ParseNode(pDepth, e57::StructureNode(pNode));
		case e57::NodeType::E57_VECTOR: return ParseNode(pDepth, e57::VectorNode(pNode));
		case e57::NodeType::E57_COMPRESSED_VECTOR: return ParseNode(pDepth, e57::CompressedVectorNode(pNode));
		case e57::NodeType::E57_INTEGER: return ParseNode(pDepth, e57::IntegerNode(pNode));
		case e57::NodeType::E57_SCALED_INTEGER: return ParseNode(pDepth, e57::ScaledIntegerNode(pNode));
		case e57::NodeType::E57_FLOAT: return ParseNode(pDepth, e57::FloatNode(pNode));
		case e57::NodeType::E57_STRING: return ParseNode(pDepth, e57::StringNode(pNode));
		case e57::NodeType::E57_BLOB: return ParseNode(pDepth, e57::BlobNode(pNode));
		default: return;
		}
	}
}

namespace pcl
{
	void E57Converter::ReadScan(const e57::ImageFile& imf, const e57::VectorNode& data3D, int64_t scanID, pcl::PointCloud<PointE57>& scanCloud)
	{
		e57::StructureNode scan(data3D.get(scanID));

		//
		E57CoodSys cs = E57CoodSys::UNKNOWN;
		Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
		bool hasPointXYZ = false;
		bool hasPointRGB = false;
		bool hasPointI = false;
		std::vector<e57::SourceDestBuffer> sdBuffers;
		std::vector<float> x;
		std::vector<float> y;
		std::vector<float> z;
		std::vector<float> i;
		std::vector<uint8_t> r;
		std::vector<uint8_t> g;
		std::vector<uint8_t> b;

		// Parse pose
		if (scan.isDefined("pose"))
		{
			e57::StructureNode scanPose(scan.get("pose"));

			if (scanPose.isDefined("translation"))
			{
				e57::StructureNode scanPoseTranslation(scanPose.get("translation"));
				transform.block(0, 3, 3, 1) =
					Eigen::Vector3d(
						e57::FloatNode(scanPoseTranslation.get("x")).value(),
						e57::FloatNode(scanPoseTranslation.get("y")).value(),
						e57::FloatNode(scanPoseTranslation.get("z")).value());
			}
			else
				PCL_WARN("[pcl::%s::ReadScan] Scan didnot define pose translation.\n", "E57Converter");


			if (scanPose.isDefined("rotation"))
			{
				e57::StructureNode scanPoseRotation(scanPose.get("rotation"));

				transform.block(0, 0, 3, 3) =
					Eigen::Quaterniond(
						e57::FloatNode(scanPoseRotation.get("w")).value(),
						e57::FloatNode(scanPoseRotation.get("x")).value(),
						e57::FloatNode(scanPoseRotation.get("y")).value(),
						e57::FloatNode(scanPoseRotation.get("z")).value()).toRotationMatrix();
			}
			else
				PCL_WARN("[pcl::%s::ReadScan] Scan didnot define pose rotation.\n", "E57Converter");

			std::stringstream ss;
			ss << "[pcl::%s::ReadScan] Scan transform - " << transform << ".\n";
			PCL_INFO(ss.str().c_str(), "E57Converter");
		}
		else
			PCL_WARN("[pcl::%s::ReadScan] Scan didnot define pose.\n", "E57Converter");

		// Parse scale & offset
		if (scan.isDefined("points"))
		{
			e57::Node scanPointsNode = scan.get("points");

			if (scanPointsNode.type() == e57::NodeType::E57_COMPRESSED_VECTOR)
			{
				e57::CompressedVectorNode scanPoints(scanPointsNode);
				e57::StructureNode proto(scanPoints.prototype());
				std::shared_ptr<e57::Node> protoXNode;
				std::shared_ptr<e57::Node> protoYNode;
				std::shared_ptr<e57::Node> protoZNode;
				std::shared_ptr<e57::Node> protoRNode;
				std::shared_ptr<e57::Node> protoGNode;
				std::shared_ptr<e57::Node> protoBNode;
				std::shared_ptr<e57::Node> protoINode;

				if (proto.isDefined("cartesianX") && proto.isDefined("cartesianY") && proto.isDefined("cartesianZ"))
				{
					cs = E57CoodSys::XYZ;
					hasPointXYZ = true;
					protoXNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("cartesianX")));
					protoYNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("cartesianY")));
					protoZNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("cartesianZ")));

					x.resize(scanPoints.childCount());
					y.resize(scanPoints.childCount());
					z.resize(scanPoints.childCount());
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "cartesianX", &x[0], scanPoints.childCount(), true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "cartesianY", &y[0], scanPoints.childCount(), true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "cartesianZ", &z[0], scanPoints.childCount(), true, true));

				}

				if (proto.isDefined("sphericalRange") && proto.isDefined("sphericalAzimuth") && proto.isDefined("sphericalElevation"))
				{
					cs = E57CoodSys::RAE;
					hasPointXYZ = true;
					protoXNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("sphericalRange")));
					protoYNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("sphericalAzimuth")));
					protoZNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("sphericalElevation")));

					x.resize(scanPoints.childCount());
					y.resize(scanPoints.childCount());
					z.resize(scanPoints.childCount());
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalRange", &x[0], scanPoints.childCount(), true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalAzimuth", &y[0], scanPoints.childCount(), true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalElevation", &z[0], scanPoints.childCount(), true, true));

				}

				if (proto.isDefined("colorRed") && proto.isDefined("colorGreen") && proto.isDefined("colorBlue"))
				{
					hasPointRGB = true;
					protoRNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("colorRed")));
					protoGNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("colorGreen")));
					protoBNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("colorBlue")));

					r.resize(scanPoints.childCount());
					g.resize(scanPoints.childCount());
					b.resize(scanPoints.childCount());
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "colorRed", &r[0], scanPoints.childCount(), true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "colorGreen", &g[0], scanPoints.childCount(), true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "colorBlue", &b[0], scanPoints.childCount(), true, true));
				}

				if (proto.isDefined("intensity"))
				{
					hasPointI = true;
					protoINode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("intensity")));

					i.resize(scanPoints.childCount());
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "intensity", &i[0], scanPoints.childCount(), true, true));

				}

				//
				if (hasPointXYZ || hasPointRGB || hasPointI)
				{
					e57::CompressedVectorReader reader = scanPoints.reader(sdBuffers);
					if (reader.read() <= 0)
						PCL_WARN("[pcl::%s::ReadScan] Failed to read E57 points, ignore the scan.\n", "E57Converter");
					else
					{
						for (int64_t pi = 0; pi < scanPoints.childCount(); ++pi)
						{
							PointE57 sp;

							if (hasPointXYZ)
							{								
								switch (cs)
								{
								case E57CoodSys::XYZ:
									sp.x = x[pi];
									sp.y = y[pi];
									sp.z = z[pi];
									break;

								case E57CoodSys::RAE:
								{
									double s_t = std::sin(z[pi]);
									double s_p = std::sin(y[pi]);
									double c_t = std::cos(z[pi]);
									double c_p = std::cos(y[pi]);

									sp.x = x[pi] * c_t * c_p;
									sp.y = x[pi] * c_t * s_p;
									sp.z = x[pi] * s_t;
								}
								break;

								default:
									throw PCLException("Coordinate system invalid!!?");
									break;
								}
							}

							if (hasPointRGB)
							{
								sp.r = r[pi];
								sp.g = g[pi];
								sp.b = b[pi];
							}
							else
							{
								sp.r = 255;
								sp.g = 255;
								sp.b = 255;
							}

							if (hasPointI)
							{
								sp.intensity = i[pi];
							}
							sp.scanID = scanID;

							if (sp.Valid())
								scanCloud.push_back(sp);
						}
					}
					reader.close();
				}
			}
			else
				PCL_WARN("[pcl::%s::ReadScan] Not supported scan points type.\n", "E57Converter");
		}
		else
			PCL_WARN("[pcl::%s::ReadScan] Scan didnot define points.\n", "E57Converter");

		//
		pcl::transformPointCloud(scanCloud, scanCloud, transform);
	}

	E57Converter::E57Converter(const boost::filesystem::path& octPath)
	{
		try
		{
			oct = OCT::Ptr(new OCT(octPath / boost::filesystem::path("root.oct_idx"), true));
		}
		catch (std::exception& ex)
		{
			std::stringstream ss;
			ss << "[pcl::%s::E57Converter] Got an std::exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "E57Converter");
		}
		catch (...)
		{
			PCL_INFO("[pcl::%s::E57Converter] Got an unknown exception.\n", "E57Converter");
		}
	}

	E57Converter::E57Converter(const boost::filesystem::path& filePath, const boost::filesystem::path& octPath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution_arg, const double sample_percent_arg, const std::string& coord_sys)
	{
		try
		{
			//
			oct = OCT::Ptr(new OCT(min, max, resolution_arg, octPath / boost::filesystem::path("root.oct_idx"), coord_sys));

			//
			e57::ImageFile imf(filePath.string().c_str(), "r");
			e57::VectorNode data3D(imf.root().get("data3D"));
			e57::VectorNode images2D(imf.root().get("images2D"));

			for (int64_t scanID = 0; scanID < std::min(data3D.childCount(), (int64_t)E57_MAX_SCAN); ++scanID)
			{
				pcl::PointCloud<PointE57>::Ptr scanCloud = pcl::PointCloud<PointE57>::Ptr(new pcl::PointCloud<PointE57>);
				ReadScan(imf, data3D, scanID, *scanCloud);
				{
					std::stringstream ss;
					ss << "[pcl::%s::E57Converter] OutOfCoreOctree addPointCloud - scann" << scanID << ".\n";
					PCL_INFO(ss.str().c_str(), "E57Converter");
				}
				oct->addPointCloud(scanCloud);
			}
			{
				std::stringstream ss;
				ss << "[pcl::%s::E57Converter] OutOfCoreOctree buildLOD - sample_percent_arg " << sample_percent_arg << ".\n";
				PCL_INFO(ss.str().c_str(), "E57Converter");
			}
			oct->setSamplePercent(sample_percent_arg);
			oct->buildLOD();
			imf.close();
		}
		catch (e57::E57Exception& ex)
		{
			std::stringstream ss;
			ss << "[pcl::%s::E57Converter] Got an e57::E57Exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "E57Converter");
		}
		catch (std::exception& ex)
		{
			std::stringstream ss;
			ss << "[pcl::%s::E57Converter] Got an std::exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "E57Converter");
		}
		catch (...)
		{
			PCL_INFO("[pcl::%s::E57Converter] Got an unknown exception.\n", "E57Converter");
		}
	}

	void E57Converter::BuildLOD(const double sample_percent_arg)
	{
		try
		{
			oct->setSamplePercent(sample_percent_arg);
			oct->buildLOD();
		}
		catch (std::exception& ex)
		{
			std::stringstream ss;
			ss << "[pcl::%s::BuildLOD] Got an std::exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "E57Converter");
		}
		catch (...)
		{
			PCL_INFO("[pcl::%s::BuildLOD] Got an unknown exception.\n", "E57Converter");
		}
	}

	void E57Converter::DownSampling(const double voxelUnit, pcl::PointCloud<PointE57>& out)
	{
		try
		{
			OCT::Iterator it(*oct);
			out.clear();
			std::size_t leafID = 0;
			while (*it != nullptr)
			{
				if ((*it)->getNodeType() == pcl::octree::LEAF_NODE)
				{
					{
						std::stringstream ss;
						ss << "[pcl::%s::DownSampling] DownSampling leaf " << leafID << ".\n";
						PCL_INFO(ss.str().c_str(), "E57Converter");
					}

					pcl::PointCloud<PointE57>::Ptr nodeCloud(new pcl::PointCloud<PointE57>);
					pcl::PointCloud<PointE57>::Ptr fNodeCloud(new pcl::PointCloud<PointE57>);

					pcl::io::loadPCDFile((*it)->getPCDFilename().string(), *nodeCloud);

					pcl::VoxelGrid<PointE57> sor;
					sor.setInputCloud(nodeCloud);
					sor.setLeafSize(voxelUnit, voxelUnit, voxelUnit);
					sor.filter(*fNodeCloud);

					out += (*fNodeCloud);

					{
						std::stringstream ss;
						ss << "[pcl::%s::DownSampling] DownSampling leaf " << leafID << " end, final cloud total " << out.size() << " points.\n";
						PCL_INFO(ss.str().c_str(), "E57Converter");
					}
					leafID++;
				}
				
				it++;
			}
		}
		catch (std::exception& ex)
		{
			std::stringstream ss;
			ss << "[pcl::%s::DownSampling] Got an std::exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "E57Converter");
		}
		catch (...)
		{
			PCL_INFO("[pcl::%s::DownSampling] Got an unknown exception.\n", "E57Converter");
		}
	}

	void E57Converter::PrintE57Format(const boost::filesystem::path& filePath)
	{
		try
		{
			e57::ImageFile imf(filePath.string().c_str(), "r");
			e57::ParseNode(0, imf.root());
		}
		catch (e57::E57Exception& ex)
		{
			std::stringstream ss;
			ss << "[pcl::%s::PrintE57Format] Got an e57::E57Exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "E57Converter");
		}
		catch (std::exception& ex)
		{
			std::stringstream ss;
			ss << "[pcl::%s::PrintE57Format] Got an std::exception, what=" << ex.what() << ".\n";
			PCL_INFO(ss.str().c_str(), "E57Converter");
		}
		catch (...)
		{
			PCL_INFO("[pcl::%s::PrintE57Format] Got an unknown exception.\n", "E57Converter");
		}
	}
}
