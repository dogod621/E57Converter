#pragma once

#include <string>
#include <memory>

#include "PointType.h"

#include <boost/filesystem.hpp>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <E57Format.h>

namespace e57
{
	std::string NodeTypeStr(NodeType t);

	void ParseNode(std::size_t pDepth, const e57::StructureNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::VectorNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::CompressedVectorNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::IntegerNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::ScaledIntegerNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::FloatNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::StringNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::BlobNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::Node& pNode);

	void PrintFormat(const boost::filesystem::path& e57Path);

	// http://www.libe57.org/bestCoordinates.html
	enum CoodSys
	{
		CoodSys_UNKNOWN = 0,
		XYZ = 1,
		RAE = 2
	};

	std::string CoodSysStr(CoodSys type);

	class Scan
	{
	public:
		CoodSys coodSys;
		Eigen::Matrix4d transform;
		bool hasPointXYZ;
		bool hasPointRGB;
		bool hasPointI;
		std::shared_ptr<float> x;
		std::shared_ptr<float> y;
		std::shared_ptr<float> z;
		std::shared_ptr<float> i;
		std::shared_ptr<uint8_t> r;
		std::shared_ptr<uint8_t> g;
		std::shared_ptr<uint8_t> b;
		std::size_t ID;
		std::size_t numPoints;
		
		Scan()
			: coodSys(CoodSys::CoodSys_UNKNOWN),
			transform(Eigen::Matrix4d::Identity()),
			hasPointXYZ(false), hasPointRGB(false), hasPointI(false), ID(0), numPoints(0) {}

		void Load(const e57::ImageFile& imf, const e57::VectorNode& data3D, int64_t scanID);
		void ToPointCloud(pcl::PointCloud<PointE57>& scanCloud);
	};
}