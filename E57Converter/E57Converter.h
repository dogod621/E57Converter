#pragma once

#define PCL_NO_PRECOMPILE

#include <iostream>

#include <E57Format.h>
#include <boost/assign.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/outofcore/outofcore.h>
#include <pcl/outofcore/outofcore_impl.h>

//
struct _PointE57
{
	PCL_ADD_POINT4D;
	PCL_ADD_RGB;
	PCL_ADD_INTENSITY;
	struct
	{
		union
		{
			uint32_t label;
			int32_t hasLabel;
		};
	};
	struct
	{
		union
		{
			uint32_t scanID;
			int32_t hasScanID;
		};
	};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointE57 : public _PointE57
{
	inline PointE57(const PointE57 &p)
	{
		x = p.x;
		y = p.y;
		z = p.z;
		data[3] = 1.0f;
		rgba = p.rgba;
		intensity = p.intensity;
		label = p.label;
		scanID = p.scanID;
	}

	inline PointE57()
	{
		x = y = z = 0.0f;
		data[3] = 1.f;
		r = g = b = 0;
		a = 1;
		intensity = 0.f;
		hasLabel = hasScanID = -1;
	}

	inline void Clear()
	{
		x = y = z = 0.0f;
		data[3] = 1.f;
		r = g = b = 0;
		a = 1;
		intensity = 0.f;
		hasLabel = hasScanID = -1;
	}

	inline bool Valid()
	{
		return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(intensity) && HasScanID() && ((r > 5) || (g > 5) || (b > 5));
	}

	inline bool HasLabel()
	{
		return !(hasLabel == -1);
	}

	inline bool HasScanID()
	{
		return !(hasScanID == -1);
	}
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointE57,
(float, x, x)
(float, y, y)
(float, z, z)
(uint32_t, rgba, rgba)
(float, intensity, intensity)
(uint32_t, label, label)
(uint32_t, scanID, scanID)
)

POINT_CLOUD_REGISTER_POINT_WRAPPER(PointE57, PointE57)


#define E57_MAX_SCAN 999999

//
namespace e57
{
	//
	std::string NodeTypeStr(NodeType t);

	// Parse Node
	void ParseNode(std::size_t pDepth, const e57::StructureNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::VectorNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::CompressedVectorNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::IntegerNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::ScaledIntegerNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::FloatNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::StringNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::BlobNode& pNode);

	void ParseNode(std::size_t pDepth, const e57::Node& pNode);
}

namespace pcl
{
	// http://www.libe57.org/bestCoordinates.html
	enum E57CoodSys
	{
		UNKNOWN = 0,
		XYZ = 1,
		RAE = 2
	};

	class E57Converter
	{
	protected:
		using OCT = pcl::outofcore::OutofcoreOctreeBase<pcl::outofcore::OutofcoreOctreeDiskContainer<PointE57>, PointE57>;
		OCT::Ptr oct;

		static void ReadScan(const e57::ImageFile& imf, const e57::VectorNode& data3D, int64_t scanID, pcl::PointCloud<PointE57>& scanCloud);

	public:
		E57Converter(const boost::filesystem::path& octPath);
		E57Converter(const boost::filesystem::path& filePath, const boost::filesystem::path& octPath, const Eigen::Vector3d& min, const Eigen::Vector3d& max, const double resolution_arg, const double sample_percent_arg, const std::string& coord_sys);
		void BuildLOD(const double sample_percent_arg);
		void DownSampling(const double voxelUnit, pcl::PointCloud<PointE57>& out);
		static void PrintE57Format(const boost::filesystem::path& filePath);
	};
}