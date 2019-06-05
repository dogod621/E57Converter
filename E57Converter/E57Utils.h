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
	enum CoodSys : unsigned int
	{
		CoodSys_UNKNOWN = 0,
		XYZ = 1,
		RAE = 2
	};

	enum ElevationMode : unsigned int
	{
		ElevationMode_UNKNOWN = 0,

		N = 1, // Elevation is the angle between measure vector and north pole
		S = 2, // Elevation is the angle between measure vector and south pole
		E = 3, // Elevation is the angle between measure vector and plane of the ecliptic
	};

	enum AzimuthMode : unsigned int
	{
		AzimuthMode_UNKNOWN = 0,

		X_Y = 1, // Azimuth is the angle between measure vector(project on X-Y plane) and X axis, and Y axis is 90 degree
		X_Z = 2, // Azimuth is the angle between measure vector(project on X-Z plane) and X axis, and Z axis is 90 degree

		Y_X = 3, // Azimuth is the angle between measure vector(project on Y-X plane) and Y axis, and X axis is 90 degree
		Y_Z = 4, // Azimuth is the angle between measure vector(project on Y-Z plane) and Y axis, and Z axis is 90 degree

		Z_X = 5, // Azimuth is the angle between measure vector(project on Z-X plane) and Z axis, and X axis is 90 degree
		Z_Y = 6, // Azimuth is the angle between measure vector(project on Z-Y plane) and Z axis, and Y axis is 90 degree
	};

	enum RAEMode : unsigned int
	{
		RAEMode_UNKNOWN = 0,

		//
		N_X_Y = ((ElevationMode::N << 16) | AzimuthMode::X_Y),
		N_X_Z = ((ElevationMode::N << 16) | AzimuthMode::X_Z),

		N_Y_X = ((ElevationMode::N << 16) | AzimuthMode::Y_X),
		N_Y_Z = ((ElevationMode::N << 16) | AzimuthMode::Y_Z),

		N_Z_X = ((ElevationMode::N << 16) | AzimuthMode::Z_X),
		N_Z_Y = ((ElevationMode::N << 16) | AzimuthMode::Z_Y),

		//
		S_X_Y = ((ElevationMode::S << 16) | AzimuthMode::X_Y),
		S_X_Z = ((ElevationMode::S << 16) | AzimuthMode::X_Z),

		S_Y_X = ((ElevationMode::S << 16) | AzimuthMode::Y_X),
		S_Y_Z = ((ElevationMode::S << 16) | AzimuthMode::Y_Z),

		S_Z_X = ((ElevationMode::S << 16) | AzimuthMode::Z_X),
		S_Z_Y = ((ElevationMode::S << 16) | AzimuthMode::Z_Y),

		//
		E_X_Y = ((ElevationMode::E << 16) | AzimuthMode::X_Y),
		E_X_Z = ((ElevationMode::E << 16) | AzimuthMode::X_Z),

		E_Y_X = ((ElevationMode::E << 16) | AzimuthMode::Y_X),
		E_Y_Z = ((ElevationMode::E << 16) | AzimuthMode::Y_Z),

		E_Z_X = ((ElevationMode::E << 16) | AzimuthMode::Z_X),
		E_Z_Y = ((ElevationMode::E << 16) | AzimuthMode::Z_Y),
	};

	std::string CoodSysToStr(CoodSys type);
	std::string RAEModeToStr(RAEMode type);

	CoodSys StrToCoodSys(const std::string& str);
	RAEMode StrToRAEMode(const std::string& str);

	Eigen::Vector3d RAEToXYZ(RAEMode type, const Eigen::Vector3d& p);
	Eigen::Vector3d XYZToRAE(RAEMode type, const Eigen::Vector3d& p);
	Eigen::Vector2d RAEToUV(RAEMode type, const Eigen::Vector3d& p);

	class Scan
	{
	public:
		CoodSys coodSys;
		RAEMode raeMode; // only avalible when coodSys is RAE
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
			: coodSys(CoodSys::CoodSys_UNKNOWN), raeMode(RAEMode::RAEMode_UNKNOWN),
			transform(Eigen::Matrix4d::Identity()),
			hasPointXYZ(false), hasPointRGB(false), hasPointI(false), ID(0), numPoints(0) {}

		void Load(const e57::ImageFile& imf, const e57::VectorNode& data3D, int64_t scanID);
		void ToPointCloud(pcl::PointCloud<PointE57>& scanCloud);
	};
}