#pragma once

#include <string>
#include <memory>

#include "PointType.h"

#include <boost/filesystem.hpp>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <E57Format.h>

std::string ToUpper(const std::string& s);
bool IsDir(boost::filesystem::path filePath);
int IsUnsignedInt(const std::string& s);

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

	//
	enum Scanner : unsigned int
	{
		Scaner_UNKNOWN = 0,

		BLK360 = 1,
	};

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

		N = 1, // Elevation is the angle between measure vector and north pole, north pole is cross of these two vectore on plane of the ecliptic: Azimuth at 0 degree, Azimuth at 90 degree
		S = 2, // Elevation is the angle between measure vector and south pole
		E = 3, // Elevation is the angle between measure vector and plane of the ecliptic, and positive side is toward north pole
	};

	enum AzimuthBase : unsigned int
	{
		AzimuthBase_UNKNOWN = 0,

		A0_X	= 1,
		A0_Y	= 2,
		A0_Z	= 3,

		A0_NX	= 4,
		A0_NY	= 5,
		A0_NZ	= 6,

		A90_X	= 7,
		A90_Y	= 8,
		A90_Z	= 9,

		A90_NX	= 10,
		A90_NY	= 11,
		A90_NZ	= 12,
	};

	enum AzimuthMode : unsigned int
	{
		AzimuthMode_UNKNOWN = 0,

		X_Y		= ((AzimuthBase::A0_X	<< 8) | AzimuthBase::A90_Y	), // Azimuth is the angle between measure vector(project on X-Y plane) and X axis, and Y axis is 90 degree
		X_Z		= ((AzimuthBase::A0_X	<< 8) | AzimuthBase::A90_Z	), // Azimuth is the angle between measure vector(project on X-Z plane) and X axis, and Z axis is 90 degree
		X_NY	= ((AzimuthBase::A0_X	<< 8) | AzimuthBase::A90_NY	), // Azimuth is the angle between measure vector(project on X-Z plane) and X axis, and negtive Y axis is 90 degree
		X_NZ	= ((AzimuthBase::A0_X	<< 8) | AzimuthBase::A90_NZ	), // Azimuth is the angle between measure vector(project on X-Z plane) and X axis, and negtive Z axis is 90 degree

		Y_X		= ((AzimuthBase::A0_Y	<< 8) | AzimuthBase::A90_X	), // Azimuth is the angle between measure vector(project on Y-X plane) and Y axis, and X axis is 90 degree
		Y_Z		= ((AzimuthBase::A0_Y	<< 8) | AzimuthBase::A90_Z	), // Azimuth is the angle between measure vector(project on Y-Z plane) and Y axis, and Z axis is 90 degree
		Y_NX	= ((AzimuthBase::A0_Y	<< 8) | AzimuthBase::A90_NX	), // Azimuth is the angle between measure vector(project on Y-X plane) and Y axis, and negtive X axis is 90 degree
		Y_NZ	= ((AzimuthBase::A0_Y	<< 8) | AzimuthBase::A90_NZ	), // Azimuth is the angle between measure vector(project on Y-Z plane) and Y axis, and negtive Z axis is 90 degree

		Z_X		= ((AzimuthBase::A0_Z	<< 8) | AzimuthBase::A90_X	), // Azimuth is the angle between measure vector(project on Z-X plane) and Z axis, and X axis is 90 degree
		Z_Y		= ((AzimuthBase::A0_Z	<< 8) | AzimuthBase::A90_Y	), // Azimuth is the angle between measure vector(project on Z-Y plane) and Z axis, and Y axis is 90 degree
		Z_NX	= ((AzimuthBase::A0_Z	<< 8) | AzimuthBase::A90_NX	), // Azimuth is the angle between measure vector(project on Z-X plane) and Z axis, and negtive X axis is 90 degree
		Z_NY	= ((AzimuthBase::A0_Z	<< 8) | AzimuthBase::A90_NY	), // Azimuth is the angle between measure vector(project on Z-Y plane) and Z axis, and negtive Y axis is 90 degree

		NX_Y	= ((AzimuthBase::A0_NX	<< 8) | AzimuthBase::A90_Y	), // Azimuth is the angle between measure vector(project on X-Y plane) and negtive X axis, and Y axis is 90 degree
		NX_Z	= ((AzimuthBase::A0_NX	<< 8) | AzimuthBase::A90_Z	), // Azimuth is the angle between measure vector(project on X-Z plane) and negtive X axis, and Z axis is 90 degree
		NX_NY	= ((AzimuthBase::A0_NX	<< 8) | AzimuthBase::A90_NY	), // Azimuth is the angle between measure vector(project on X-Z plane) and negtive X axis, and negtive Y axis is 90 degree
		NX_NZ	= ((AzimuthBase::A0_NX	<< 8) | AzimuthBase::A90_NZ	), // Azimuth is the angle between measure vector(project on X-Z plane) and negtive X axis, and negtive Z axis is 90 degree

		NY_X	= ((AzimuthBase::A0_NY	<< 8) | AzimuthBase::A90_X	), // Azimuth is the angle between measure vector(project on Y-X plane) and negtive Y axis, and X axis is 90 degree
		NY_Z	= ((AzimuthBase::A0_NY	<< 8) | AzimuthBase::A90_Z	), // Azimuth is the angle between measure vector(project on Y-Z plane) and negtive Y axis, and Z axis is 90 degree
		NY_NX	= ((AzimuthBase::A0_NY	<< 8) | AzimuthBase::A90_NX	), // Azimuth is the angle between measure vector(project on Y-X plane) and negtive Y axis, and negtive X axis is 90 degree
		NY_NZ	= ((AzimuthBase::A0_NY	<< 8) | AzimuthBase::A90_NZ	), // Azimuth is the angle between measure vector(project on Y-Z plane) and negtive Y axis, and negtive Z axis is 90 degree

		NZ_X	= ((AzimuthBase::A0_NZ	<< 8) | AzimuthBase::A90_X	), // Azimuth is the angle between measure vector(project on Z-X plane) and negtive Z axis, and X axis is 90 degree
		NZ_Y	= ((AzimuthBase::A0_NZ	<< 8) | AzimuthBase::A90_Y	), // Azimuth is the angle between measure vector(project on Z-Y plane) and negtive Z axis, and Y axis is 90 degree
		NZ_NX	= ((AzimuthBase::A0_NZ	<< 8) | AzimuthBase::A90_NX	), // Azimuth is the angle between measure vector(project on Z-X plane) and negtive Z axis, and negtive X axis is 90 degree
		NZ_NY	= ((AzimuthBase::A0_NZ	<< 8) | AzimuthBase::A90_NY	), // Azimuth is the angle between measure vector(project on Z-Y plane) and negtive Z axis, and negtive Y axis is 90 degree
	};

	enum RAEMode : unsigned int
	{
		RAEMode_UNKNOWN = 0,

		//
		N_X_Y	= ((ElevationMode::N << 16) | AzimuthMode::X_Y		),
		N_X_Z	= ((ElevationMode::N << 16) | AzimuthMode::X_Z		),
		N_X_NY	= ((ElevationMode::N << 16) | AzimuthMode::X_NY		),
		N_X_NZ	= ((ElevationMode::N << 16) | AzimuthMode::X_NZ		),
		N_Y_X	= ((ElevationMode::N << 16) | AzimuthMode::Y_X		),
		N_Y_Z	= ((ElevationMode::N << 16) | AzimuthMode::Y_Z		),
		N_Y_NX	= ((ElevationMode::N << 16) | AzimuthMode::Y_NX		),
		N_Y_NZ	= ((ElevationMode::N << 16) | AzimuthMode::Y_NZ		),
		N_Z_X	= ((ElevationMode::N << 16) | AzimuthMode::Z_X		),
		N_Z_Y	= ((ElevationMode::N << 16) | AzimuthMode::Z_Y		),
		N_Z_NX	= ((ElevationMode::N << 16) | AzimuthMode::Z_NX		),
		N_Z_NY	= ((ElevationMode::N << 16) | AzimuthMode::Z_NY		),
		N_NX_Y	= ((ElevationMode::N << 16) | AzimuthMode::NX_Y		),
		N_NX_Z	= ((ElevationMode::N << 16) | AzimuthMode::NX_Z		),
		N_NX_NY	= ((ElevationMode::N << 16) | AzimuthMode::NX_NY	),
		N_NX_NZ	= ((ElevationMode::N << 16) | AzimuthMode::NX_NZ	),
		N_NY_X	= ((ElevationMode::N << 16) | AzimuthMode::NY_X		),
		N_NY_Z	= ((ElevationMode::N << 16) | AzimuthMode::NY_Z		),
		N_NY_NX	= ((ElevationMode::N << 16) | AzimuthMode::NY_NX	),
		N_NY_NZ	= ((ElevationMode::N << 16) | AzimuthMode::NY_NZ	),
		N_NZ_X	= ((ElevationMode::N << 16) | AzimuthMode::NZ_X		),
		N_NZ_Y	= ((ElevationMode::N << 16) | AzimuthMode::NZ_Y		),
		N_NZ_NX	= ((ElevationMode::N << 16) | AzimuthMode::NZ_NX	),
		N_NZ_NY	= ((ElevationMode::N << 16) | AzimuthMode::NZ_NY	),

		//
		S_X_Y	= ((ElevationMode::S << 16) | AzimuthMode::X_Y		),
		S_X_Z	= ((ElevationMode::S << 16) | AzimuthMode::X_Z		),
		S_X_NY	= ((ElevationMode::S << 16) | AzimuthMode::X_NY		),
		S_X_NZ	= ((ElevationMode::S << 16) | AzimuthMode::X_NZ		),
		S_Y_X	= ((ElevationMode::S << 16) | AzimuthMode::Y_X		),
		S_Y_Z	= ((ElevationMode::S << 16) | AzimuthMode::Y_Z		),
		S_Y_NX	= ((ElevationMode::S << 16) | AzimuthMode::Y_NX		),
		S_Y_NZ	= ((ElevationMode::S << 16) | AzimuthMode::Y_NZ		),
		S_Z_X	= ((ElevationMode::S << 16) | AzimuthMode::Z_X		),
		S_Z_Y	= ((ElevationMode::S << 16) | AzimuthMode::Z_Y		),
		S_Z_NX	= ((ElevationMode::S << 16) | AzimuthMode::Z_NX		),
		S_Z_NY	= ((ElevationMode::S << 16) | AzimuthMode::Z_NY		),
		S_NX_Y	= ((ElevationMode::S << 16) | AzimuthMode::NX_Y		),
		S_NX_Z	= ((ElevationMode::S << 16) | AzimuthMode::NX_Z		),
		S_NX_NY	= ((ElevationMode::S << 16) | AzimuthMode::NX_NY	),
		S_NX_NZ	= ((ElevationMode::S << 16) | AzimuthMode::NX_NZ	),
		S_NY_X	= ((ElevationMode::S << 16) | AzimuthMode::NY_X		),
		S_NY_Z	= ((ElevationMode::S << 16) | AzimuthMode::NY_Z		),
		S_NY_NX	= ((ElevationMode::S << 16) | AzimuthMode::NY_NX	),
		S_NY_NZ	= ((ElevationMode::S << 16) | AzimuthMode::NY_NZ	),
		S_NZ_X	= ((ElevationMode::S << 16) | AzimuthMode::NZ_X		),
		S_NZ_Y	= ((ElevationMode::S << 16) | AzimuthMode::NZ_Y		),
		S_NZ_NX	= ((ElevationMode::S << 16) | AzimuthMode::NZ_NX	),
		S_NZ_NY	= ((ElevationMode::S << 16) | AzimuthMode::NZ_NY	),

		//
		E_X_Y	= ((ElevationMode::E << 16) | AzimuthMode::X_Y		),
		E_X_Z	= ((ElevationMode::E << 16) | AzimuthMode::X_Z		),
		E_X_NY	= ((ElevationMode::E << 16) | AzimuthMode::X_NY		),
		E_X_NZ	= ((ElevationMode::E << 16) | AzimuthMode::X_NZ		),
		E_Y_X	= ((ElevationMode::E << 16) | AzimuthMode::Y_X		),
		E_Y_Z	= ((ElevationMode::E << 16) | AzimuthMode::Y_Z		),
		E_Y_NX	= ((ElevationMode::E << 16) | AzimuthMode::Y_NX		),
		E_Y_NZ	= ((ElevationMode::E << 16) | AzimuthMode::Y_NZ		),
		E_Z_X	= ((ElevationMode::E << 16) | AzimuthMode::Z_X		),
		E_Z_Y	= ((ElevationMode::E << 16) | AzimuthMode::Z_Y		),
		E_Z_NX	= ((ElevationMode::E << 16) | AzimuthMode::Z_NX		),
		E_Z_NY	= ((ElevationMode::E << 16) | AzimuthMode::Z_NY		),
		E_NX_Y	= ((ElevationMode::E << 16) | AzimuthMode::NX_Y		),
		E_NX_Z	= ((ElevationMode::E << 16) | AzimuthMode::NX_Z		),
		E_NX_NY	= ((ElevationMode::E << 16) | AzimuthMode::NX_NY	),
		E_NX_NZ	= ((ElevationMode::E << 16) | AzimuthMode::NX_NZ	),
		E_NY_X	= ((ElevationMode::E << 16) | AzimuthMode::NY_X		),
		E_NY_Z	= ((ElevationMode::E << 16) | AzimuthMode::NY_Z		),
		E_NY_NX	= ((ElevationMode::E << 16) | AzimuthMode::NY_NX	),
		E_NY_NZ	= ((ElevationMode::E << 16) | AzimuthMode::NY_NZ	),
		E_NZ_X	= ((ElevationMode::E << 16) | AzimuthMode::NZ_X		),
		E_NZ_Y	= ((ElevationMode::E << 16) | AzimuthMode::NZ_Y		),
		E_NZ_NX	= ((ElevationMode::E << 16) | AzimuthMode::NZ_NX	),
		E_NZ_NY	= ((ElevationMode::E << 16) | AzimuthMode::NZ_NY	),
	};

	ElevationMode GetElevationMode(unsigned int type);
	AzimuthMode GetAzimuthMode(unsigned int type);
	Eigen::Vector3d GetAzimuth0DegreeVector(unsigned int type);
	Eigen::Vector3d GetAzimuth90DegreeVector(unsigned int type);

	std::string ScannerToStr(Scanner type);
	std::string CoodSysToStr(CoodSys type);
	std::string RAEModeToStr(RAEMode type);

	Scanner StrToScanner(const std::string& str);
	CoodSys StrToCoodSys(const std::string& str);
	RAEMode StrToRAEMode(const std::string& str);

	Eigen::Vector3d RAEToXYZ(RAEMode type, const Eigen::Vector3d& rae);
	Eigen::Vector3d XYZToRAE(RAEMode type, const Eigen::Vector3d& xyz);
	Eigen::Vector2d RAEToUV(RAEMode type, const Eigen::Vector3d& rae);

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