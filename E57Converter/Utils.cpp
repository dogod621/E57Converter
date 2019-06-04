#include <iostream>
#include <iomanip>
#include <string>

#include "E57Utils.h"
#include "E57Converter.h"
#include "Utils.h"

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

//
void Start(int argc, char **argv)
{
	if (argc > 1)
	{
		if (pcl::console::find_switch(argc, argv, "-h"))
			PrintHelp(argc, argv);

		else if (pcl::console::find_switch(argc, argv, "-convert"))
			Convert(argc, argv);

		else if (pcl::console::find_switch(argc, argv, "-saveScanImage"))
			SaveScanImage(argc, argv);

		else if (pcl::console::find_switch(argc, argv, "-buildLOD"))
			BuildLOD(argc, argv);

		else if (pcl::console::find_switch(argc, argv, "-printE57Format"))
			PrintE57Format(argc, argv);
	}
	else
		PrintHelp(argc, argv);
}

//
#define CMD_SPACE 15
#define PRINT_HELP(prefix, cmd, parms, info) std::cout << prefix << std::left << "-" << std::setw (CMD_SPACE) << cmd \
<< std::left << "[" << parms << "] : " << info << std::endl << std::endl;

void PrintHelp(int argc, char **argv)
{
	std::cout << "PrintHelp:" << std::endl << std::endl;

	std::cout << "Main Functions:===========================================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "convert", "", "Command that that convert inputFile to to outputFile.");
	}

	std::cout << "Parmameters of -convert -src \"*.e57\"  -dst \"*/\":==========================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "src", "sting \"\"", "Input e57 pointCloud file.");
		PRINT_HELP("\t", "dst", "sting \"\"", "Output OutOfCoreOctree pointCloud file.");
		PRINT_HELP("\t", "res", "float 4", "Gird unit size of OutOfCoreOctree in meters.");
		PRINT_HELP("\t", "min", "XYZ_string \"-100 -100 -100\"", "Min AABB corner of OutOfCoreOctree in meters. For example: -min \"-100 -100 -100\".");
		PRINT_HELP("\t", "max", "XYZ_string \"100 100 100\"", "Max AABB corner of OutOfCoreOctree in meters. For example: -max \"100 100 100\".");
		PRINT_HELP("\t", "samplePercent", "float 0.125", "Sample percent for building OutOfCoreOctree LOD.");
	}
	
	std::cout << "Parmameters of -convert -src \"*/\"  -dst \"*.pcd\":==========================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "src", "sting \"\"", "Input OutOfCoreOctree pointCloud file.");
		PRINT_HELP("\t", "dst", "sting \"\"", "Output pcd pointCloud file.");
		PRINT_HELP("\t", "voxelUnit 0.01", "float", "Gird voxel size in meters.");
		PRINT_HELP("\t", "searchRadiusNumVoxels 8", "int", "Gird voxel size in meters.");
		PRINT_HELP("\t", "minRGB 5", "int", "Gird voxel size in meters.");
	}

	std::cout << "Parmameters of -convert -src \"*.pcd\"  -dst \"*.ply\":=======================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "binary", "", "Output as binary.");
		PRINT_HELP("\t", "normal", "", "Output normal.");
		PRINT_HELP("\t", "rgb", "", "Output rgb.");
		PRINT_HELP("\t", "camera", "", "Output camera.");
	}

	std::cout << "Help Functions:===========================================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "saveScanImage", "", "Command that save scan images of imported e57 files to output folder.");
		PRINT_HELP("\t", "printE57Format", "", "Command that print formate of e57 file.");
		PRINT_HELP("\t", "buildLOD", "", "Command that buildLOD for OutOfCoreOctree.");
		PRINT_HELP("\t", "h", "", "Command that Print help.");
	}

	std::cout << "Parmameters of -saveScanImage:============================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "src", "sting \"\"", "Input OutOfCoreOctree file.");
		PRINT_HELP("\t", "dst", "sting \"\"", "Output folder.");
	}

	std::cout << "Parmameters of -printE57Format:===========================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "src", "sting \"\"", "Input e57 file.");
	}

	std::cout << "Parmameters of -buildLOD:=================================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "src", "sting \"\"", "Input OutOfCoreOctree file.");
		PRINT_HELP("\t", "samplePercent", "float  0.125", "Sample percent for building OutOfCoreOctree LOD.");
	}
	std::cout << "==========================================================================================================================================================" << std::endl << std::endl;
}

void Convert(int argc, char **argv)
{
	std::cout << "Convert:" << std::endl;

	std::string _srcFilePath = "";
	std::string _dstFilePath = "";
	pcl::console::parse_argument(argc, argv, "-src", _srcFilePath);
	pcl::console::parse_argument(argc, argv, "-dst", _dstFilePath);

	boost::filesystem::path srcFilePath(_srcFilePath);
	boost::filesystem::path dstFilePath(_dstFilePath);
	FileType srcFileType = GetFileType(srcFilePath);
	FileType dstFileType = GetFileType(dstFilePath);

	std::cout << "Parmameters -src: " << srcFilePath << std::endl;
	std::cout << "Parmameters -dst: " << dstFilePath << std::endl;

	switch (GetConvertType(srcFileType, dstFileType)) 
	{
	case ConvertType::E57_E57: Convert_E57_E57(srcFilePath, dstFilePath, argc, argv); break;
	case ConvertType::E57_PCD: Convert_E57_PCD(srcFilePath, dstFilePath, argc, argv); break;
	case ConvertType::E57_OCT: Convert_E57_OCT(srcFilePath, dstFilePath, argc, argv); break;
	case ConvertType::E57_PLY: Convert_E57_PLY(srcFilePath, dstFilePath, argc, argv); break;

	case ConvertType::PCD_E57: Convert_PCD_E57(srcFilePath, dstFilePath, argc, argv); break;
	case ConvertType::PCD_PCD: Convert_PCD_PCD(srcFilePath, dstFilePath, argc, argv); break;
	case ConvertType::PCD_OCT: Convert_PCD_OCT(srcFilePath, dstFilePath, argc, argv); break;
	case ConvertType::PCD_PLY: Convert_PCD_PLY(srcFilePath, dstFilePath, argc, argv); break;

	case ConvertType::OCT_E57: Convert_OCT_E57(srcFilePath, dstFilePath, argc, argv); break;
	case ConvertType::OCT_PCD: Convert_OCT_PCD(srcFilePath, dstFilePath, argc, argv); break;
	case ConvertType::OCT_OCT: Convert_OCT_OCT(srcFilePath, dstFilePath, argc, argv); break;
	case ConvertType::OCT_PLY: Convert_OCT_PLY(srcFilePath, dstFilePath, argc, argv); break;

	case ConvertType::PLY_E57: Convert_PLY_E57(srcFilePath, dstFilePath, argc, argv); break;
	case ConvertType::PLY_PCD: Convert_PLY_PCD(srcFilePath, dstFilePath, argc, argv); break;
	case ConvertType::PLY_OCT: Convert_PLY_OCT(srcFilePath, dstFilePath, argc, argv); break;
	case ConvertType::PLY_PLY: Convert_PLY_PLY(srcFilePath, dstFilePath, argc, argv); break;

	default:break;
	}
}

void SaveScanImage(int argc, char **argv)
{
	std::cout << "SaveScanImage:" << std::endl;

	std::string _srcFilePath = "";
	std::string _dstFilePath = "";
	pcl::console::parse_argument(argc, argv, "-src", _srcFilePath);
	pcl::console::parse_argument(argc, argv, "-dst", _dstFilePath);

	boost::filesystem::path srcFilePath(_srcFilePath);
	boost::filesystem::path dstFilePath(_dstFilePath);
	FileType srcFileType = GetFileType(srcFilePath);

	std::cout << "Parmameters -src: " << srcFilePath << std::endl;
	std::cout << "Parmameters -dst: " << dstFilePath << std::endl;

	if (srcFileType != FileType::OCT)
	{
		std::cout << "srcFileType is not OutOfCoreOctree." << std::endl;
		exit(EXIT_FAILURE);
	}

	if (!IsDir(dstFilePath))
	{
		std::cout << "dstFilePath is not a directory." << std::endl;
		exit(EXIT_FAILURE);
	}

	std::shared_ptr < e57::Converter > e57Converter = std::shared_ptr < e57::Converter >(new e57::Converter(srcFilePath));
	e57Converter->SaveScanImages(dstFilePath);
}

void PrintE57Format(int argc, char **argv)
{
	std::cout << "PrintE57Format:" << std::endl;

	std::string _srcFilePath = "";
	pcl::console::parse_argument(argc, argv, "-src", _srcFilePath);
	boost::filesystem::path srcFilePath(_srcFilePath);
	FileType srcFileType = GetFileType(srcFilePath);

	std::cout << "Parmameters -src: " << srcFilePath << std::endl;

	if (srcFileType != FileType::E57)
	{
		std::cout << "srcFileType is not e57." << std::endl;
		exit(EXIT_FAILURE);
	}

	e57::PrintFormat(srcFilePath);
}

void BuildLOD(int argc, char **argv)
{
	std::cout << "BuildLOD:" << std::endl;

	std::string _srcFilePath = "";
	pcl::console::parse_argument(argc, argv, "-src", _srcFilePath);
	boost::filesystem::path srcFilePath(_srcFilePath);
	FileType srcFileType = GetFileType(srcFilePath);
	std::cout << "Parmameters -src: " << srcFilePath << std::endl;

	if (srcFileType != FileType::OCT)
	{
		std::cout << "srcFileType is not OutOfCoreOctree." << std::endl;
		exit(EXIT_FAILURE);
	}

	double samplePercent = 0.125;
	pcl::console::parse_argument(argc, argv, "-samplePercent", samplePercent);

	std::cout << "Parmameters -samplePercent: " << samplePercent << std::endl;

	std::shared_ptr < e57::Converter > e57Converter = std::shared_ptr < e57::Converter >(new e57::Converter(srcFilePath));
	e57Converter->BuildLOD(samplePercent);
}

//
void Convert_E57_E57(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	std::cerr << "Not implement." << std::endl;
	exit(EXIT_FAILURE);
}

void Convert_E57_PCD(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	std::cerr << "Not implement." << std::endl;
	exit(EXIT_FAILURE);
}

void Convert_E57_OCT(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	double res = 4;
	pcl::console::parse_argument(argc, argv, "-res", res);
	std::cout << "Parmameters -res: " << res << std::endl;

	std::string _minStr = "-100 -100 -100";
	std::string _maxStr = "100 100 100";
	pcl::console::parse_argument(argc, argv, "-min", _minStr);
	pcl::console::parse_argument(argc, argv, "-max", _maxStr);
	std::stringstream minStr(_minStr);
	std::stringstream maxStr(_maxStr);
	double minX, minY, minZ, maxX, maxY, maxZ;
	minStr >> minX >> minY >> minZ;
	maxStr >> maxX >> maxY >> maxZ;
	Eigen::Vector3d min(minX, minY, minZ);
	Eigen::Vector3d max(maxX, maxY, maxZ);
	std::cout << "Parmameters -min: " << min << std::endl;
	std::cout << "Parmameters -max: " << max << std::endl;

	double samplePercent = 0.125;
	pcl::console::parse_argument(argc, argv, "-samplePercent", samplePercent);
	std::cout << "Parmameters -samplePercent: " << samplePercent << std::endl;

	std::shared_ptr < e57::Converter > e57Converter = std::shared_ptr < e57::Converter >(new e57::Converter(dstFilePath, min, max, res, "ECEF"));
	e57Converter->LoadE57(srcFilePath, samplePercent);
}

void Convert_E57_PLY(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	std::cerr << "Not implement." << std::endl;
	exit(EXIT_FAILURE);
}

//
void Convert_PCD_E57(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	std::cerr << "Not implement." << std::endl;
	exit(EXIT_FAILURE);
}

void Convert_PCD_PCD(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	std::cerr << "Not implement." << std::endl;
	exit(EXIT_FAILURE);
}

void Convert_PCD_OCT(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	std::cerr << "Not implement." << std::endl;
	exit(EXIT_FAILURE);
}

void Convert_PCD_PLY(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	bool binary = pcl::console::find_switch(argc, argv, "-binary");
	bool normal = pcl::console::find_switch(argc, argv, "-normal");
	bool rgb = pcl::console::find_switch(argc, argv, "-rgb");
	bool camera = pcl::console::find_switch(argc, argv, "-camera");

	std::cout << "Parmameters -normal: " << normal << std::endl;
	std::cout << "Parmameters -rgb: " << rgb << std::endl;
	std::cout << "Parmameters -binary: " << binary << std::endl;
	std::cout << "Parmameters -camera: " << camera << std::endl;

	if (normal)
	{
		if (rgb)
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			pcl::io::loadPCDFile(srcFilePath.string(), *cloud);
			pcl::PLYWriter writer;
			writer.write(dstFilePath.string(), *cloud, binary, camera);
		}
		else
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
			pcl::io::loadPCDFile(srcFilePath.string(), *cloud);
			pcl::PLYWriter writer;
			writer.write(dstFilePath.string(), *cloud, binary, camera);
		}
	}
	else
	{
		if (rgb)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::io::loadPCDFile(srcFilePath.string(), *cloud);
			pcl::PLYWriter writer;
			writer.write(dstFilePath.string(), *cloud, binary, camera);
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::io::loadPCDFile(srcFilePath.string(), *cloud);
			pcl::PLYWriter writer;
			writer.write(dstFilePath.string(), *cloud, binary, camera);
		}
	}
}

//
void Convert_OCT_E57(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	std::cerr << "Not implement." << std::endl;
	exit(EXIT_FAILURE);
}

void Convert_OCT_PCD(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	double voxelUnit = 0.01; // 1cm for default
	unsigned int searchRadiusNumVoxels = 8; // searchRadius 8cm for default
	unsigned int minRGB = 5;

	pcl::console::parse_argument(argc, argv, "-voxelUnit", voxelUnit);
	pcl::console::parse_argument(argc, argv, "-searchRadiusNumVoxels", searchRadiusNumVoxels);
	pcl::console::parse_argument(argc, argv, "-minRGB", minRGB);

	std::cout << "Parmameters -voxelUnit: " << voxelUnit << std::endl;
	std::cout << "Parmameters -searchRadiusNumVoxels: " << searchRadiusNumVoxels << std::endl;
	std::cout << "Parmameters -minRGB: " << minRGB << std::endl;

	pcl::PointCloud<PointPCD>::Ptr cloud(new pcl::PointCloud<PointPCD>);
	std::shared_ptr < e57::Converter > e57Converter = std::shared_ptr < e57::Converter >(new e57::Converter(srcFilePath));
	e57Converter->ExportToPCD(voxelUnit, searchRadiusNumVoxels, minRGB, *cloud);
	pcl::io::savePCDFile(dstFilePath.string(), *cloud, true);
}

void Convert_OCT_OCT(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	std::cerr << "Not implement." << std::endl;
	exit(EXIT_FAILURE);
}

void Convert_OCT_PLY(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	std::cerr << "Not implement." << std::endl;
	exit(EXIT_FAILURE);
}

//
void Convert_PLY_E57(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	std::cerr << "Not implement." << std::endl;
	exit(EXIT_FAILURE);
}

void Convert_PLY_PCD(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	std::cerr << "Not implement." << std::endl;
	exit(EXIT_FAILURE);
}

void Convert_PLY_OCT(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	std::cerr << "Not implement." << std::endl;
	exit(EXIT_FAILURE);
}

void Convert_PLY_PLY(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv)
{
	std::cerr << "Not implement." << std::endl;
	exit(EXIT_FAILURE);
}

//

std::string ToUpper(const std::string& s)
{
	std::string rs = s;
	int shift = ((int)'A') - ((int)'a');
	for (auto& c : rs)
	{
		int ci = (int)c;
		if (ci <= ((int)'z') && ci >= ((int)'a'))
			c = (char)(ci + shift);
	}
	return rs;
}

ConvertType GetConvertType(FileType srcType, FileType dstType)
{
	return (ConvertType)((srcType << 16) | dstType);
}

bool IsDir(boost::filesystem::path filePath)
{
	if (filePath.string().back() == '/' || filePath.string().back() == '\\')
		return true;
	return false;
}

bool IsE57(boost::filesystem::path filePath)
{
	if (!IsDir(filePath))
	{
		if (ToUpper(filePath.extension().string()) == ".E57")
			return true;
	}
	return false;
}

bool IsPCD(boost::filesystem::path filePath)
{
	if (!IsDir(filePath))
	{
		if (ToUpper(filePath.extension().string()) == ".PCD")
			return true;
	}
	return false;
}

bool IsOCT(boost::filesystem::path filePath)
{
	if (IsDir(filePath))
		return true;
	return false;
}

bool IsPLY(boost::filesystem::path filePath)
{
	if (!IsDir(filePath))
	{
		if (ToUpper(filePath.extension().string()) == ".PLY")
			return true;
	}
	return false;
}

FileType GetFileType(boost::filesystem::path filePath)
{
	if (IsE57(filePath)) return FileType::E57;
	else if (IsPCD(filePath)) return FileType::PCD;
	else if (IsOCT(filePath)) return FileType::OCT;
	else if (IsPLY(filePath)) return FileType::PLY;
	else return FileType::FileType_UNKNOWN;
}

std::string FileTypeStr(FileType t)
{
	switch (t)
	{
	case FileType::E57: return "E57";
	case FileType::PCD: return "PCD";
	case FileType::OCT: return "OCT";
	case FileType::PLY: return "PLY";
	default: return "UNKNOWN";
	}
}