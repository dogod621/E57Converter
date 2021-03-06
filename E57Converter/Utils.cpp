#include <iostream>
#include <iomanip>
#include <string>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "E57Utils.h"
#include "E57Converter.h"
#include "Utils.h"

//
void Start(int argc, char **argv)
{
	if (argc > 1)
	{
		if (pcl::console::find_switch(argc, argv, "-h"))
			PrintHelp(argc, argv);

		else if (pcl::console::find_switch(argc, argv, "-convert"))
			Convert(argc, argv);

		else if (pcl::console::find_switch(argc, argv, "-reconstructNDF"))
			ReconstructNDF(argc, argv);

		else if (pcl::console::find_switch(argc, argv, "-loadScanHDRI"))
			LoadScanHDRI(argc, argv);
		
		else if (pcl::console::find_switch(argc, argv, "-reconstructScanImages"))
			ReconstructScanImages(argc, argv);

		else if (pcl::console::find_switch(argc, argv, "-buildLOD"))
			BuildLOD(argc, argv);

		else if (pcl::console::find_switch(argc, argv, "-printE57Format"))
			PrintE57Format(argc, argv);
	}
	else
		PrintHelp(argc, argv);
}

//
#define CMD_SPACE 25
#define PRINT_HELP(prefix, cmd, parms, info) std::cout << prefix << std::left << "-" << std::setw (CMD_SPACE) << cmd \
<< std::left << "[" << parms << "] : " << info << std::endl << std::endl;

void PrintHelp(int argc, char **argv)
{
	std::cout << "PrintHelp:" << std::endl << std::endl;

	std::cout << "Main Functions:===========================================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t"	, "convert"					, ""								, "Command that that convert inputFile to to outputFile.");
		PRINT_HELP("\t"	, "reconstructNDF"			, ""								, "Command that that reconstruct micro-facet normal distribution for PCD file.");
		PRINT_HELP("\t"	, "loadScanHDRI"			, ""								, "Command that that load scanHDRI info into point cloud.");
	}

	std::cout << "Parmameters of -convert -src \"*.e57\"  -dst \"*/\":==========================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t"	, "src"						, "sting \"\""						, "Input e57 pointCloud file.");
		PRINT_HELP("\t"	, "dst"						, "sting \"\""						, "Output OutOfCoreOctree pointCloud file.");
		PRINT_HELP("\t"	, "res"						, "float 4"							, "Gird unit size of OutOfCoreOctree in meters.");
		PRINT_HELP("\t"	, "min"						, "XYZ_string \"-100 -100 -100\""	, "Min AABB corner of OutOfCoreOctree in meters. For example: -min \"-100 -100 -100\".");
		PRINT_HELP("\t"	, "max"						, "XYZ_string \"100 100 100\""		, "Max AABB corner of OutOfCoreOctree in meters. For example: -max \"100 100 100\".");
		PRINT_HELP("\t"	, "samplePercent"			, "float 0.125"						, "Sample percent for building OutOfCoreOctree LOD.");
		PRINT_HELP("\t"	, "minRGB"					, "int 6"							, "Mean a point will be kept only if one of R, G, B is larger than minRGB. This parameters is used to filter out the black noise which is generated by some scanner (such as BLK360).");
		PRINT_HELP("\t"	, "scanner"					, "sting \"UNKNOWN\""				, "(Optional, leave it keeping UNKNOWN if you are not goint to load HDRI or reconstruct scene albedo) Specify scanner type.");
	}
	
	std::cout << "Parmameters of -convert -src \"*/\"  -dst \"*.pcd\":==========================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t"	, "src"						, "sting \"\""						, "Input OutOfCoreOctree pointCloud file.");
		PRINT_HELP("\t"	, "dst"						, "sting \"\""						, "Output pcd pointCloud file.");
		PRINT_HELP("\t"	, "voxelUnit"				, "float 0.01"						, "Gird voxel size in meters.");
		PRINT_HELP("\t"	, "searchRadiusNumVoxels"	, "int 8"							, "Search radius(unit is voxel), this is used for surface/normal estimation, outlier removal and albedo reconstruction.");
		PRINT_HELP("\t"	, "meanK"					, "int -1"							, "(Optional, set to negative to close it)Parameter for StatisticalOutlierRemoval to remove outliers.");
		PRINT_HELP("\t"	, "polynomialOrder"			, "int -1"							, "(Optional, set to negative to close it)Parameter for MovingLeastSquares to esitmate surface. If closed, use NormalEstimation instead, or it will use MovingLeastSquares to filter and estimate normal of surface.");
		PRINT_HELP("\t"	, "reconstructAlbedo"		, ""								, "(Optional) Enable scene albedo reconstruction.");
		PRINT_HELP("\t"	, "reconstructNDF"			, ""								, "(Optional, if true, it will set reconstructAlbedo altomatically) Enable scene micro-facet normal distribution reconstruction.");
	}

	std::cout << "Parmameters of -convert -src \"*.pcd\"  -dst \"*.ply\":=======================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t"	, "binary"					, ""								, "Output as binary.");
		PRINT_HELP("\t"	, "normal"					, ""								, "Output normal.");
		PRINT_HELP("\t"	, "rgb"						, ""								, "Output rgb.");
		PRINT_HELP("\t"	, "camera"					, ""								, "Output camera.");
	}

	std::cout << "Parmameters of -reconstructNDF:===========================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t", "src", "sting \"\"", "Input OutOfCoreOctree file.");
		PRINT_HELP("\t", "pcd", "sting \"\"", "Input and output pcd file.");
		PRINT_HELP("\t"	, "voxelUnit"				, "float 0.01"						, "Gird voxel size in meters.");
		PRINT_HELP("\t"	, "searchRadiusNumVoxels"	, "int 8"							, "Search radius(unit is voxel), this is used for surface/normal estimation, outlier removal and albedo reconstruction.");
		
	}

	std::cout << "Parmameters of -loadScanHDRI:=============================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t"	, "src"						, "sting \"\""						, "Input OutOfCoreOctree file.");
		PRINT_HELP("\t"	, "data"					, "sting \"\""						, "Input scanned data file path.");
	}

	std::cout << "Help Functions:===========================================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t"	, "reconstructScanImages"	, ""								, "Command that reconstruct scan images of converted PCD.");
		PRINT_HELP("\t"	, "printE57Format"			, ""								, "Command that print formate of e57 file.");
		PRINT_HELP("\t"	, "buildLOD"				, ""								, "Command that buildLOD for OutOfCoreOctree.");
		PRINT_HELP("\t"	, "h"						, ""								, "Command that Print help.");
	}

	std::cout << "Parmameters of -reconstructScanImages:====================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t"	, "src"						, "sting \"\""						, "Input OutOfCoreOctree file.");
		PRINT_HELP("\t"	, "pcd"						, "sting \"\""						, "Input pcd file.");
		PRINT_HELP("\t"	, "dst"						, "sting \"\""						, "Output folder.");
		PRINT_HELP("\t"	, "coodSys"					, "sting \"XYZ\""					, "Specify scanner coordinate system. If is XYZ, means it is a camera. If is RAE, means it is a 360 camera.");
		PRINT_HELP("\t"	, "width"					, "int 1024"						, "Specify scanImage width.");
		PRINT_HELP("\t"	, "height"					, "int 512"							, "Specify scanImage height.");
		PRINT_HELP("\t"	, "raeMode"					, "sting \"E_X_Y\""					, "(Only used when -coodSys \"RAE\" ) Specify scanner RAE coordinate system mode.");
		PRINT_HELP("\t"	, "fovy"					, "float 60"						, "(Only used when -coodSys \"XYZ\" ) Specify scanner projection fovy in degrees.");
	}

	std::cout << "Parmameters of -printE57Format:===========================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t"	, "src"						, "sting \"\""						, "Input e57 file.");
	}

	std::cout << "Parmameters of -buildLOD:=================================================================================================================================" << std::endl << std::endl;
	{
		PRINT_HELP("\t"	, "src"						, "sting \"\""						, "Input OutOfCoreOctree file.");
		PRINT_HELP("\t"	, "samplePercent"			, "float 0.125"						, "Sample percent for building OutOfCoreOctree LOD.");
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

void ReconstructNDF(int argc, char **argv)
{
	std::cout << "ReconstructNDF:" << std::endl;

	std::string _srcFilePath = "";
	std::string _pcdFilePath = "";
	pcl::console::parse_argument(argc, argv, "-src", _srcFilePath);
	pcl::console::parse_argument(argc, argv, "-pcd", _pcdFilePath);

	boost::filesystem::path srcFilePath(_srcFilePath);
	boost::filesystem::path pcdFilePath(_pcdFilePath);

	FileType srcFileType = GetFileType(srcFilePath);
	FileType pcdFileType = GetFileType(pcdFilePath);

	std::cout << "Parmameters -src: " << srcFilePath << std::endl;
	std::cout << "Parmameters -pcd: " << pcdFilePath << std::endl;

	if (srcFileType != FileType::OCT)
	{
		std::cout << "srcFileType is not OutOfCoreOctree." << std::endl;
		exit(EXIT_FAILURE);
	}

	if (pcdFileType != FileType::PCD)
	{
		std::cout << "pcdFileType is not pcd." << std::endl;
		exit(EXIT_FAILURE);
	}

	std::shared_ptr < e57::Converter > e57Converter = std::shared_ptr < e57::Converter >(new e57::Converter(srcFilePath));
	pcl::PointCloud<PointPCD>::Ptr cloud(new pcl::PointCloud<PointPCD>);
	pcl::io::loadPCDFile(pcdFilePath.string(), *cloud);

	double voxelUnit = 0.01; // 1cm for default
	unsigned int searchRadiusNumVoxels = 8; // searchRadius 8cm for default
	pcl::console::parse_argument(argc, argv, "-voxelUnit", voxelUnit);
	pcl::console::parse_argument(argc, argv, "-searchRadiusNumVoxels", searchRadiusNumVoxels);
	std::cout << "Parmameters -voxelUnit: " << voxelUnit << std::endl;
	std::cout << "Parmameters -searchRadiusNumVoxels: " << searchRadiusNumVoxels << std::endl;

	float spatialImportance = 1.0f;
	float normalImportance = 1.0f;
	pcl::console::parse_argument(argc, argv, "-spatialImportance", spatialImportance);
	pcl::console::parse_argument(argc, argv, "-normalImportance", normalImportance);
	std::cout << "Parmameters -spatialImportance: " << spatialImportance << std::endl;
	std::cout << "Parmameters -normalImportance: " << normalImportance << std::endl;

	std::vector<pcl::PointCloud<PointNDF>::Ptr> NDFs;
	e57Converter->ExportToPCD_ReconstructNDF(voxelUnit, searchRadiusNumVoxels, spatialImportance, normalImportance, cloud, NDFs);
	pcl::io::savePCDFile(pcdFilePath.string(), *cloud, true);

	boost::filesystem::path dirFilePath = pcdFilePath.parent_path();
	boost::filesystem::path baseName = pcdFilePath.stem();
	for (std::size_t i = 0; i < NDFs.size(); ++i)
		pcl::io::savePCDFile((dirFilePath / boost::filesystem::path(baseName.string() + "_segment_"+ std::to_string(i) + "_NDF.pcd")).string(), *NDFs[i], true);
}

void LoadScanHDRI(int argc, char **argv)
{
	std::cout << "LoadScanHDRI:" << std::endl;

	std::string _srcFilePath = "";
	std::string _dataFilePath = "";
	pcl::console::parse_argument(argc, argv, "-src", _srcFilePath);
	pcl::console::parse_argument(argc, argv, "-data", _dataFilePath);

	boost::filesystem::path srcFilePath(_srcFilePath);
	boost::filesystem::path dataFilePath(_dataFilePath);

	FileType srcFileType = GetFileType(srcFilePath);

	std::cout << "Parmameters -src: " << srcFilePath << std::endl;
	std::cout << "Parmameters -data: " << dataFilePath << std::endl;

	if (srcFileType != FileType::OCT)
	{
		std::cout << "srcFileType is not OutOfCoreOctree." << std::endl;
		exit(EXIT_FAILURE);
	}

	std::shared_ptr < e57::Converter > e57Converter = std::shared_ptr < e57::Converter >(new e57::Converter(srcFilePath));
	e57Converter->LoadScanHDRI(dataFilePath);
}

void ReconstructScanImages(int argc, char **argv)
{
	std::cout << "ReconstructScanImages:" << std::endl;

	std::string _srcFilePath = "";
	std::string _pcdFilePath = "";
	std::string _dstFilePath = "";
	pcl::console::parse_argument(argc, argv, "-src", _srcFilePath);
	pcl::console::parse_argument(argc, argv, "-pcd", _pcdFilePath);
	pcl::console::parse_argument(argc, argv, "-dst", _dstFilePath);

	boost::filesystem::path srcFilePath(_srcFilePath);
	boost::filesystem::path pcdFilePath(_pcdFilePath);
	boost::filesystem::path dstFilePath(_dstFilePath);

	FileType srcFileType = GetFileType(srcFilePath);
	FileType pcdFileType = GetFileType(pcdFilePath);

	std::cout << "Parmameters -src: " << srcFilePath << std::endl;
	std::cout << "Parmameters -pcd: " << pcdFilePath << std::endl;
	std::cout << "Parmameters -dst: " << dstFilePath << std::endl;

	if (srcFileType != FileType::OCT)
	{
		std::cout << "srcFileType is not OutOfCoreOctree." << std::endl;
		exit(EXIT_FAILURE);
	}

	if (pcdFileType != FileType::PCD)
	{
		std::cout << "pcdFileType is not pcd." << std::endl;
		exit(EXIT_FAILURE);
	}

	if (!IsDir(dstFilePath))
	{
		std::cout << "dstFilePath is not a directory." << std::endl;
		exit(EXIT_FAILURE);
	}

	std::string coodSysStr = "XYZ";
	pcl::console::parse_argument(argc, argv, "-coodSys", coodSysStr);
	CoodSys coodSys = StrToCoodSys(coodSysStr);
	std::cout << "Parmameters -coodSys: " << CoodSysToStr(coodSys) << std::endl;

	int width = 1024;
	int height = 512;
	pcl::console::parse_argument(argc, argv, "-width", width);
	pcl::console::parse_argument(argc, argv, "-height", height);
	std::cout << "Parmameters -width: " << width << std::endl;
	std::cout << "Parmameters -height: " << height << std::endl;

	std::string raeModeStr = "E_X_Y";
	pcl::console::parse_argument(argc, argv, "-raeMode", raeModeStr);
	RAEMode raeMode = StrToRAEMode(raeModeStr);
	std::cout << "Parmameters -raeMode: " << RAEModeToStr(raeMode) << std::endl;

	float fovy = 60.f;
	pcl::console::parse_argument(argc, argv, "-fovy", fovy);
	std::cout << "Parmameters -fovy: " << fovy << std::endl;
	fovy = fovy * M_PI / 180.f;

	pcl::PointCloud<PointPCD>::Ptr cloud(new pcl::PointCloud<PointPCD>);
	pcl::io::loadPCDFile(pcdFilePath.string(), *cloud);
	std::shared_ptr < e57::Converter > e57Converter = std::shared_ptr < e57::Converter >(new e57::Converter(srcFilePath));
	e57Converter->ReconstructScanImages(*cloud, dstFilePath, coodSys, raeMode, fovy, width, height);
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
	unsigned int minRGB = 6;
	pcl::console::parse_argument(argc, argv, "-samplePercent", samplePercent);
	pcl::console::parse_argument(argc, argv, "-minRGB", minRGB);
	std::cout << "Parmameters -samplePercent: " << samplePercent << std::endl;
	std::cout << "Parmameters -minRGB: " << minRGB << std::endl;

	std::string scannerStr = "UNKNOWN";
	pcl::console::parse_argument(argc, argv, "-scanner", scannerStr);
	Scanner scanner = StrToScanner(scannerStr);
	std::cout << "Parmameters -scanner: " << ScannerToStr(scanner) << std::endl;

	std::shared_ptr < e57::Converter > e57Converter = std::shared_ptr < e57::Converter >(new e57::Converter(dstFilePath, min, max, res, "ECEF"));
	e57Converter->LoadE57(srcFilePath, samplePercent, minRGB, scanner);
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
	int meanK = -1;
	int polynomialOrder = -1;

	pcl::console::parse_argument(argc, argv, "-voxelUnit", voxelUnit);
	pcl::console::parse_argument(argc, argv, "-searchRadiusNumVoxels", searchRadiusNumVoxels);
	pcl::console::parse_argument(argc, argv, "-meanK", meanK);
	pcl::console::parse_argument(argc, argv, "-polynomialOrder", polynomialOrder);

	std::cout << "Parmameters -voxelUnit: " << voxelUnit << std::endl;
	std::cout << "Parmameters -searchRadiusNumVoxels: " << searchRadiusNumVoxels << std::endl;
	std::cout << "Parmameters -meanK: " << meanK << std::endl;
	std::cout << "Parmameters -polynomialOrder: " << polynomialOrder << std::endl;

	bool reconstructAlbedo = pcl::console::find_switch(argc, argv, "-reconstructAlbedo");
	bool reconstructNDF = pcl::console::find_switch(argc, argv, "-reconstructNDF");
	if (reconstructNDF)
		reconstructAlbedo = true;
	std::cout << "Parmameters -reconstructAlbedo: " << reconstructAlbedo << std::endl;
	std::cout << "Parmameters -reconstructNDF: " << reconstructNDF << std::endl;
	
	pcl::PointCloud<PointPCD>::Ptr cloud(new pcl::PointCloud<PointPCD>);
	std::vector<pcl::PointCloud<PointNDF>::Ptr> NDFs;
	std::shared_ptr < e57::Converter > e57Converter = std::shared_ptr < e57::Converter >(new e57::Converter(srcFilePath));
	e57Converter->ExportToPCD(voxelUnit, searchRadiusNumVoxels, meanK, polynomialOrder, reconstructAlbedo, reconstructNDF, cloud, NDFs);
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
ConvertType GetConvertType(FileType srcType, FileType dstType)
{
	return (ConvertType)((srcType << 16) | dstType);
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