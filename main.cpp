#include <iostream>
#include <string>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include "E57Converter.h"

std::string to_upper(const std::string& s)
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

enum FileType
{
	UNKNOWN = 0,
	E57 = 1, // e57 file
	PCD = 2, // pcl file
	OCT = 3, // pcl out of core octree
	PLY = 4
};

bool IsE57(boost::filesystem::path filePath)
{
	if (filePath.string().back() != '/' && filePath.string().back() != '\\')
	{
		if (to_upper(filePath.extension().string()) == ".E57")
			return true;
	}
	return false;
}

bool IsPCD(boost::filesystem::path filePath)
{
	if (filePath.string().back() != '/' && filePath.string().back() != '\\')
	{
		if (to_upper(filePath.extension().string()) == ".PCD")
			return true;
	}
	return false;
}

bool IsOCT(boost::filesystem::path filePath)
{
	if (filePath.string().back() == '/' || filePath.string().back() == '\\')
		return true;
	return false;
}


bool IsPLY(boost::filesystem::path filePath)
{
	if (filePath.string().back() != '/' && filePath.string().back() != '\\')
	{
		if (to_upper(filePath.extension().string()) == ".PLY")
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
	else return FileType::UNKNOWN;
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

//

#define CMD_SPACE 20
#define PARMS_SPACE 40
#define PRINT_HELP(cmd, parms, info) std::cout << std::right << setw (CMD_SPACE) << cmd << std::left << setw(PARMS_SPACE) << parms << std::endl << std::left << setw (CMD_SPACE) << "" << "\t" << info << std::endl<< std::endl;

void
print_help(int, char **argv)
{
	std::cout << "Options:" << std::endl;

	PRINT_HELP("-convert"		, "", "Command that convert inputFile(from -src) to to outputFile(from -dst)");
	PRINT_HELP("-printE57Format", "", "Command that print formate of e57File(from -src)");
	PRINT_HELP("-buildLOD"		, "", "Command that buildLOD for OutOfCoreOctreeFile(from -src)");
	PRINT_HELP("-h"				, "", "Command that Print help");

	std::cout << "Parmameters - convert:" << std::endl;
	
	PRINT_HELP("-src"			, "<sting>"					, "Input pointCloud file");
	PRINT_HELP("-dst"			, "<sting>"					, "Output pointCloud file");
	PRINT_HELP("-res"			, "<float>"					, "Gird size for converting inputFile(from -src) to OutOfCoreOctreeFile(from -dst)");
	PRINT_HELP("-min"			, "<sting of tree floats>"	, "AABB min corner for converting inputFile(from -src) to OutOfCoreOctreeFile(from -dst). For example: -min \"-100 -100 -100\"");
	PRINT_HELP("-max"			, "<sting of tree floats>"	, "AABB max corner for converting inputFile(from -src) to OutOfCoreOctreeFile(from -dst). For example: -max \"100 100 100");
	PRINT_HELP("-samplePercent"	, "<float>"					, "Sample percent for build OutOfCoreOctree(from -src) LOD");

	PRINT_HELP("-voxelUnit"		, "<float>"					, "Gird size for converting OutOfCoreOctree(from -src) to pcdFile(from -dst)");
	
	PRINT_HELP("-rgb"			, ""						, "Output rgb for converting pcdFile(from -src) to plyFile(from -dst)");
	PRINT_HELP("-binary"		, ""						, "Output binary for converting pcdFile(from -src) to plyFile(from -dst)");
	PRINT_HELP("-camera"		, ""						, "Output camera for converting pcdFile(from -src) to plyFile(from -dst)");

	std::cout << "Parmameters - buildLOD:" << std::endl;

	PRINT_HELP("-samplePercent", "<float>", "Sample percent for build OutOfCoreOctree(from -src) LOD");
}


int main(int argc, char** argv)
{
	if (argc > 1)
	{
		if (pcl::console::find_switch(argc, argv, "-h"))
			print_help(argc, argv);

		else if (pcl::console::find_switch(argc, argv, "-convert"))
		{
			std::cout << "Options - convert" << std::endl;

			std::string _srcFilePath = "";
			std::string _dstFilePath = "";
			pcl::console::parse_argument(argc, argv, "-src", _srcFilePath);
			pcl::console::parse_argument(argc, argv, "-dst", _dstFilePath);

			boost::filesystem::path srcFilePath(_srcFilePath);
			boost::filesystem::path dstFilePath(_dstFilePath);
			FileType srcFileType = GetFileType(srcFilePath);
			FileType dstFileType = GetFileType(dstFilePath);

			std::cout << "Parmameters - src: " << srcFilePath << std::endl;
			std::cout << "Parmameters - dst: " << dstFilePath << std::endl;

			switch (srcFileType)
			{
			case FileType::PLY:
				switch (dstFileType)
				{
				case FileType::PCD:
				case FileType::E57:
				case FileType::OCT:
					std::cout << "Not implement." << std::endl;
					break;

				default:break;
				}
				break;

			case FileType::E57:
				switch (dstFileType)
				{
				case FileType::PLY:
					std::cout << "Cant direct convert to PLY, please convert to PCD before convert to PLY." << std::endl;
					break;

				case FileType::PCD:
					std::cout << "Cant direct convert to PCD, please convert to OutOfCoreOctree before convert to PCD." << std::endl;
					break;

				case FileType::OCT:
				{
					double res = 4;
					pcl::console::parse_argument(argc, argv, "-res", res);

					std::string _minStr = "-100 -100 -100";
					std::string _maxStr = "100 100 100";
					pcl::console::parse_argument(argc, argv, "-min", _minStr);
					pcl::console::parse_argument(argc, argv, "-max", _maxStr);
					std::stringstream minStr(_minStr);
					std::stringstream maxStr(_maxStr);
					double minX, minY, minZ, maxX, maxY, maxZ;
					minStr >> minX;
					minStr >> minY;
					minStr >> minZ;

					maxStr >> maxX;
					maxStr >> maxY;
					maxStr >> maxZ;

					Eigen::Vector3d min(minX, minY, minZ);
					Eigen::Vector3d max(maxX, maxY, maxZ);
					std::cout << "Parmameters - res: " << res << std::endl;
					std::cout << "Parmameters - min: " << min << std::endl;
					std::cout << "Parmameters - max: " << max << std::endl;

					double samplePercent = 0.125;
					pcl::console::parse_argument(argc, argv, "-samplePercent", samplePercent);
					std::cout << "Parmameters - samplePercent: " << samplePercent << std::endl;

					std::shared_ptr < pcl::E57Converter > e57Converter = std::shared_ptr < pcl::E57Converter >(new pcl::E57Converter(srcFilePath, dstFilePath, min, max, res, samplePercent, "ECEF"));
				}
				break;

				default:break;
				}
				break;

			case FileType::PCD:
				switch (dstFileType)
				{
				case FileType::PLY:
				{
					bool rgb = pcl::console::find_switch(argc, argv, "-rgb");
					bool binary = pcl::console::find_switch(argc, argv, "-binary");
					bool camera = pcl::console::find_switch(argc, argv, "-camera");

					std::cout << "Parmameters - rgb: " << rgb << std::endl;
					std::cout << "Parmameters - binary: " << binary << std::endl;
					std::cout << "Parmameters - camera: " << camera << std::endl;

					if (rgb)
					{
						pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
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
				break;

				case FileType::E57:
				case FileType::OCT:
					std::cout << "Not implement." << std::endl;
					break;

				default:break;
				}
				break;

			case FileType::OCT:
				switch (dstFileType)
				{
				case FileType::PLY:
				case FileType::E57:
					std::cout << "Not implement." << std::endl;
					break;

				case FileType::PCD:
				{
					double voxelUnit = 0.1;
					pcl::console::parse_argument(argc, argv, "-voxelUnit", voxelUnit);

					pcl::PointCloud<PointE57>::Ptr cloud(new pcl::PointCloud<PointE57>);

					std::cout << "Parmameters - voxelUnit: " << voxelUnit << std::endl;

					std::shared_ptr < pcl::E57Converter > e57Converter = std::shared_ptr < pcl::E57Converter >(new pcl::E57Converter(srcFilePath));
					e57Converter->DownSampling(voxelUnit, *cloud);
					pcl::io::savePCDFile(dstFilePath.string(), *cloud, true);
				}
				break;

				default:break;
				}
				break;

			default:break;
			}
		}

		else if (pcl::console::find_switch(argc, argv, "-buildLOD"))
		{
			std::cout << "Options - buildLOD" << std::endl;

			std::string _srcFilePath = "";
			pcl::console::parse_argument(argc, argv, "-src", _srcFilePath);
			boost::filesystem::path srcFilePath(_srcFilePath);
			FileType srcFileType = GetFileType(srcFilePath);
			std::cout << "Parmameters - src: " << srcFilePath << std::endl;

			if (srcFileType != FileType::OCT)
			{
				std::cout << "srcFileType is not OutOfCoreOctree" << std::endl;
				exit(EXIT_FAILURE);
			}

			double samplePercent = 0.125;
			pcl::console::parse_argument(argc, argv, "-samplePercent", samplePercent);

			std::cout << "Parmameters - samplePercent: " << samplePercent << std::endl;

			std::shared_ptr < pcl::E57Converter > e57Converter = std::shared_ptr < pcl::E57Converter >(new pcl::E57Converter(srcFilePath));
			e57Converter->BuildLOD(samplePercent);

		}

		else if (pcl::console::find_switch(argc, argv, "-printE57Format"))
		{
			std::cout << "Options - printE57Format" << std::endl;

			std::string _srcFilePath = "";
			pcl::console::parse_argument(argc, argv, "-src", _srcFilePath);
			boost::filesystem::path srcFilePath(_srcFilePath);
			FileType srcFileType = GetFileType(srcFilePath);
			std::cout << "Parmameters - src: " << srcFilePath << std::endl;

			if (srcFileType != FileType::E57)
			{
				std::cout << "srcFileType is not E57" << std::endl;
				exit(EXIT_FAILURE);
			}

			pcl::E57Converter::PrintE57Format(srcFilePath);

		}
	}
	else
		print_help(argc, argv);

	return EXIT_SUCCESS;
}