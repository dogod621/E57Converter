#pragma once

#include "Common.h"

//
enum FileType : unsigned int
{
	FileType_UNKNOWN = 0,

	E57 = 1, // e57 file
	PCD = 2, // pcl file
	OCT = 3, // pcl out of core octree
	PLY = 4
};

enum ConvertType : unsigned int
{
	ConvertType_UNKNOWN = 0,

	E57_E57 = ((FileType::E57 << 16) | FileType::E57),
	E57_PCD = ((FileType::E57 << 16) | FileType::PCD),
	E57_OCT = ((FileType::E57 << 16) | FileType::OCT),
	E57_PLY = ((FileType::E57 << 16) | FileType::PLY),

	PCD_E57 = ((FileType::PCD << 16) | FileType::E57),
	PCD_PCD = ((FileType::PCD << 16) | FileType::PCD),
	PCD_OCT = ((FileType::PCD << 16) | FileType::OCT),
	PCD_PLY = ((FileType::PCD << 16) | FileType::PLY),

	OCT_E57 = ((FileType::OCT << 16) | FileType::E57),
	OCT_PCD = ((FileType::OCT << 16) | FileType::PCD),
	OCT_OCT = ((FileType::OCT << 16) | FileType::OCT),
	OCT_PLY = ((FileType::OCT << 16) | FileType::PLY),

	PLY_E57 = ((FileType::PLY << 16) | FileType::E57),
	PLY_PCD = ((FileType::PLY << 16) | FileType::PCD),
	PLY_OCT = ((FileType::PLY << 16) | FileType::OCT),
	PLY_PLY = ((FileType::PLY << 16) | FileType::PLY),
};

ConvertType GetConvertType(FileType srcType, FileType dstType);
bool IsE57(boost::filesystem::path filePath);
bool IsPCD(boost::filesystem::path filePath);
bool IsOCT(boost::filesystem::path filePath);
bool IsPLY(boost::filesystem::path filePath);
FileType GetFileType(boost::filesystem::path filePath);
std::string FileTypeStr(FileType t);

//
void Start(int argc, char **argv);

//
void PrintHelp(int argc, char **argv);
void Convert(int argc, char **argv);
void ReconstructNDF(int argc, char **argv);
void LoadScanHDRI(int argc, char **argv);
void ReconstructScanImages(int argc, char **argv);
void PrintE57Format(int argc, char **argv);
void BuildLOD(int argc, char **argv);

//
void Convert_E57_E57(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);
void Convert_E57_PCD(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);
void Convert_E57_OCT(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);
void Convert_E57_PLY(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);

//
void Convert_PCD_E57(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);
void Convert_PCD_PCD(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);
void Convert_PCD_OCT(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);
void Convert_PCD_PLY(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);

//
void Convert_OCT_E57(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);
void Convert_OCT_PCD(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);
void Convert_OCT_OCT(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);
void Convert_OCT_PLY(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);

//
void Convert_PLY_E57(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);
void Convert_PLY_PCD(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);
void Convert_PLY_OCT(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);
void Convert_PLY_PLY(const boost::filesystem::path& srcFilePath, const boost::filesystem::path& dstFilePath, int argc, char** argv);
