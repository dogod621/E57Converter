#include "Common.h"

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

bool IsDir(boost::filesystem::path filePath)
{
	if (filePath.string().back() == '/' || filePath.string().back() == '\\')
		return true;
	return false;
}

int IsUnsignedInt(const std::string& s)
{
	std::string rs;
	for (auto& c : s)
	{
		int ci = (int)c;
		if (ci != ((int)' '))
		{
			if ((ci <= ((int)'9')) && (ci >= ((int)'0')))
				rs.push_back(ci);
			else
				return -1;
		}
	}
	if (rs.size() > 0)
		return std::stoi(rs);
	return -1;
}

ElevationMode GetElevationMode(unsigned int type)
{
	unsigned int mask = 0xFFFF;
	mask = mask << 16;
	return ElevationMode((type & mask) >> 16);
}

AzimuthMode GetAzimuthMode(unsigned int type)
{
	unsigned int mask = 0xFFFF;
	return AzimuthMode(type & mask);
}

Eigen::Vector3d GetAzimuth0DegreeVector(unsigned int type)
{
	unsigned int mask = 0xFF;
	mask = mask << 8;
	switch (AzimuthBase((type & mask) >> 8))
	{
	case AzimuthBase::A0_X: return Eigen::Vector3d(1.0, 0.0, 0.0); break;
	case AzimuthBase::A0_Y: return Eigen::Vector3d(0.0, 1.0, 0.0); break;
	case AzimuthBase::A0_Z: return Eigen::Vector3d(0.0, 0.0, 1.0); break;
	case AzimuthBase::A0_NX: return Eigen::Vector3d(-1.0, 0.0, 0.0); break;
	case AzimuthBase::A0_NY: return Eigen::Vector3d(0.0, -1.0, 0.0); break;
	case AzimuthBase::A0_NZ: return Eigen::Vector3d(0.0, 0.0, -1.0); break;
	default: throw std::exception("type is not support."); break;
	}
}

Eigen::Vector3d GetAzimuth90DegreeVector(unsigned int type)
{
	unsigned int mask = 0xFF;
	switch (AzimuthBase(type & mask))
	{
	case AzimuthBase::A90_X: return Eigen::Vector3d(1.0, 0.0, 0.0); break;
	case AzimuthBase::A90_Y: return Eigen::Vector3d(0.0, 1.0, 0.0); break;
	case AzimuthBase::A90_Z: return Eigen::Vector3d(0.0, 0.0, 1.0); break;
	case AzimuthBase::A90_NX: return Eigen::Vector3d(-1.0, 0.0, 0.0); break;
	case AzimuthBase::A90_NY: return Eigen::Vector3d(0.0, -1.0, 0.0); break;
	case AzimuthBase::A90_NZ: return Eigen::Vector3d(0.0, 0.0, -1.0); break;
	default: throw std::exception("type is not support."); break;
	}
}

std::string ScannerToStr(Scanner type)
{
	switch (type)
	{
	case Scanner::BLK360: return "BLK360"; break;
	default: return "UNKNOWN"; break;
	}
}

std::string CoodSysToStr(CoodSys type)
{
	switch (type)
	{
	case CoodSys::XYZ: return "XYZ"; break;
	case CoodSys::RAE: return "RAE"; break;
	default: return "UNKNOWN"; break;
	}
}

std::string RAEModeToStr(RAEMode type)
{
	switch (type)
	{
	case RAEMode::N_X_Y		: return "N_X_Y"	; break;
	case RAEMode::N_X_Z		: return "N_X_Z"	; break;
	case RAEMode::N_X_NY	: return "N_X_NY"	; break;
	case RAEMode::N_X_NZ	: return "N_X_NZ"	; break;
	case RAEMode::N_Y_X		: return "N_Y_X"	; break;
	case RAEMode::N_Y_Z		: return "N_Y_Z"	; break;
	case RAEMode::N_Y_NX	: return "N_Y_NX"	; break;
	case RAEMode::N_Y_NZ	: return "N_Y_NZ"	; break;
	case RAEMode::N_Z_X		: return "N_Z_X"	; break;
	case RAEMode::N_Z_Y		: return "N_Z_Y"	; break;
	case RAEMode::N_Z_NX	: return "N_Z_NX"	; break;
	case RAEMode::N_Z_NY	: return "N_Z_NY"	; break;
	case RAEMode::N_NX_Y	: return "N_NX_Y"	; break;
	case RAEMode::N_NX_Z	: return "N_NX_Z"	; break;
	case RAEMode::N_NX_NY	: return "N_NX_NY"	; break;
	case RAEMode::N_NX_NZ	: return "N_NX_NZ"	; break;
	case RAEMode::N_NY_X	: return "N_NY_X"	; break;
	case RAEMode::N_NY_Z	: return "N_NY_Z"	; break;
	case RAEMode::N_NY_NX	: return "N_NY_NX"	; break;
	case RAEMode::N_NY_NZ	: return "N_NY_NZ"	; break;
	case RAEMode::N_NZ_X	: return "N_NZ_X"	; break;
	case RAEMode::N_NZ_Y	: return "N_NZ_Y"	; break;
	case RAEMode::N_NZ_NX	: return "N_NZ_NX"	; break;
	case RAEMode::N_NZ_NY	: return "N_NZ_NY"	; break;

	case RAEMode::S_X_Y		: return "S_X_Y"	; break;
	case RAEMode::S_X_Z		: return "S_X_Z"	; break;
	case RAEMode::S_X_NY	: return "S_X_NY"	; break;
	case RAEMode::S_X_NZ	: return "S_X_NZ"	; break;
	case RAEMode::S_Y_X		: return "S_Y_X"	; break;
	case RAEMode::S_Y_Z		: return "S_Y_Z"	; break;
	case RAEMode::S_Y_NX	: return "S_Y_NX"	; break;
	case RAEMode::S_Y_NZ	: return "S_Y_NZ"	; break;
	case RAEMode::S_Z_X		: return "S_Z_X"	; break;
	case RAEMode::S_Z_Y		: return "S_Z_Y"	; break;
	case RAEMode::S_Z_NX	: return "S_Z_NX"	; break;
	case RAEMode::S_Z_NY	: return "S_Z_NY"	; break;
	case RAEMode::S_NX_Y	: return "S_NX_Y"	; break;
	case RAEMode::S_NX_Z	: return "S_NX_Z"	; break;
	case RAEMode::S_NX_NY	: return "S_NX_NY"	; break;
	case RAEMode::S_NX_NZ	: return "S_NX_NZ"	; break;
	case RAEMode::S_NY_X	: return "S_NY_X"	; break;
	case RAEMode::S_NY_Z	: return "S_NY_Z"	; break;
	case RAEMode::S_NY_NX	: return "S_NY_NX"	; break;
	case RAEMode::S_NY_NZ	: return "S_NY_NZ"	; break;
	case RAEMode::S_NZ_X	: return "S_NZ_X"	; break;
	case RAEMode::S_NZ_Y	: return "S_NZ_Y"	; break;
	case RAEMode::S_NZ_NX	: return "S_NZ_NX"	; break;
	case RAEMode::S_NZ_NY	: return "S_NZ_NY"	; break;

	case RAEMode::E_X_Y		: return "E_X_Y"	; break;
	case RAEMode::E_X_Z		: return "E_X_Z"	; break;
	case RAEMode::E_X_NY	: return "E_X_NY"	; break;
	case RAEMode::E_X_NZ	: return "E_X_NZ"	; break;
	case RAEMode::E_Y_X		: return "E_Y_X"	; break;
	case RAEMode::E_Y_Z		: return "E_Y_Z"	; break;
	case RAEMode::E_Y_NX	: return "E_Y_NX"	; break;
	case RAEMode::E_Y_NZ	: return "E_Y_NZ"	; break;
	case RAEMode::E_Z_X		: return "E_Z_X"	; break;
	case RAEMode::E_Z_Y		: return "E_Z_Y"	; break;
	case RAEMode::E_Z_NX	: return "E_Z_NX"	; break;
	case RAEMode::E_Z_NY	: return "E_Z_NY"	; break;
	case RAEMode::E_NX_Y	: return "E_NX_Y"	; break;
	case RAEMode::E_NX_Z	: return "E_NX_Z"	; break;
	case RAEMode::E_NX_NY	: return "E_NX_NY"	; break;
	case RAEMode::E_NX_NZ	: return "E_NX_NZ"	; break;
	case RAEMode::E_NY_X	: return "E_NY_X"	; break;
	case RAEMode::E_NY_Z	: return "E_NY_Z"	; break;
	case RAEMode::E_NY_NX	: return "E_NY_NX"	; break;
	case RAEMode::E_NY_NZ	: return "E_NY_NZ"	; break;
	case RAEMode::E_NZ_X	: return "E_NZ_X"	; break;
	case RAEMode::E_NZ_Y	: return "E_NZ_Y"	; break;
	case RAEMode::E_NZ_NX	: return "E_NZ_NX"	; break;
	case RAEMode::E_NZ_NY	: return "E_NZ_NY"	; break;

	default: return "UNKNOWN"; break;
	}
}

Scanner StrToScanner(const std::string& str)
{
	if (str == "BLK360") return Scanner::BLK360;
	else return Scanner::Scaner_UNKNOWN;
}

CoodSys StrToCoodSys(const std::string& str)
{
	if (str == "XYZ") return CoodSys::XYZ;
	else if (str == "RAE") return CoodSys::RAE;
	else return CoodSys::CoodSys_UNKNOWN;
}

RAEMode StrToRAEMode(const std::string& str)
{
	if(		str == "N_X_Y"	) return RAEMode::N_X_Y		;
	else if(str == "N_X_Z"	) return RAEMode::N_X_Z		;
	else if(str == "N_X_NY"	) return RAEMode::N_X_NY	;
	else if(str == "N_X_NZ"	) return RAEMode::N_X_NZ	;
	else if(str == "N_Y_X"	) return RAEMode::N_Y_X		;
	else if(str == "N_Y_Z"	) return RAEMode::N_Y_Z		;
	else if(str == "N_Y_NX"	) return RAEMode::N_Y_NX	;
	else if(str == "N_Y_NZ"	) return RAEMode::N_Y_NZ	;
	else if(str == "N_Z_X"	) return RAEMode::N_Z_X		;
	else if(str == "N_Z_Y"	) return RAEMode::N_Z_Y		;
	else if(str == "N_Z_NX"	) return RAEMode::N_Z_NX	;
	else if(str == "N_Z_NY"	) return RAEMode::N_Z_NY	;
	else if(str == "N_NX_Y"	) return RAEMode::N_NX_Y	;
	else if(str == "N_NX_Z"	) return RAEMode::N_NX_Z	;
	else if(str == "N_NX_NY") return RAEMode::N_NX_NY	;
	else if(str == "N_NX_NZ") return RAEMode::N_NX_NZ	;
	else if(str == "N_NY_X"	) return RAEMode::N_NY_X	;
	else if(str == "N_NY_Z"	) return RAEMode::N_NY_Z	;
	else if(str == "N_NY_NX") return RAEMode::N_NY_NX	;
	else if(str == "N_NY_NZ") return RAEMode::N_NY_NZ	;
	else if(str == "N_NZ_X"	) return RAEMode::N_NZ_X	;
	else if(str == "N_NZ_Y"	) return RAEMode::N_NZ_Y	;
	else if(str == "N_NZ_NX") return RAEMode::N_NZ_NX	;
	else if(str == "N_NZ_NY") return RAEMode::N_NZ_NY	;

	else if(str == "S_X_Y"	) return RAEMode::S_X_Y		;
	else if(str == "S_X_Z"	) return RAEMode::S_X_Z		;
	else if(str == "S_X_NY"	) return RAEMode::S_X_NY	;
	else if(str == "S_X_NZ"	) return RAEMode::S_X_NZ	;
	else if(str == "S_Y_X"	) return RAEMode::S_Y_X		;
	else if(str == "S_Y_Z"	) return RAEMode::S_Y_Z		;
	else if(str == "S_Y_NX"	) return RAEMode::S_Y_NX	;
	else if(str == "S_Y_NZ"	) return RAEMode::S_Y_NZ	;
	else if(str == "S_Z_X"	) return RAEMode::S_Z_X		;
	else if(str == "S_Z_Y"	) return RAEMode::S_Z_Y		;
	else if(str == "S_Z_NX"	) return RAEMode::S_Z_NX	;
	else if(str == "S_Z_NY"	) return RAEMode::S_Z_NY	;
	else if(str == "S_NX_Y"	) return RAEMode::S_NX_Y	;
	else if(str == "S_NX_Z"	) return RAEMode::S_NX_Z	;
	else if(str == "S_NX_NY") return RAEMode::S_NX_NY	;
	else if(str == "S_NX_NZ") return RAEMode::S_NX_NZ	;
	else if(str == "S_NY_X"	) return RAEMode::S_NY_X	;
	else if(str == "S_NY_Z"	) return RAEMode::S_NY_Z	;
	else if(str == "S_NY_NX") return RAEMode::S_NY_NX	;
	else if(str == "S_NY_NZ") return RAEMode::S_NY_NZ	;
	else if(str == "S_NZ_X"	) return RAEMode::S_NZ_X	;
	else if(str == "S_NZ_Y"	) return RAEMode::S_NZ_Y	;
	else if(str == "S_NZ_NX") return RAEMode::S_NZ_NX	;
	else if(str == "S_NZ_NY") return RAEMode::S_NZ_NY	;

	else if(str == "E_X_Y"	) return RAEMode::E_X_Y		;
	else if(str == "E_X_Z"	) return RAEMode::E_X_Z		;
	else if(str == "E_X_NY"	) return RAEMode::E_X_NY	;
	else if(str == "E_X_NZ"	) return RAEMode::E_X_NZ	;
	else if(str == "E_Y_X"	) return RAEMode::E_Y_X		;
	else if(str == "E_Y_Z"	) return RAEMode::E_Y_Z		;
	else if(str == "E_Y_NX"	) return RAEMode::E_Y_NX	;
	else if(str == "E_Y_NZ"	) return RAEMode::E_Y_NZ	;
	else if(str == "E_Z_X"	) return RAEMode::E_Z_X		;
	else if(str == "E_Z_Y"	) return RAEMode::E_Z_Y		;
	else if(str == "E_Z_NX"	) return RAEMode::E_Z_NX	;
	else if(str == "E_Z_NY"	) return RAEMode::E_Z_NY	;
	else if(str == "E_NX_Y"	) return RAEMode::E_NX_Y	;
	else if(str == "E_NX_Z"	) return RAEMode::E_NX_Z	;
	else if(str == "E_NX_NY") return RAEMode::E_NX_NY	;
	else if(str == "E_NX_NZ") return RAEMode::E_NX_NZ	;
	else if(str == "E_NY_X"	) return RAEMode::E_NY_X	;
	else if(str == "E_NY_Z"	) return RAEMode::E_NY_Z	;
	else if(str == "E_NY_NX") return RAEMode::E_NY_NX	;
	else if(str == "E_NY_NZ") return RAEMode::E_NY_NZ	;
	else if(str == "E_NZ_X"	) return RAEMode::E_NZ_X	;
	else if(str == "E_NZ_Y"	) return RAEMode::E_NZ_Y	;
	else if(str == "E_NZ_NX") return RAEMode::E_NZ_NX	;
	else if(str == "E_NZ_NY") return RAEMode::E_NZ_NY	;

	else return RAEMode::RAEMode_UNKNOWN;
}

Eigen::Vector3d RAEToXYZ(RAEMode type, const Eigen::Vector3d& rae)
{
	ElevationMode elevationMode = GetElevationMode(type);
	double r = rae.x();
	double s_e = std::sin(rae.z());
	double s_a = std::sin(rae.y());
	double c_e = std::cos(rae.z());
	double c_a = std::cos(rae.y());
	Eigen::Vector3d uv_a0 = GetAzimuth0DegreeVector(type);
	Eigen::Vector3d uv_a90 = GetAzimuth90DegreeVector(type);
	Eigen::Vector3d uv_n = uv_a0.cross(uv_a90);

	//
	double len_n;
	double len_e;
	switch (elevationMode)
	{
	case ElevationMode::N:
		len_n = r* c_e;
		len_e = r * s_e;
		break;
	case ElevationMode::S:
		len_n = r * (-c_e);
		len_e = r * s_e;
		break;
	case ElevationMode::E:
		len_n = r * s_e;
		len_e = r * c_e;
		break;
	default:
		throw std::exception("ElevationMode is not support.");
		break;
	}

	return uv_a0 * len_e * c_a + uv_a90 * len_e * s_a + uv_n * len_n;
}

Eigen::Vector3d XYZToRAE(RAEMode type, const Eigen::Vector3d& xyz)
{
	ElevationMode elevationMode = GetElevationMode(type);
	Eigen::Vector3d rae;
	rae.x() = xyz.norm();
	Eigen::Vector3d uv_xyz = xyz / rae.x();
	Eigen::Vector3d uv_a0 = GetAzimuth0DegreeVector(type);
	Eigen::Vector3d uv_a90 = GetAzimuth90DegreeVector(type);
	Eigen::Vector3d uv_n = uv_a0.cross(uv_a90);

	double len_n = uv_xyz.dot(uv_n);
	switch (elevationMode)
	{
	case ElevationMode::N:
		rae.z() = std::acos(len_n);
		break;
	case ElevationMode::S:
		rae.z() = M_PI - std::acos(len_n);
		break;
	case ElevationMode::E:
		rae.z() = std::asin(len_n);
		break;
	default:
		throw std::exception("ElevationMode is not support.");
		break;
	}

	Eigen::Vector3d v_e = uv_xyz - len_n * uv_n;
	rae.y() = std::atan2(v_e.dot(uv_a90), v_e.dot(uv_a0));
	return rae;
}

Eigen::Vector2d RAEToUV(RAEMode type, const Eigen::Vector3d& rae)
{
	Eigen::Vector2d p2;
	switch (type)
	{
	case RAEMode::E_X_Y:
		p2.x() = 0.5 * (1.0 - rae.y() / (M_PI));
		p2.y() = 0.5 * (1.0 + rae.z() / (M_PI*0.5));
		p2.x() = std::min(std::max(0.0, p2.x()), 1.0);
		p2.y() = std::min(std::max(0.0, p2.y()), 1.0);
		return p2;
	default:
		throw std::exception("RAEMode is not support.");
		break;
	}
}