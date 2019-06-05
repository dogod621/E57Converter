#include <iostream>
#include <iomanip>
#include <cmath>

#include "E57Utils.h"

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

	// Parse Node
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

	void PrintFormat(const boost::filesystem::path& e57Path)
	{
		try
		{
			e57::ImageFile imf(e57Path.string().c_str(), "r");
			e57::ParseNode(0, imf.root());
		}
		catch (e57::E57Exception& ex)
		{
			std::cerr << "[PrintE57Format] Got an e57::E57Exception, what=" << ex.what() << "." << std::endl;
		}
		catch (std::exception& ex)
		{
			std::cerr << "[PrintE57Format] Got an std::exception, what=" << ex.what() << "." << std::endl;
		}
		catch (...)
		{
			std::cerr << "[PrintE57Format] Got an unknown exception." << std::endl;
		}
	}

	//
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
		case RAEMode::N_X_Y: return "N_X_Y"; break;
		case RAEMode::N_X_Z: return "N_X_Z"; break;
		case RAEMode::N_Y_X: return "N_Y_X"; break;
		case RAEMode::N_Y_Z: return "N_Y_Z"; break;
		case RAEMode::N_Z_X: return "N_Z_X"; break;
		case RAEMode::N_Z_Y: return "N_Z_Y"; break;

		case RAEMode::S_X_Y: return "S_X_Y"; break;
		case RAEMode::S_X_Z: return "S_X_Z"; break;
		case RAEMode::S_Y_X: return "S_Y_X"; break;
		case RAEMode::S_Y_Z: return "S_Y_Z"; break;
		case RAEMode::S_Z_X: return "S_Z_X"; break;
		case RAEMode::S_Z_Y: return "S_Z_Y"; break;

		case RAEMode::E_X_Y: return "E_X_Y"; break;
		case RAEMode::E_X_Z: return "E_X_Z"; break;
		case RAEMode::E_Y_X: return "E_Y_X"; break;
		case RAEMode::E_Y_Z: return "E_Y_Z"; break;
		case RAEMode::E_Z_X: return "E_Z_X"; break;
		case RAEMode::E_Z_Y: return "E_Z_Y"; break;

		default: return "UNKNOWN"; break;
		}
	}

	CoodSys StrToCoodSys(const std::string& str)
	{
		if (str == "XYZ") return CoodSys::XYZ;
		else if (str == "RAE") return CoodSys::RAE;
		else return CoodSys::CoodSys_UNKNOWN;
	}

	RAEMode StrToRAEMode(const std::string& str)
	{
		if (str == "N_X_Y") return RAEMode::N_X_Y;
		else if (str == "N_X_Z") return RAEMode::N_X_Z;
		else if (str == "N_Y_X") return RAEMode::N_Y_X;
		else if (str == "N_Y_Z") return RAEMode::N_Y_Z;
		else if (str == "N_Z_X") return RAEMode::N_Z_X;
		else if (str == "N_Z_Y") return RAEMode::N_Z_Y;

		else if (str == "S_X_Y") return RAEMode::S_X_Y;
		else if (str == "S_X_Z") return RAEMode::S_X_Z;
		else if (str == "S_Y_X") return RAEMode::S_Y_X;
		else if (str == "S_Y_Z") return RAEMode::S_Y_Z;
		else if (str == "S_Z_X") return RAEMode::S_Z_X;
		else if (str == "S_Z_Y") return RAEMode::S_Z_Y;

		else if (str == "E_X_Y") return RAEMode::E_X_Y;
		else if (str == "E_X_Z") return RAEMode::E_X_Z;
		else if (str == "E_Y_X") return RAEMode::E_Y_X;
		else if (str == "E_Y_Z") return RAEMode::E_Y_Z;
		else if (str == "E_Z_X") return RAEMode::E_Z_X;
		else if (str == "E_Z_Y") return RAEMode::E_Z_Y;

		else return RAEMode::RAEMode_UNKNOWN;
	}

	Eigen::Vector3d RAEToXYZ(RAEMode type, const Eigen::Vector3d& p)
	{
		Eigen::Vector3d p2;

		double r = p.x();
		double s_e = std::sin(p.z());
		double s_a = std::sin(p.y());
		double c_e = std::cos(p.z());
		double c_a = std::cos(p.y());

		switch (type)
		{
		case RAEMode::N_X_Y:
			p2.x() = r * s_e * c_a;
			p2.y() = r * s_e * s_a;
			p2.z() = r * c_e;
			return p2;

		case RAEMode::N_X_Z:
			p2.x() = r * s_e * c_a;
			p2.y() = -r * c_e;
			p2.z() = r * s_e * s_a;
			return p2;

		case RAEMode::N_Y_X:
			p2.x() = r * s_e * s_a;
			p2.y() = r * s_e * c_a;
			p2.z() = -r * c_e;
			return p2;

		case RAEMode::N_Y_Z:
			p2.x() = r * c_e;
			p2.y() = r * s_e * c_a;
			p2.z() = r * s_e * s_a;
			return p2;

		case RAEMode::N_Z_X:
			p2.x() = r * s_e * s_a;
			p2.y() = r * c_e;
			p2.z() = r * s_e * c_a;
			return p2;

		case RAEMode::N_Z_Y:
			p2.x() = -r * c_e;
			p2.y() = r * s_e * s_a;
			p2.z() = r * s_e * c_a;
			return p2;

		case RAEMode::S_X_Y:
			p2.x() = r * s_e * c_a;
			p2.y() = r * s_e * s_a;
			p2.z() = -r * c_e;
			return p2;

		case RAEMode::S_X_Z:
			p2.x() = r * s_e * c_a;
			p2.y() = r * c_e;
			p2.z() = r * s_e * s_a;
			return p2;

		case RAEMode::S_Y_X:
			p2.x() = r * s_e * s_a;
			p2.y() = r * s_e * c_a;
			p2.z() = r * c_e;
			return p2;

		case RAEMode::S_Y_Z:
			p2.x() = -r * c_e;
			p2.y() = r * s_e * c_a;
			p2.z() = r * s_e * s_a;
			return p2;

		case RAEMode::S_Z_X:
			p2.x() = r * s_e * s_a;
			p2.y() = -r * c_e;
			p2.z() = r * s_e * c_a;
			return p2;

		case RAEMode::S_Z_Y:
			p2.x() = r * c_e;
			p2.y() = r * s_e * s_a;
			p2.z() = r * s_e * c_a;
			return p2;

		case RAEMode::E_X_Y:
			p2.x() = r * c_e * c_a;
			p2.y() = r * c_e * s_a;
			p2.z() = r * s_e;
			return p2;

		case RAEMode::E_X_Z:
			p2.x() = r * c_e * c_a;
			p2.y() = -r * s_e;
			p2.z() = r * c_e * s_a;
			return p2;

		case RAEMode::E_Y_X:
			p2.x() = r * c_e * s_a;
			p2.y() = r * c_e * c_a;
			p2.z() = -r * s_e;
			return p2;

		case RAEMode::E_Y_Z:
			p2.x() = r * s_e;
			p2.y() = r * c_e * c_a;
			p2.z() = r * c_e * s_a;
			return p2;

		case RAEMode::E_Z_X:
			p2.x() = r * c_e * s_a;
			p2.y() = r * s_e;
			p2.z() = r * c_e * c_a;
			return p2;

		case RAEMode::E_Z_Y:
			p2.x() = -r * s_e;
			p2.y() = r * c_e * s_a;
			p2.z() = r * c_e * c_a;
			return p2;

		default:
			throw std::exception("RAEMode is not support.");
			break;
		}
	}

	Eigen::Vector3d XYZToRAE(RAEMode type, const Eigen::Vector3d& p)
	{
		Eigen::Vector3d p2;
		double r = p.norm();
		p2.x() = r;

		switch (type)
		{
		case RAEMode::N_X_Y:
			p2.y() = std::atan2(p.y(), p.x());
			p2.z() = std::acos(p.z() / r);
			return p2;

		case RAEMode::N_X_Z:
			p2.y() = std::atan2(p.z(), p.x());
			p2.z() = std::acos(-p.y() / r);
			return p2;

		case RAEMode::N_Y_X:
			p2.y() = std::atan2(p.x(), p.y());
			p2.z() = std::acos(-p.z() / r);
			return p2;

		case RAEMode::N_Y_Z:
			p2.y() = std::atan2(p.z(), p.y());
			p2.z() = std::acos(p.x() / r);
			return p2;

		case RAEMode::N_Z_X:
			p2.y() = std::atan2(p.x(), p.z());
			p2.z() = std::acos(p.y() / r);
			return p2;

		case RAEMode::N_Z_Y:
			p2.y() = std::atan2(p.y(), p.z());
			p2.z() = std::acos(-p.x() / r);
			return p2;

		case RAEMode::S_X_Y:
			p2.y() = std::atan2(p.y(), p.x());
			p2.z() = M_PI - std::acos(p.z() / r);
			return p2;

		case RAEMode::S_X_Z:
			p2.y() = std::atan2(p.z(), p.x());
			p2.z() = M_PI - std::acos(-p.y() / r);
			return p2;

		case RAEMode::S_Y_X:
			p2.y() = std::atan2(p.x(), p.y());
			p2.z() = M_PI - std::acos(-p.z() / r);
			return p2;

		case RAEMode::S_Y_Z:
			p2.y() = std::atan2(p.z(), p.y());
			p2.z() = M_PI - std::acos(p.x() / r);
			return p2;

		case RAEMode::S_Z_X:
			p2.y() = std::atan2(p.x(), p.z());
			p2.z() = M_PI - std::acos(p.y() / r);
			return p2;

		case RAEMode::S_Z_Y:
			p2.y() = std::atan2(p.y(), p.z());
			p2.z() = M_PI - std::acos(-p.x() / r);
			return p2;

		case RAEMode::E_X_Y:
			p2.y() = std::atan2(p.y(), p.x());
			p2.z() = std::asin(p.z() / r);
			return p2;

		case RAEMode::E_X_Z:
			p2.y() = std::atan2(p.z(), p.x());
			p2.z() = std::asin(-p.y() / r);
			return p2;

		case RAEMode::E_Y_X:
			p2.y() = std::atan2(p.x(), p.y());
			p2.z() = std::asin(-p.z() / r);
			return p2;

		case RAEMode::E_Y_Z:
			p2.y() = std::atan2(p.z(), p.y());
			p2.z() = std::asin(p.x() / r);
			return p2;

		case RAEMode::E_Z_X:
			p2.y() = std::atan2(p.x(), p.z());
			p2.z() = std::asin(p.y() / r);
			return p2;

		case RAEMode::E_Z_Y:
			p2.y() = std::atan2(p.y(), p.z());
			p2.z() = std::asin(-p.x() / r);
			return p2;

		default:
			throw std::exception("RAEMode is not support.");
			break;
		}
	}

	Eigen::Vector2d RAEToUV(RAEMode type, const Eigen::Vector3d& p)
	{
		Eigen::Vector2d p2;
		switch (type)
		{
		case RAEMode::E_X_Y:
			p2.x() = p.y();
			p2.y() = p.z() + (M_PI * 0.5);
			if (p2.x() < 0)
				p2.x() += 2.0 * M_PI;
			p2.x() /= (2.0 * M_PI);
			p2.y() /= M_PI;
			p2.x() = std::min(std::max(0.0, p2.x()), 1.0);
			p2.y() = std::min(std::max(0.0, p2.y()), 1.0);
			return p2;
		default:
			throw std::exception("RAEMode is not support.");
			break;
		}
	}

	void Scan::Load(const e57::ImageFile& imf, const e57::VectorNode& data3D, int64_t scanID)
	{
		ID = scanID;
		e57::StructureNode scan(data3D.get(scanID));
		std::vector<e57::SourceDestBuffer> sdBuffers;

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
				PCL_WARN("[e57::%s::Load] Scan didnot define pose translation.\n", "Scan");

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
				PCL_WARN("[e57::%s::Load] Scan didnot define pose rotation.\n", "Scan");

			std::stringstream ss;
			ss << "[e57::%s::Load] Scan transform - " << transform << ".\n";
			PCL_INFO(ss.str().c_str(), "Scan");
		}
		else
			PCL_WARN("[e57::%s::Load] Scan didnot define pose.\n", "Scan");

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
				numPoints = scanPoints.childCount();

				if (proto.isDefined("cartesianX") && proto.isDefined("cartesianY") && proto.isDefined("cartesianZ"))
				{
					coodSys = CoodSys::XYZ;
					hasPointXYZ = true;
					protoXNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("cartesianX")));
					protoYNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("cartesianY")));
					protoZNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("cartesianZ")));
					x = std::shared_ptr<float>(new float[numPoints]);
					y = std::shared_ptr<float>(new float[numPoints]);
					z = std::shared_ptr<float>(new float[numPoints]);
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "cartesianX", x.get(), numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "cartesianY", y.get(), numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "cartesianZ", z.get(), numPoints, true, true));
				}
				else if (proto.isDefined("sphericalRange") && proto.isDefined("sphericalAzimuth") && proto.isDefined("sphericalElevation"))
				{
					coodSys = CoodSys::RAE;
					raeMode = RAEMode::E_X_Y; // E57 use this
					hasPointXYZ = true;
					protoXNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("sphericalRange")));
					protoYNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("sphericalAzimuth")));
					protoZNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("sphericalElevation")));
					x = std::shared_ptr<float>(new float[numPoints]);
					y = std::shared_ptr<float>(new float[numPoints]);
					z = std::shared_ptr<float>(new float[numPoints]);
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalRange", x.get(), numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalAzimuth", y.get(), numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "sphericalElevation", z.get(), numPoints, true, true));
				}

				if (proto.isDefined("colorRed") && proto.isDefined("colorGreen") && proto.isDefined("colorBlue") && E57_CAN_CONTAIN_RGB)
				{
					hasPointRGB = true;
					protoRNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("colorRed")));
					protoGNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("colorGreen")));
					protoBNode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("colorBlue")));
					r = std::shared_ptr<uint8_t>(new uint8_t[numPoints]);
					g = std::shared_ptr<uint8_t>(new uint8_t[numPoints]);
					b = std::shared_ptr<uint8_t>(new uint8_t[numPoints]);
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "colorRed", r.get(), numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "colorGreen", g.get(), numPoints, true, true));
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "colorBlue", b.get(), numPoints, true, true));
				}

				if (proto.isDefined("intensity") && E57_CAN_CONTAIN_INTENSITY)
				{
					hasPointI = true;
					protoINode = std::shared_ptr<e57::Node>(new e57::Node(proto.get("intensity")));
					i = std::shared_ptr<float>(new float[numPoints]);
					sdBuffers.push_back(e57::SourceDestBuffer(imf, "intensity", i.get(), numPoints, true, true));
				}

				//
				if ((hasPointXYZ || hasPointRGB || hasPointI))
				{
					e57::CompressedVectorReader reader = scanPoints.reader(sdBuffers);
					if (reader.read() <= 0)
					{
						PCL_WARN("[e57::%s::Load] Failed to read E57 points, ignore the scan.\n", "Scan");
						hasPointXYZ = false;
						hasPointRGB = false;
						hasPointI = false;
					}
					reader.close();
				}
			}
			else
				PCL_WARN("[e57::%s::Load] Not supported scan points type.\n", "Scan");
		}
		else
			PCL_WARN("[e57::%s::Load] Scan didnot define points.\n", "Scan");
	}

	void Scan::ToPointCloud(pcl::PointCloud<PointE57>& scanCloud)
	{
		if ((hasPointXYZ || hasPointRGB || hasPointI))
		{
			scanCloud.reserve(numPoints);
			float* _x = x.get();
			float* _y = y.get();
			float* _z = z.get();
			float* _i = i.get();
			uint8_t * _r = r.get();
			uint8_t * _g = g.get();
			uint8_t * _b = b.get();

			for (int64_t pi = 0; pi < numPoints; ++pi)
			{
				PointE57 sp;
				if (hasPointXYZ)
				{
					switch (coodSys)
					{
					case CoodSys::XYZ:
						sp.x = _x[pi];
						sp.y = _y[pi];
						sp.z = _z[pi];
						break;

					case CoodSys::RAE:
					{
						Eigen::Vector3d xyz = RAEToXYZ(raeMode, Eigen::Vector3d(_x[pi], _y[pi], _z[pi]));

						sp.x = xyz.x();
						sp.y = xyz.y();
						sp.z = xyz.z();
					}
					break;

					default:
						throw std::exception("Coordinate system invalid!!?");
						break;
					}
				}

#ifdef POINT_E57_WITH_RGB
				if (hasPointRGB)
				{
					sp.r = _r[pi];
					sp.g = _g[pi];
					sp.b = _b[pi];
				}
				else
				{
					sp.r = 255;
					sp.g = 255;
					sp.b = 255;
				}
#endif

#ifdef POINT_E57_WITH_INTENSITY
				if (hasPointI)
				{
					sp.intensity = _i[pi];
				}
#endif

#ifdef POINT_E57_WITH_LABEL
				sp.label = (uint32_t)ID;
#endif
				if (sp.Valid())
					scanCloud.push_back(sp);
			}
			pcl::transformPointCloud(scanCloud, scanCloud, transform);
		}
	}
}