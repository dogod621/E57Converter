#pragma once

#include <pcl/point_types.h>

#ifdef POINT_E57_WITH_RGB
#define E57_CAN_CONTAIN_RGB true
#else
#define E57_CAN_CONTAIN_RGB false
#endif

#ifdef POINT_E57_WITH_INTENSITY
#define E57_CAN_CONTAIN_INTENSITY true
#else
#define E57_CAN_CONTAIN_INTENSITY false
#endif

#ifdef POINT_E57_WITH_LABEL
#define E57_CAN_CONTAIN_LABEL true
#else
#define E57_CAN_CONTAIN_LABEL false
#endif

#ifdef POINT_E57_WITH_HDR
#define E57_CAN_CONTAIN_HDR true
#else
#define E57_CAN_CONTAIN_HDR false
#endif

#ifdef POINT_PCD_WITH_RGB
#define PCD_CAN_CONTAIN_RGB true
#else
#define PCD_CAN_CONTAIN_RGB false
#endif

#ifdef POINT_PCD_WITH_INTENSITY
#define PCD_CAN_CONTAIN_INTENSITY true
#else
#define PCD_CAN_CONTAIN_INTENSITY false
#endif

#ifdef POINT_PCD_WITH_NORMAL
#define PCD_CAN_CONTAIN_NORMAL true
#else
#define PCD_CAN_CONTAIN_NORMAL false
#endif

#ifdef POINT_PCD_WITH_LABEL
#define PCD_CAN_CONTAIN_LABEL true
#else
#define PCD_CAN_CONTAIN_LABEL false
#endif

//
#define ADD_LABEL union { uint32_t label; int32_t hasLabel; };

#define ADD_NORMAL_RGB_INTENSITY_LABEL \
	PCL_ADD_NORMAL4D; \
	union EIGEN_ALIGN16 \
	{ \
		struct \
		{ \
			PCL_ADD_UNION_RGB; \
			PCL_ADD_INTENSITY; \
			ADD_LABEL; \
			float curvature; \
		}; \
		float data_c[4]; \
	}; \
	PCL_ADD_EIGEN_MAPS_RGB;

#define ADD_RGB_INTENSITY_LABEL \
	union EIGEN_ALIGN16 \
	{ \
		struct \
		{ \
			PCL_ADD_UNION_RGB; \
			PCL_ADD_INTENSITY; \
			ADD_LABEL; \
		}; \
		float data_c[4]; \
	}; \
	PCL_ADD_EIGEN_MAPS_RGB;



struct EIGEN_ALIGN16 _PointE57
{
	PCL_ADD_POINT4D;

#ifdef POINT_E57_WITH_HDR
	union EIGEN_ALIGN16 { float data_hdr[4]; struct { float hdr_r; float hdr_g; float hdr_b; float hdr_a; }; };
#endif

#ifdef POINT_E57_WITH_RGB
#	ifdef POINT_E57_WITH_INTENSITY
#		ifdef POINT_E57_WITH_LABEL
			ADD_RGB_INTENSITY_LABEL;
#		else
			ADD_RGB_INTENSITY_LABEL;
#		endif
#	elif defined POINT_E57_WITH_LABEL
		ADD_RGB_INTENSITY_LABEL;
#	else
		PCL_ADD_RGB;
#	endif

#elif defined POINT_E57_WITH_INTENSITY
#	ifdef POINT_E57_WITH_LABEL
		ADD_RGB_INTENSITY_LABEL;
#	else
		PCL_ADD_INTENSITY;
#	endif

#elif defined POINT_E57_WITH_LABEL
	ADD_LABEL;
#endif

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 _PointExchange
{
	PCL_ADD_POINT4D;

#ifdef POINT_E57_WITH_HDR
	union EIGEN_ALIGN16 { float data_hdr[4]; struct { float hdr_r; float hdr_g; float hdr_b; float hdr_a; }; };
#endif

	ADD_NORMAL_RGB_INTENSITY_LABEL;
	union { uint32_t segmentLabel; int32_t hasSegmentLabel; };

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 _PointPCD
{
	PCL_ADD_POINT4D;

#ifdef POINT_PCD_WITH_NORMAL
	ADD_NORMAL_RGB_INTENSITY_LABEL;
#elif defined POINT_PCD_WITH_RGB
#	ifdef POINT_PCD_WITH_INTENSITY
#		ifdef POINT_PCD_WITH_LABEL
			ADD_RGB_INTENSITY_LABEL;
#		else
			ADD_RGB_INTENSITY_LABEL;
#		endif
#	elif defined POINT_PCD_WITH_LABEL
		ADD_RGB_INTENSITY_LABEL;
#	else
		PCL_ADD_RGB;
#	endif

#elif defined POINT_PCD_WITH_INTENSITY
#	ifdef POINT_PCD_WITH_LABEL
		ADD_RGB_INTENSITY_LABEL;
#	else
		PCL_ADD_INTENSITY;
#	endif

#elif defined POINT_PCD_WITH_LABEL
	ADD_LABEL;
#endif
};

struct EIGEN_ALIGN16 _PointNDF
{
	PCL_ADD_POINT4D;
	PCL_ADD_INTENSITY;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//
struct PointExchange;
struct PointE57;
struct PointPCD;

struct PointE57 : public _PointE57
{
	inline PointE57(const PointE57& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
#ifdef POINT_E57_WITH_HDR
		hdr_r = p.hdr_r; hdr_g = p.hdr_g; hdr_b = p.hdr_b; hdr_a = p.hdr_a;
#endif
#ifdef POINT_E57_WITH_RGB
		rgba = p.rgba;
#endif
#ifdef POINT_E57_WITH_INTENSITY
		intensity = p.intensity;
#endif
#ifdef POINT_E57_WITH_LABEL
		label = p.label;
#endif
	}

	inline PointE57(const PointExchange& p);

	inline PointE57(const PointPCD& p);

	inline PointE57()
	{
		Clear();
	}

	inline void Clear()
	{
		x = y = z = 0.0f; data[3] = 1.f;
#ifdef POINT_E57_WITH_HDR
		hdr_r = hdr_g = hdr_b = hdr_a = 0.f;
#endif
#ifdef POINT_E57_WITH_RGB
		r = g = b = 0; a = 1;
#endif
#ifdef POINT_E57_WITH_INTENSITY
		intensity = 0.f;
#endif
#ifdef POINT_E57_WITH_LABEL
		hasLabel = -1;
#endif
	}

	inline bool Valid()
	{
		return
			std::isfinite(x) && std::isfinite(y) && std::isfinite(z)
#ifdef POINT_E57_WITH_HDR
			&& std::isfinite(hdr_r) && std::isfinite(hdr_g) && std::isfinite(hdr_b) && std::isfinite(hdr_a)
#endif
#ifdef POINT_E57_WITH_INTENSITY
			&& std::isfinite(intensity)
#endif
			;
	}

#ifdef POINT_E57_WITH_LABEL
	inline bool HasLabel()
	{
		return !(hasLabel == -1);
	}
#endif
};

struct PointExchange : public _PointExchange
{
	inline PointExchange(const PointExchange& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
#ifdef POINT_PCD_WITH_NORMAL
		normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
#endif
#ifdef POINT_E57_WITH_HDR
		hdr_r = p.hdr_r; hdr_g = p.hdr_g; hdr_b = p.hdr_b; hdr_a = p.hdr_a;
#endif
#ifdef POINT_E57_WITH_RGB
		rgba = p.rgba;
#endif
#ifdef POINT_E57_WITH_INTENSITY
		intensity = p.intensity;
#endif
#ifdef POINT_E57_WITH_LABEL
		label = p.label;
#endif
#ifdef POINT_PCD_WITH_LABEL
		segmentLabel = p.segmentLabel;
#endif
	}

	inline PointExchange(const PointE57& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
#ifdef POINT_PCD_WITH_NORMAL
		normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
#endif
#ifdef POINT_E57_WITH_HDR
		hdr_r = p.hdr_r; hdr_g = p.hdr_g; hdr_b = p.hdr_b; hdr_a = p.hdr_a;
#endif
#ifdef POINT_E57_WITH_RGB
		rgba = p.rgba;
#endif
#ifdef POINT_E57_WITH_INTENSITY
		intensity = p.intensity;
#endif
#ifdef POINT_E57_WITH_LABEL
		label = p.label;
#endif
#ifdef POINT_PCD_WITH_LABEL
		hasSegmentLabel = -1;
#endif
	}

	inline PointExchange(const PointPCD& p);

	inline PointExchange()
	{
		Clear();
	}

	inline void Clear()
	{
		x = y = z = 0.0f; data[3] = 1.f;
#ifdef POINT_PCD_WITH_NORMAL
		normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
#endif
#ifdef POINT_E57_WITH_HDR
		hdr_r = hdr_g = hdr_b = hdr_a = 0.f;
#endif
#ifdef POINT_E57_WITH_RGB
		r = g = b = 0; a = 1;
#endif
#ifdef POINT_E57_WITH_INTENSITY
		intensity = 0.f;
#endif
#ifdef POINT_E57_WITH_LABEL
		hasLabel = -1;
#endif
#ifdef POINT_PCD_WITH_LABEL
		hasSegmentLabel = -1;
#endif
	}

	inline bool Valid()
	{
		return
			std::isfinite(x) && std::isfinite(y) && std::isfinite(z)
#ifdef POINT_PCD_WITH_NORMAL
			&& std::isfinite(normal_x) && std::isfinite(normal_y) && std::isfinite(normal_z) && std::isfinite(curvature)
#endif
#ifdef POINT_E57_WITH_HDR
			&& std::isfinite(hdr_r) && std::isfinite(hdr_g) && std::isfinite(hdr_b) && std::isfinite(hdr_a)
#endif
#ifdef POINT_E57_WITH_INTENSITY
			&& std::isfinite(intensity)
#endif
			;
	}

#ifdef POINT_E57_WITH_LABEL
	inline bool HasLabel()
	{
		return !(hasLabel == -1);
	}
#endif

#ifdef POINT_PCD_WITH_LABEL
	inline bool HasSegmentLabel()
	{
		return !(hasSegmentLabel == -1);
	}
#endif
};

struct PointPCD : public _PointPCD
{
	inline PointPCD(const PointPCD& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
#ifdef POINT_PCD_WITH_NORMAL
		normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
#endif
#ifdef POINT_PCD_WITH_RGB
		rgba = p.rgba;
#endif
#ifdef POINT_PCD_WITH_INTENSITY
		intensity = p.intensity;
#endif
#ifdef POINT_PCD_WITH_LABEL
		label = p.label;
#endif
	}

	inline PointPCD(const PointE57& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
#ifdef POINT_PCD_WITH_NORMAL
		normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
#endif
#ifdef POINT_PCD_WITH_RGB
#	ifdef POINT_E57_WITH_RGB
		rgba = p.rgba;
#	else
		r = g = b = 0; a = 1;
#	endif
#endif
#ifdef POINT_PCD_WITH_INTENSITY
#	ifdef POINT_E57_WITH_INTENSITY
		intensity = p.intensity;
#	else
		intensity = 0.f;
#	endif
#endif
#ifdef POINT_PCD_WITH_LABEL
		hasLabel = -1;
#endif
	}

	inline PointPCD(const PointExchange& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
#ifdef POINT_PCD_WITH_NORMAL
		normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
#endif
#ifdef POINT_PCD_WITH_RGB
#	ifdef POINT_E57_WITH_RGB
		rgba = p.rgba;
#	else
		r = g = b = 0; a = 1;
#	endif
#endif
#ifdef POINT_PCD_WITH_INTENSITY
#	ifdef POINT_E57_WITH_INTENSITY
		intensity = p.intensity;
#	else
		intensity = 0.f;
#	endif
#endif
#ifdef POINT_PCD_WITH_LABEL
		label = p.segmentLabel;
#endif
	}

	inline PointPCD()
	{
		Clear();
	}

	inline void Clear()
	{
		x = y = z = 0.0f; data[3] = 1.f;
#ifdef POINT_PCD_WITH_NORMAL
		normal_x = normal_y = normal_z = data_n[3] = curvature = 0.f;
#endif
#ifdef POINT_PCD_WITH_RGB
		r = g = b = 0; a = 1;
#endif
#ifdef POINT_PCD_WITH_INTENSITY
		intensity = 0.f;
#endif
#ifdef POINT_PCD_WITH_LABEL
		hasLabel = -1;
#endif
	}

	inline bool Valid()
	{
		return
			std::isfinite(x) && std::isfinite(y) && std::isfinite(z)
#ifdef POINT_PCD_WITH_NORMAL
			&& std::isfinite(normal_x) && std::isfinite(normal_y) && std::isfinite(normal_z) && std::isfinite(curvature)
#endif
#ifdef POINT_PCD_WITH_INTENSITY
			&& std::isfinite(intensity)
#endif
			;
	}

#ifdef POINT_PCD_WITH_LABEL
	inline bool HasLabel()
	{
		return !(hasLabel == -1);
	}
#endif
};

struct PointNDF : public _PointNDF
{
	inline PointNDF(const PointNDF& p)
	{
		x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
		intensity = p.intensity;
	}

	inline PointNDF()
	{
		Clear();
	}

	inline void Clear()
	{
		x = y = z = 0.0f; data[3] = 1.f;
		intensity = 0.f;
	}

	inline bool Valid()
	{
		return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(intensity);
	}
};

inline PointE57::PointE57(const PointExchange& p)
{
	x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
#ifdef POINT_E57_WITH_HDR
	hdr_r = p.hdr_r; hdr_g = p.hdr_g; hdr_b = p.hdr_b; hdr_a = p.hdr_a;
#endif
#ifdef POINT_E57_WITH_RGB
	rgba = p.rgba;
#endif
#ifdef POINT_E57_WITH_INTENSITY
	intensity = p.intensity;
#endif
#ifdef POINT_E57_WITH_LABEL
	label = p.label;
#endif
}

inline PointE57::PointE57(const PointPCD& p)
{
	x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
#ifdef POINT_E57_WITH_HDR
	hdr_r = hdr_g = hdr_b = hdr_a = 0.f;
#endif
#ifdef POINT_E57_WITH_RGB
#	ifdef POINT_PCD_WITH_RGB
	rgba = p.rgba;
#	else
	r = g = b = 0; a = 1;
#	endif
#endif
#ifdef POINT_E57_WITH_INTENSITY
#	ifdef POINT_PCD_WITH_INTENSITY
	intensity = p.intensity;
#	else
	intensity = 0.f;
#	endif
#endif
#ifdef POINT_E57_WITH_LABEL
	hasLabel = -1;
#endif
}

inline PointExchange::PointExchange(const PointPCD& p)
{
	x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
#ifdef POINT_PCD_WITH_NORMAL
	normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.f; curvature = p.curvature;
#endif
#ifdef POINT_E57_WITH_HDR
	hdr_r = hdr_g = hdr_b = hdr_a = 0.f;
#endif
#ifdef POINT_E57_WITH_RGB
#	ifdef POINT_PCD_WITH_RGB
	rgba = p.rgba;
#	else
	r = g = b = 0; a = 1;
#	endif
#endif
#ifdef POINT_E57_WITH_INTENSITY
#	ifdef POINT_PCD_WITH_INTENSITY
	intensity = p.intensity;
#	else
	intensity = 0.f;
#	endif
#endif
#ifdef POINT_E57_WITH_LABEL
	hasLabel = -1;
#endif
#ifdef POINT_PCD_WITH_LABEL
	segmentLabel = p.label;
#endif
}

//
#define REGISTER_E57_XYZ (float, x, x) (float, y, y) (float, z, z)
#ifdef POINT_E57_WITH_HDR
#define REGISTER_E57_HDR (float, hdr_r, hdr_r) (float, hdr_g, hdr_g) (float, hdr_b, hdr_b) (float, hdr_a, hdr_a)
#else
#define REGISTER_E57_HDR 
#endif
#ifdef POINT_E57_WITH_RGB
#define REGISTER_E57_RGB (uint32_t, rgba, rgba)
#else
#define REGISTER_E57_RGB 
#endif
#ifdef POINT_E57_WITH_INTENSITY
#define REGISTER_E57_INTENSITY (float, intensity, intensity)
#else
#define REGISTER_E57_INTENSITY
#endif
#ifdef POINT_E57_WITH_LABEL
#define REGISTER_E57_LABEL (uint32_t, label, label)
#else
#define REGISTER_E57_LABEL
#endif

#define REGISTER_PCD_XYZ (float, x, x) (float, y, y) (float, z, z)
#ifdef POINT_PCD_WITH_NORMAL
#define REGISTER_PCD_NORMAL (float, normal_x, normal_x) (float, normal_y, normal_y) (float, normal_z, normal_z) (float, curvature, curvature)
#else
#define REGISTER_PCD_NORMAL
#endif
#ifdef POINT_PCD_WITH_RGB
#define REGISTER_PCD_RGB (uint32_t, rgba, rgba)
#else
#define REGISTER_PCD_RGB 
#endif
#ifdef POINT_PCD_WITH_INTENSITY
#define REGISTER_PCD_INTENSITY (float, intensity, intensity)
#else
#define REGISTER_PCD_INTENSITY
#endif
#ifdef POINT_PCD_WITH_LABEL
#define REGISTER_PCD_LABEL (uint32_t, label, label)
#else
#define REGISTER_PCD_LABEL
#endif
#ifdef POINT_PCD_WITH_LABEL
#define REGISTER_EXCHANGE_SEGMENT_LABEL (uint32_t, segmentLabel, segmentLabel)
#else
#define REGISTER_EXCHANGE_SEGMENT_LABEL
#endif

POINT_CLOUD_REGISTER_POINT_STRUCT(PointE57,
	REGISTER_E57_XYZ
	REGISTER_E57_HDR
	REGISTER_E57_RGB
	REGISTER_E57_INTENSITY
	REGISTER_E57_LABEL
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(PointE57, PointE57)

POINT_CLOUD_REGISTER_POINT_STRUCT(PointExchange,
	REGISTER_E57_XYZ
	REGISTER_PCD_NORMAL
	REGISTER_E57_HDR
	REGISTER_E57_RGB
	REGISTER_E57_INTENSITY
	REGISTER_E57_LABEL
	REGISTER_EXCHANGE_SEGMENT_LABEL
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(PointExchange, PointExchange)

POINT_CLOUD_REGISTER_POINT_STRUCT(PointPCD,
	REGISTER_PCD_XYZ
	REGISTER_PCD_NORMAL
	REGISTER_PCD_RGB
	REGISTER_PCD_INTENSITY
	REGISTER_PCD_LABEL
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(PointPCD, PointPCD)

POINT_CLOUD_REGISTER_POINT_STRUCT(PointNDF,
	(float, x, x) (float, y, y) (float, z, z)
	(float, intensity, intensity)
)
POINT_CLOUD_REGISTER_POINT_WRAPPER(PointNDF, PointNDF)
