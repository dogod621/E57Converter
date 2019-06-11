#pragma once

#include <vector>

#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>

#include "Common.h"
#include "E57Utils.h"

namespace e57
{
	class AlbedoEstimation : public pcl::Feature<PointE57, PointPCD>
	{
	public:
		typedef boost::shared_ptr<AlbedoEstimation> Ptr;
		typedef boost::shared_ptr<const AlbedoEstimation> ConstPtr;
		using pcl::Feature<PointE57, PointPCD>::feature_name_;
		using pcl::Feature<PointE57, PointPCD>::getClassName;
		using pcl::Feature<PointE57, PointPCD>::indices_;
		using pcl::Feature<PointE57, PointPCD>::input_;
		using pcl::Feature<PointE57, PointPCD>::surface_;
		using pcl::Feature<PointE57, PointPCD>::k_;
		using pcl::Feature<PointE57, PointPCD>::search_radius_;
		using pcl::Feature<PointE57, PointPCD>::search_parameter_;

		typedef typename pcl::Feature<PointE57, PointPCD>::PointCloudOut PointCloudOut;
		typedef typename pcl::Feature<PointE57, PointPCD>::PointCloudConstPtr PointCloudConstPtr;

	public:
		AlbedoEstimation()
		{
			feature_name_ = "AlbedoEstimation";
		};

		virtual ~AlbedoEstimation() {}

		virtual inline void setInputCloud(const PointCloudConstPtr &cloud)
		{
			input_ = cloud;
		}

		inline bool ComputePointAlbedo(const pcl::PointCloud<PointE57> &cloud, const std::vector<int> &indices, float& albedo);


	protected:
		std::vector<ScanInfo> scanInfo;

		void SetScanInfo(const std::vector<ScanInfo>& scanInfo_)
		{
			scanInfo = scanInfo_;
		}

		void computeFeature(PointCloudOut &output);

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};


	class AlbedoEstimationOMP : public AlbedoEstimation
	{
	public:
		typedef boost::shared_ptr<AlbedoEstimationOMP> Ptr;
		typedef boost::shared_ptr<const AlbedoEstimationOMP> ConstPtr;
		using AlbedoEstimation::feature_name_;
		using AlbedoEstimation::getClassName;
		using AlbedoEstimation::indices_;
		using AlbedoEstimation::input_;
		using AlbedoEstimation::surface_;
		using AlbedoEstimation::k_;
		using AlbedoEstimation::search_radius_;
		using AlbedoEstimation::search_parameter_;
		
		typedef typename AlbedoEstimation::PointCloudOut PointCloudOut;

		AlbedoEstimationOMP(unsigned int nr_threads = 0)
		{
			feature_name_ = "AlbedoEstimationOMP";

			SetNumberOfThreads(nr_threads);
		}

		void SetNumberOfThreads(unsigned int nr_threads = 0);

	protected:
		unsigned int threads_;

	private:
		void computeFeature(PointCloudOut &output);
	};
}