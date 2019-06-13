#pragma once

#include <vector>

#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>

#include "Common.h"
#include "E57Utils.h"

namespace e57
{
	struct ScannLaserInfo
	{
		Eigen::Vector3d incidentDirection;
		Eigen::Vector3d reflectedDirection;
		double intensity;
		Eigen::Vector3d hitPosition;
		Eigen::Vector3d hitNormal;
		Eigen::Vector3d hitTangent;
		Eigen::Vector3d hitBitangent;
		double hitDistance;
		double weight;
		double beamFalloff;
	};

	class AlbedoEstimation : public pcl::Feature<PointE57_Normal, PointPCD>
	{
	public:
		typedef boost::shared_ptr<AlbedoEstimation> Ptr;
		typedef boost::shared_ptr<const AlbedoEstimation> ConstPtr;
		using pcl::Feature<PointE57_Normal, PointPCD>::feature_name_;
		using pcl::Feature<PointE57_Normal, PointPCD>::getClassName;
		using pcl::Feature<PointE57_Normal, PointPCD>::indices_;
		using pcl::Feature<PointE57_Normal, PointPCD>::input_;
		using pcl::Feature<PointE57_Normal, PointPCD>::surface_;
		using pcl::Feature<PointE57_Normal, PointPCD>::k_;
		using pcl::Feature<PointE57_Normal, PointPCD>::search_radius_;
		using pcl::Feature<PointE57_Normal, PointPCD>::search_parameter_;

		typedef typename pcl::Feature<PointE57_Normal, PointPCD>::PointCloudOut PointCloudOut;
		typedef typename pcl::Feature<PointE57_Normal, PointPCD>::PointCloudConstPtr PointCloudConstPtr;

	public:
		AlbedoEstimation(const std::vector<ScanInfo>& scanInfos, 
			const LinearSolver linearSolver = LinearSolver::EIGEN_SVD, const double distInterParm = 10.0, const double angleInterParm = 20.0, const double frontInterParm = 5.0, const double cutFalloff = 0.33, const double cutGrazing = 0.86602540378)
			: scanInfos(scanInfos), linearSolver(linearSolver), distInterParm(distInterParm), angleInterParm(angleInterParm), frontInterParm(frontInterParm), cutFalloff(cutFalloff), cutGrazing(cutGrazing)
		{
			feature_name_ = "AlbedoEstimation";
		};

		virtual ~AlbedoEstimation() {}

		virtual inline void setInputCloud(const PointCloudConstPtr &cloud)
		{
			input_ = cloud;
		}

		inline bool CollectScannLaserInfo(const pcl::PointCloud<PointE57_Normal>& cloud, const std::size_t k, const std::vector<int>& indices, const std::vector<float>& distance, const PointE57_Normal& inPoint, std::vector<ScannLaserInfo>& scannLaserInfos);

		inline bool ComputePointAlbedo(const std::vector<ScannLaserInfo>& scannLaserInfos, const PointE57_Normal& inPoint, PointPCD& outPoint);


	protected:
		LinearSolver linearSolver;
		double distInterParm;
		double angleInterParm;
		double frontInterParm;
		double cutFalloff;
		double cutGrazing;

		std::vector<ScanInfo> scanInfos;

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

		AlbedoEstimationOMP(const std::vector<ScanInfo>& scanInfos, 
			const LinearSolver linearSolver = LinearSolver::EIGEN_SVD, const double distInterParm = 10.0, const double angleInterParm = 20.0, const double frontInterParm = 5.0, const double cutFalloff = 0.33, const double cutGrazing = 0.86602540378,
			unsigned int nr_threads = 0)
			: AlbedoEstimation(scanInfos, linearSolver, distInterParm, angleInterParm, frontInterParm, cutFalloff, cutGrazing)
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