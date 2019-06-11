#pragma once

#include <vector>

#include "Common.h"
#include "E57Utils.h"

#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>

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

		inline bool ComputePointAlbedo(const pcl::PointCloud<PointE57> &cloud, const std::vector<int> &indices, float& albedo)
		{
#ifdef POINT_E57_WITH_LABEL
#ifdef POINT_E57_WITH_INTENSITY
#ifdef POINT_PCD_WITH_INTENSITY
			/*std::vector<Eigen::Vector3d> viewVector;

			for (std::size_t idx = 0; idx < indices->size(); ++idx)
			{

			}

			return true;

			switch (scanner)
			{
			case Scanner::BLK360:
			{

			}
			break;

			default:
				PCL_WARN("[e57::%s::ComputePointAlbedo] Scanner type is not support, ignore AlbedoEstimation.\n", "AlbedoEstimation");
			}*/
#endif
#endif
#endif
			return true;
		}



	protected:
		std::vector<ScanInfo> scanInfo;

		void SetScanInfo(const std::vector<ScanInfo>& scanInfo_)
		{
			scanInfo = scanInfo_;
		}

		void computeFeature(PointCloudOut &output)
		{
#ifdef POINT_E57_WITH_LABEL
#ifdef POINT_E57_WITH_INTENSITY
#ifdef POINT_PCD_WITH_INTENSITY
			std::vector<int> nn_indices(k_);
			std::vector<float> nn_dists(k_);
			output.is_dense = true;

			if (input_->is_dense)
			{
				for (std::size_t idx = 0; idx < indices_->size(); ++idx)
				{
					if (this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) != 0)
					{
						if (ComputePointAlbedo(*surface_, nn_indices, output.points[idx].intensity))
						{
							continue;
						}
					}
					output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN();
					output.is_dense = false;
				}
			}
			else
			{
				for (std::size_t idx = 0; idx < indices_->size(); ++idx)
				{
					if (pcl::isFinite((*input_)[(*indices_)[idx]]))
					{
						if (this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) != 0)
						{
							if (ComputePointAlbedo(*surface_, nn_indices, output.points[idx].intensity))
							{
								continue;
							}
						}
					}
					output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN();
					output.is_dense = false;
				}
			}
#endif
#endif
#endif
		}

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#include <E57AlbedoEstimation.hpp>