#include "E57AlbedoEstimation.h"

namespace e57
{
	inline bool AlbedoEstimation::ComputePointAlbedo(const pcl::PointCloud<PointE57> &cloud, const std::vector<int> &indices, float& albedo)
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

	void AlbedoEstimation::computeFeature(PointCloudOut &output)
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
				output.points[idx].intensity = std::numeric_limits<float>::quiet_NaN();
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
				output.points[idx].intensity = std::numeric_limits<float>::quiet_NaN();
				output.is_dense = false;
			}
		}
#endif
#endif
#endif
	}

	void AlbedoEstimationOMP::SetNumberOfThreads(unsigned int nr_threads)
	{
		if (nr_threads == 0)
#ifdef _OPENMP
			threads_ = omp_get_num_procs();
#else
			threads_ = 1;
#endif
		else
			threads_ = nr_threads;
	}

	void AlbedoEstimationOMP::computeFeature(PointCloudOut &output)
	{
#ifdef POINT_E57_WITH_LABEL
#ifdef POINT_E57_WITH_INTENSITY
#ifdef POINT_PCD_WITH_INTENSITY
		std::vector<int> nn_indices(k_);
		std::vector<float> nn_dists(k_);

		output.is_dense = true;
		if (input_->is_dense)
		{
#ifdef _OPENMP
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
#endif
			for (int idx = 0; idx < static_cast<int> (indices_->size()); ++idx)
			{
				if (this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) != 0)
				{
					if (ComputePointAlbedo(*surface_, nn_indices, output.points[idx].intensity))
					{
						continue;
					}
				}
				output.points[idx].intensity = std::numeric_limits<float>::quiet_NaN();
				output.is_dense = false;
			}
		}
		else
		{
#ifdef _OPENMP
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
#endif
			for (int idx = 0; idx < static_cast<int> (indices_->size()); ++idx)
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
				output.points[idx].intensity = std::numeric_limits<float>::quiet_NaN();
				output.is_dense = false;
			}
		}
#endif
#endif
#endif
	}
}