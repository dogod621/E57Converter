#include "E57AlbedoEstimation.h"

namespace e57
{
	inline bool AlbedoEstimation::ComputePointAlbedo(const pcl::PointCloud<PointE57>& cloud, const pcl::PointCloud<pcl::Normal>& cloudNormal, const std::vector<int> &indices, PointPCD& point)
	{
#ifdef POINT_E57_WITH_LABEL
#ifdef POINT_E57_WITH_INTENSITY
#ifdef POINT_PCD_WITH_INTENSITY
		point.intensity = 0.0f;
		float radius = search_radius_;
		float sumWeight = 0.0f;
		float cutFalloff = std::numeric_limits<float>::epsilon() * 100.0f;
		Eigen::Vector3f pointPosition(point.x, point.y, point.z);
		Eigen::Vector3f pointNormal(point.normal_x, point.normal_y, point.normal_z);
		for (std::size_t idx = 0; idx < indices.size(); ++idx)
		{
			const Eigen::Vector3f nNormal(cloudNormal[idx].normal_x, cloudNormal[idx].normal_y, cloudNormal[idx].normal_z);
			if (std::abs(nNormal.norm() - 1.0f) > 0.05f)
			{
				PCL_WARN("[e57::%s::ComputePointAlbedo] Search point normal is not valid, ignore.\n", "AlbedoEstimation");
			}
			else
			{
				const PointE57& nPoint = cloud[idx];
				Eigen::Vector3f nPointPosition(nPoint.x, nPoint.y, nPoint.z);
				const ScanInfo& nScanInfo = scanInfo[nPoint.label];
				switch (nScanInfo.scanner)
				{
				case Scanner::BLK360:
				{
					Eigen::Vector3f measureVec = nScanInfo.position_float - nPointPosition;
					float nDistance = measureVec.norm();
					float nDotNL = std::abs(nNormal.dot(measureVec / nDistance));
					float nDotNN = std::abs(nNormal.dot(pointNormal));

					// Ref - BLK 360 Spec - laser wavelength & Beam divergence : https://lasers.leica-geosystems.com/global/sites/lasers.leica-geosystems.com.global/files/leica_media/product_documents/blk/853811_leica_blk360_um_v2.0.0_en.pdf
					// Ref - Gaussian beam : https://en.wikipedia.org/wiki/Gaussian_beam
					// Ref - Beam divergence to Beam waist(w0) : http://www2.nsysu.edu.tw/optics/laser/angle.htm
					float temp = nDistance / 26.2854504782;
					float nGaussianBeamFalloff = 1.0f / (1 + temp * temp);

					float nFalloff = nDotNL * nGaussianBeamFalloff;
					if (nFalloff > cutFalloff)
					{
						float weight = std::powf(std::abs(radius - (pointPosition - nPointPosition).norm()) / radius, distInterParm) * std::powf(nDotNN, angleInterParm)* std::powf(nDotNL, frontInterParm);
						point.intensity += nPoint.intensity * weight / nFalloff;
						sumWeight += weight;
					}
				}
				break;

				default:
					PCL_WARN("[e57::%s::ComputePointAlbedo] Scan data Scanner type is not support, ignore.\n", "AlbedoEstimation");
					break;
				}
			}
			
		}
		if (sumWeight > 0.0f)
		{
			point.intensity /= sumWeight;
			if (point.intensity < 0)
			{
				PCL_WARN("[e57::%s::ComputePointAlbedo] Final intensity is negative!!?.\n", "AlbedoEstimation");
				return false;
			}
		}
		else
		{
			PCL_WARN("[e57::%s::ComputePointAlbedo] No valid neighbor scan data found.\n", "AlbedoEstimation");
			return false;
		}
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

		if (input_->is_dense)
		{
			for (std::size_t idx = 0; idx < indices_->size(); ++idx)
			{
				PointPCD& point = output.points[idx];
				if (std::abs(Eigen::Vector3f(point.normal_x, point.normal_y, point.normal_z).norm() - 1.0f) > 0.05f)
				{
					PCL_WARN("[e57::%s::computeFeature] Point normal is not valid!!?.\n", "AlbedoEstimation");
					point.intensity = 0.0f;
				}
				else
				{
					if (this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) != 0)
					{
						if (ComputePointAlbedo(*surface_, *searchSurfaceNormal, nn_indices, point))
						{
							continue;
						}
						else
						{
							PCL_WARN("[e57::%s::computeFeature] ComputePointAlbedo failed.\n", "AlbedoEstimation");
							point.intensity = 0.0f;
						}
					}
					else
					{
						PCL_WARN("[e57::%s::computeFeature] No neighbour point is found!!?.\n", "AlbedoEstimation");
						point.intensity = 0.0f;
					}
				}
			}
		}
		else
		{
			for (std::size_t idx = 0; idx < indices_->size(); ++idx)
			{
				PointPCD& point = output.points[idx];
				if (pcl::isFinite((*input_)[(*indices_)[idx]]))
				{
					if (std::abs(Eigen::Vector3f(point.normal_x, point.normal_y, point.normal_z).norm() - 1.0f) > 0.05f)
					{
						PCL_WARN("[e57::%s::computeFeature] Point normal is not valid!!?.\n", "AlbedoEstimation");
						point.intensity = 0.0f;
					}
					else
					{
						if (this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) != 0)
						{
							if (ComputePointAlbedo(*surface_, *searchSurfaceNormal, nn_indices, point))
							{
								continue;
							}
							else
							{
								PCL_WARN("[e57::%s::computeFeature] ComputePointAlbedo failed.\n", "AlbedoEstimation");
								point.intensity = 0.0f;
							}
						}
						else
						{
							PCL_WARN("[e57::%s::computeFeature] No neighbour point is found!!?.\n", "AlbedoEstimation");
							point.intensity = 0.0f;
						}
					}
				}
				else
				{
					PCL_WARN("[e57::%s::computeFeature] Input point contain non finite value!!?.\n", "AlbedoEstimation");
					point.intensity = 0.0f;
				}
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

		if (input_->is_dense)
		{
#ifdef _OPENMP
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
#endif
			for (int idx = 0; idx < static_cast<int> (indices_->size()); ++idx)
			{
				PointPCD& point = output.points[idx];
				if (std::abs(Eigen::Vector3f(point.normal_x, point.normal_y, point.normal_z).norm() - 1.0f) > 0.05f)
				{
					PCL_WARN("[e57::%s::computeFeature] Point normal is not valid!!?.\n", "AlbedoEstimation");
					point.intensity = 0.0f;
				}
				else
				{
					if (this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) != 0)
					{
						if (ComputePointAlbedo(*surface_, *searchSurfaceNormal, nn_indices, point))
						{
							continue;
						}
						else
						{
							PCL_WARN("[e57::%s::computeFeature] ComputePointAlbedo failed.\n", "AlbedoEstimation");
							point.intensity = 0.0f;
						}
					}
					else
					{
						PCL_WARN("[e57::%s::computeFeature] No neighbour point is found!!?.\n", "AlbedoEstimation");
						point.intensity = 0.0f;
					}
				}
			}
		}
		else
		{
#ifdef _OPENMP
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
#endif
			for (int idx = 0; idx < static_cast<int> (indices_->size()); ++idx)
			{
				PointPCD& point = output.points[idx];
				if (pcl::isFinite((*input_)[(*indices_)[idx]]))
				{
					if (std::abs(Eigen::Vector3f(point.normal_x, point.normal_y, point.normal_z).norm() - 1.0f) > 0.05f)
					{
						PCL_WARN("[e57::%s::computeFeature] Point normal is not valid!!?.\n", "AlbedoEstimation");
						point.intensity = 0.0f;
					}
					else
					{
						if (this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) != 0)
						{
							if (ComputePointAlbedo(*surface_, *searchSurfaceNormal, nn_indices, point))
							{
								continue;
							}
							else
							{
								PCL_WARN("[e57::%s::computeFeature] ComputePointAlbedo failed.\n", "AlbedoEstimation");
								point.intensity = 0.0f;
							}
						}
						else
						{
							PCL_WARN("[e57::%s::computeFeature] No neighbour point is found!!?.\n", "AlbedoEstimation");
							point.intensity = 0.0f;
						}
					}
				}
				else
				{
					PCL_WARN("[e57::%s::computeFeature] Input point contain non finite value!!?.\n", "AlbedoEstimation");
					point.intensity = 0.0f;
				}
			}
		}
#endif
#endif
#endif
	}
}