#include "E57AlbedoEstimation.h"

namespace e57
{
	inline bool AlbedoEstimation::CollectScannLaserInfo(const pcl::PointCloud<PointE57>& cloud, const pcl::PointCloud<pcl::Normal>& cloudNormal, const std::size_t k, const std::vector<int>& indices, const std::vector<float>& distance, Eigen::Vector3d& centerNormal, std::vector<ScannLaserInfo>& scannLaserInfos)
	{
#ifdef POINT_E57_WITH_LABEL
#ifdef POINT_E57_WITH_INTENSITY
		if (std::abs(centerNormal.norm() - 1.0f) > 0.05f)
		{
			PCL_WARN("[e57::%s::CollectScannLaserInfo] centerNormal is not valid!!?.\n", "AlbedoEstimation");
			return false;
		}

		double radius = search_radius_;
		scannLaserInfos.clear();
		scannLaserInfos.reserve(k);
		Eigen::Vector3d tempVec(1.0, 1.0, 1.0);
		tempVec /= tempVec.norm();
		Eigen::Vector3d centerTangent = centerNormal.cross(tempVec);
		double centerTangentNorm = centerTangent.norm();
		if (!(centerTangentNorm > 0.0))
		{
			PCL_WARN("[e57::%s::CollectScannLaserInfo] centerTangent is not valid!!?.\n", "AlbedoEstimation");
			return false;
		}
		centerTangent /= centerTangentNorm;

		//
		for (std::size_t idx = 0; idx < k; ++idx)
		{
			int px = indices[idx];
			double d = distance[idx];
			ScannLaserInfo scannLaserInfo;

			scannLaserInfo.hitNormal = Eigen::Vector3d(cloudNormal[px].normal_x, cloudNormal[px].normal_y, cloudNormal[px].normal_z);
			if (std::abs(scannLaserInfo.hitNormal.norm() - 1.0) > 0.05)
			{
				PCL_WARN("[e57::%s::CollectScannLaserInfo] scannLaserInfo.hitNormal is not valid, ignore.\n", "AlbedoEstimation");
			}
			else
			{
				double dotNN = scannLaserInfo.hitNormal.dot(centerNormal);
				if (dotNN > cutGrazing)
				{
					const PointE57& scanPoint = cloud[px];
					const ScanInfo& scanScanInfo = scanInfos[scanPoint.label];
					scannLaserInfo.hitPosition = Eigen::Vector3d(scanPoint.x, scanPoint.y, scanPoint.z);
					switch (scanScanInfo.scanner)
					{
					case Scanner::BLK360:
					{
						scannLaserInfo.incidentDirection = scanScanInfo.position - scannLaserInfo.hitPosition;
						scannLaserInfo.hitDistance = scannLaserInfo.incidentDirection.norm();
						scannLaserInfo.incidentDirection /= scannLaserInfo.hitDistance;
						if (scannLaserInfo.incidentDirection.dot(scannLaserInfo.hitNormal) < 0)
							scannLaserInfo.incidentDirection *= -1.0;
						scannLaserInfo.reflectedDirection = scannLaserInfo.incidentDirection; // BLK360 

						// Ref - BLK 360 Spec - laser wavelength & Beam divergence : https://lasers.leica-geosystems.com/global/sites/lasers.leica-geosystems.com.global/files/leica_media/product_documents/blk/853811_leica_blk360_um_v2.0.0_en.pdf
						// Ref - Gaussian beam : https://en.wikipedia.org/wiki/Gaussian_beam
						// Ref - Beam divergence to Beam waist(w0) : http://www2.nsysu.edu.tw/optics/laser/angle.htm
						double temp = scannLaserInfo.hitDistance / 26.2854504782;
						scannLaserInfo.beamFalloff = 1.0f / (1 + temp * temp);
						if ((scannLaserInfo.beamFalloff > cutFalloff))
						{
							scannLaserInfo.hitTangent = scannLaserInfo.hitNormal.cross(tempVec);
							double hitTangentNorm = scannLaserInfo.hitTangent.norm();
							if (hitTangentNorm > 0.0)
							{
								scannLaserInfo.hitTangent /= hitTangentNorm;
								scannLaserInfo.weight = std::pow(std::abs(radius - d) / radius, distInterParm) * std::pow(dotNN, angleInterParm);
								scannLaserInfo.intensity = (double)scanPoint.intensity;
								scannLaserInfos.push_back(scannLaserInfo);
							}
						}
					}
					break;

					default:
						PCL_WARN("[e57::%s::ComputePointAlbedo] Scan data Scanner type is not support, ignore.\n", "AlbedoEstimation");
						break;
					}
				}
			}
		}
#endif
#endif
		return scannLaserInfos.size() > 0;
	}

	inline bool AlbedoEstimation::ComputePointAlbedo(const std::vector<ScannLaserInfo>& scannLaserInfos, const PointE57& centerPoint, const Eigen::Vector3d& centerNormal, PointPCD& outPoint)
	{
#ifdef POINT_PCD_WITH_INTENSITY
		Eigen::MatrixXf A;
		Eigen::MatrixXf B;

		A = Eigen::MatrixXf(scannLaserInfos.size() * 2, 3);
		B = Eigen::MatrixXf(scannLaserInfos.size() * 2, 1);

		std::size_t shifter = 0;
		for (std::vector<ScannLaserInfo>::const_iterator it = scannLaserInfos.begin(); it != scannLaserInfos.end(); ++it)
		{
			A(shifter, 0) = it->weight * it->incidentDirection.x();
			A(shifter, 1) = it->weight * it->incidentDirection.y();
			A(shifter, 2) = it->weight * it->incidentDirection.z();
			B(shifter, 0) = it->weight * (it->intensity / it->beamFalloff);

			A(shifter + 1, 0) = it->weight * it->hitTangent.x();
			A(shifter + 1, 1) = it->weight * it->hitTangent.y();
			A(shifter + 1, 2) = it->weight * it->hitTangent.z();
			B(shifter + 1, 0) = 0.0;

			shifter += 2;
		}

		Eigen::MatrixXf X;
		switch (linearSolver)
		{
		case LinearSolver::EIGEN_QR:
		{
			X = A.colPivHouseholderQr().solve(B);
		}
		break;
		case LinearSolver::EIGEN_SVD:
		{
			X = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
		}
		break;
		case LinearSolver::EIGEN_NE:
		{
			Eigen::MatrixXf localAT = A.transpose();
			X = (localAT * A).ldlt().solve(localAT * B);
		}
		break;
		default:
		{
			throw pcl::PCLException("LinearSolver is not supported.");
		}
		break;
		}

		//
		Eigen::Vector3d xVec(X(0, 0), X(1, 0), X(2, 0));
		if (std::isfinite(xVec.x()) && std::isfinite(xVec.y()) && std::isfinite(xVec.z()))
		{
			double xVecNorm = xVec.norm();
			if (xVecNorm > 0.0)
			{
				outPoint.intensity = xVecNorm;
				xVec /= xVecNorm;
				outPoint.normal_x = xVec.x();
				outPoint.normal_y = xVec.y();
				outPoint.normal_z = xVec.z();
				return true;
			}
			else
			{
				PCL_WARN("[e57::%s::ComputePointAlbedo] LinearSolver solve zero norm.\n", "AlbedoEstimation");
				return false;
			}
		}
		else
		{
			PCL_WARN("[e57::%s::ComputePointAlbedo] LinearSolver solve non finite value.\n", "AlbedoEstimation");
			return false;
		}
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
				const PointE57& inPoint = (*input_)[(*indices_)[idx]];
				PointPCD& outPoint = output.points[idx];

				std::vector<ScannLaserInfo> scannLaserInfos;
				Eigen::Vector3d centerNormal(outPoint.normal_x, outPoint.normal_y, outPoint.normal_z);
				if (CollectScannLaserInfo(*surface_, *searchSurfaceNormal,
					this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists),
					nn_indices, nn_dists,
					centerNormal, scannLaserInfos))
				{
					if (!ComputePointAlbedo(scannLaserInfos, inPoint, centerNormal, outPoint))
					{
						PCL_WARN("[e57::%s::computeFeature] ComputePointAlbedo failed.\n", "AlbedoEstimation");
						outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
						output.is_dense = false;
					}
				}
				else
				{
					PCL_WARN("[e57::%s::computeFeature] CollectScannLaserInfo failed.\n", "AlbedoEstimation");
					outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
					output.is_dense = false;
				}
			}
		}
		else
		{
			for (std::size_t idx = 0; idx < indices_->size(); ++idx)
			{
				const PointE57& inPoint = (*input_)[(*indices_)[idx]];
				PointPCD& outPoint = output.points[idx];
				if (pcl::isFinite(inPoint))
				{
					std::vector<ScannLaserInfo> scannLaserInfos;
					Eigen::Vector3d centerNormal(outPoint.normal_x, outPoint.normal_y, outPoint.normal_z);
					if (CollectScannLaserInfo(*surface_, *searchSurfaceNormal,
						this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists),
						nn_indices, nn_dists,
						centerNormal, scannLaserInfos))
					{
						if (!ComputePointAlbedo(scannLaserInfos, inPoint, centerNormal, outPoint))
						{
							PCL_WARN("[e57::%s::computeFeature] ComputePointAlbedo failed.\n", "AlbedoEstimation");
							outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
							output.is_dense = false;
						}
					}
					else
					{
						PCL_WARN("[e57::%s::computeFeature] CollectScannLaserInfo failed.\n", "AlbedoEstimation");
						outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
						output.is_dense = false;
					}
				}
				else
				{
					PCL_WARN("[e57::%s::computeFeature] Input point contain non finite value!!?.\n", "AlbedoEstimation");
					outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
					output.is_dense = false;
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

		output.is_dense = true;
		if (input_->is_dense)
		{
#ifdef _OPENMP
#pragma omp parallel for shared (output) private (nn_indices, nn_dists) num_threads(threads_)
#endif
			for (int idx = 0; idx < static_cast<int> (indices_->size()); ++idx)
			{
				const PointE57& inPoint = (*input_)[(*indices_)[idx]];
				PointPCD& outPoint = output.points[idx];

				std::vector<ScannLaserInfo> scannLaserInfos;
				Eigen::Vector3d centerNormal(outPoint.normal_x, outPoint.normal_y, outPoint.normal_z);
				if (CollectScannLaserInfo(*surface_, *searchSurfaceNormal,
					this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists),
					nn_indices, nn_dists,
					centerNormal, scannLaserInfos))
				{
					if (!ComputePointAlbedo(scannLaserInfos, inPoint, centerNormal, outPoint))
					{
						PCL_WARN("[e57::%s::computeFeature] ComputePointAlbedo failed.\n", "AlbedoEstimationOMP");
						outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
						output.is_dense = false;
					}
				}
				else
				{
					PCL_WARN("[e57::%s::computeFeature] CollectScannLaserInfo failed.\n", "AlbedoEstimationOMP");
					outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
					output.is_dense = false;
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
				const PointE57& inPoint = (*input_)[(*indices_)[idx]];
				PointPCD& outPoint = output.points[idx];
				if (pcl::isFinite(inPoint))
				{
					std::vector<ScannLaserInfo> scannLaserInfos;
					Eigen::Vector3d centerNormal(outPoint.normal_x, outPoint.normal_y, outPoint.normal_z);
					if (CollectScannLaserInfo(*surface_, *searchSurfaceNormal,
						this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists),
						nn_indices, nn_dists,
						centerNormal, scannLaserInfos))
					{
						if (!ComputePointAlbedo(scannLaserInfos, inPoint, centerNormal, outPoint))
						{
							PCL_WARN("[e57::%s::computeFeature] ComputePointAlbedo failed.\n", "AlbedoEstimationOMP");
							outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
							output.is_dense = false;
						}
					}
					else
					{
						PCL_WARN("[e57::%s::computeFeature] CollectScannLaserInfo failed.\n", "AlbedoEstimationOMP");
						outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
						output.is_dense = false;
					}
				}
				else
				{
					PCL_WARN("[e57::%s::computeFeature] Input point contain non finite value!!?.\n", "AlbedoEstimationOMP");
					outPoint.intensity = std::numeric_limits<float>::quiet_NaN();
					output.is_dense = false;
				}
			}
		}
#endif
#endif
#endif
	}
}