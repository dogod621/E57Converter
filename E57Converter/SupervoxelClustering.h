#pragma once

#include <pcl/features/normal_3d.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/boost.h>



//DEBUG TODO REMOVE
#include <pcl/common/time.h>


namespace e57
{
	template <typename PointT>
	class Supervoxel
	{
	public:
		Supervoxel() :
			voxels_(new pcl::PointCloud<PointT>()),
			normals_(new pcl::PointCloud<pcl::Normal>())
		{  }

		typedef boost::shared_ptr<Supervoxel<PointT> > Ptr;
		typedef boost::shared_ptr<const Supervoxel<PointT> > ConstPtr;

		void getCentroidPoint(pcl::PointXYZRGBA &centroid_arg)
		{
			centroid_arg = centroid_;
		}

		void getCentroidPointNormal(pcl::PointNormal &normal_arg)
		{
			normal_arg.x = centroid_.x;
			normal_arg.y = centroid_.y;
			normal_arg.z = centroid_.z;
			normal_arg.normal_x = normal_.normal_x;
			normal_arg.normal_y = normal_.normal_y;
			normal_arg.normal_z = normal_.normal_z;
			normal_arg.curvature = normal_.curvature;
		}

		pcl::Normal normal_;
		pcl::PointXYZRGBA centroid_;
		typename pcl::PointCloud<PointT>::Ptr voxels_;
		typename pcl::PointCloud<pcl::Normal>::Ptr normals_;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	template <typename PointT>
	class PCL_EXPORTS SupervoxelClustering : public pcl::PCLBase<PointT>
	{
		class SupervoxelHelper;
		friend class SupervoxelHelper;
	public:
		class VoxelData
		{
		public:
			VoxelData() :
				xyz_(0.0f, 0.0f, 0.0f),
				rgb_(0.0f, 0.0f, 0.0f),
				normal_(0.0f, 0.0f, 0.0f, 0.0f),
				curvature_(0.0f),
				owner_(0)
			{}


			void getPoint(PointT &point_arg) const;

			void getNormal(pcl::Normal &normal_arg) const;

			Eigen::Vector3f xyz_;
			Eigen::Vector3f rgb_;
			Eigen::Vector4f normal_;
			float curvature_;
			float distance_;
			int idx_;
			SupervoxelHelper* owner_;

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		typedef pcl::octree::OctreePointCloudAdjacencyContainer<PointT, VoxelData> LeafContainerT;
		typedef std::vector <LeafContainerT*> LeafVectorT;

		typedef typename pcl::PointCloud<PointT> PointCloudT;
		typedef typename pcl::PointCloud<pcl::Normal> NormalCloudT;
		typedef typename pcl::octree::OctreePointCloudAdjacency<PointT, LeafContainerT> OctreeAdjacencyT;
		typedef typename pcl::octree::OctreePointCloudSearch <PointT> OctreeSearchT;
		typedef typename pcl::search::KdTree<PointT> KdTreeT;
		typedef boost::shared_ptr<std::vector<int> > IndicesPtr;

		using pcl::PCLBase <PointT>::initCompute;
		using pcl::PCLBase <PointT>::deinitCompute;
		using pcl::PCLBase <PointT>::input_;

		typedef boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, uint32_t, float> VoxelAdjacencyList;
		typedef VoxelAdjacencyList::vertex_descriptor VoxelID;
		typedef VoxelAdjacencyList::edge_descriptor EdgeID;

	public:
		SupervoxelClustering(float voxel_resolution, float seed_resolution);

		virtual ~SupervoxelClustering();

		void setVoxelResolution(float resolution);

		float getVoxelResolution() const;

		void setSeedResolution(float seed_resolution);

		float getSeedResolution() const;

		void setColorImportance(float val);

		void setSpatialImportance(float val);

		void setNormalImportance(float val);

		void setUseSingleCameraTransform(bool val);

		virtual void extract(std::map<uint32_t, typename Supervoxel<PointT>::Ptr > &supervoxel_clusters);

		virtual void setInputCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud);

		virtual void setNormalCloud(typename NormalCloudT::ConstPtr normal_cloud);

		virtual void refineSupervoxels(int num_itr, std::map<uint32_t, typename Supervoxel<PointT>::Ptr > &supervoxel_clusters);

		typename pcl::PointCloud<PointT>::Ptr getVoxelCentroidCloud() const;

		typename pcl::PointCloud<pcl::PointXYZL>::Ptr getLabeledCloud() const;

		pcl::PointCloud<pcl::PointXYZL>::Ptr getLabeledVoxelCloud() const;

		void getSupervoxelAdjacencyList(VoxelAdjacencyList &adjacency_list_arg) const;

		void getSupervoxelAdjacency(std::multimap<uint32_t, uint32_t> &label_adjacency) const;

		static pcl::PointCloud<pcl::PointNormal>::Ptr makeSupervoxelNormalCloud(std::map<uint32_t, typename Supervoxel<PointT>::Ptr > &supervoxel_clusters);

		int getMaxLabel() const;

	private:
		virtual bool prepareForSegmentation();

		void selectInitialSupervoxelSeeds(std::vector<int> &seed_indices);

		void createSupervoxelHelpers(std::vector<int> &seed_indices);

		void expandSupervoxels(int depth);

		void computeVoxelData();

		void reseedSupervoxels();

		void makeSupervoxels(std::map<uint32_t, typename Supervoxel<PointT>::Ptr > &supervoxel_clusters);

		float resolution_;

		float seed_resolution_;

		float voxelDataDistance(const VoxelData &v1, const VoxelData &v2) const;

		void transformFunction(PointT &p);

		typename pcl::search::KdTree<PointT>::Ptr voxel_kdtree_;

		typename OctreeAdjacencyT::Ptr adjacency_octree_;

		typename PointCloudT::Ptr voxel_centroid_cloud_;

		typename NormalCloudT::ConstPtr input_normals_;

		float color_importance_;
		float spatial_importance_;
		float normal_importance_;

		bool use_single_camera_transform_;
		bool use_default_transform_behaviour_;


		class SupervoxelHelper
		{
		public:
			struct compareLeaves
			{
				bool operator() (LeafContainerT* const &left, LeafContainerT* const &right) const
				{
					const VoxelData& leaf_data_left = left->getData();
					const VoxelData& leaf_data_right = right->getData();
					return leaf_data_left.idx_ < leaf_data_right.idx_;
				}
			};
			typedef std::set<LeafContainerT*, typename SupervoxelHelper::compareLeaves> LeafSetT;
			typedef typename LeafSetT::iterator iterator;
			typedef typename LeafSetT::const_iterator const_iterator;

			SupervoxelHelper(uint32_t label, SupervoxelClustering* parent_arg) :
				label_(label),
				parent_(parent_arg)
			{ }

			void addLeaf(LeafContainerT* leaf_arg);

			void removeLeaf(LeafContainerT* leaf_arg);

			void removeAllLeaves();

			void expand();

			void refineNormals();

			void updateCentroid();

			void getVoxels(typename pcl::PointCloud<PointT>::Ptr &voxels) const;

			void getNormals(typename pcl::PointCloud<pcl::Normal>::Ptr &normals) const;

			typedef float (SupervoxelClustering::*DistFuncPtr)(const VoxelData &v1, const VoxelData &v2);

			uint32_t getLabel() const
			{
				return label_;
			}

			Eigen::Vector4f getNormal() const
			{
				return centroid_.normal_;
			}

			Eigen::Vector3f getRGB() const
			{
				return centroid_.rgb_;
			}

			Eigen::Vector3f getXYZ() const
			{
				return centroid_.xyz_;
			}

			void getXYZ(float &x, float &y, float &z) const
			{
				x = centroid_.xyz_[0]; y = centroid_.xyz_[1]; z = centroid_.xyz_[2];
			}

			void getRGB(uint32_t &rgba) const
			{
				rgba = static_cast<uint32_t>(centroid_.rgb_[0]) << 16 |
					static_cast<uint32_t>(centroid_.rgb_[1]) << 8 |
					static_cast<uint32_t>(centroid_.rgb_[2]);
			}

			void getNormal(pcl::Normal &normal_arg) const
			{
				normal_arg.normal_x = centroid_.normal_[0];
				normal_arg.normal_y = centroid_.normal_[1];
				normal_arg.normal_z = centroid_.normal_[2];
				normal_arg.curvature = centroid_.curvature_;
			}

			void getNeighborLabels(std::set<uint32_t> &neighbor_labels) const;

			VoxelData getCentroid() const
			{
				return centroid_;
			}

			size_t
				size() const { return leaves_.size(); }
		private:
			//Stores leaves
			LeafSetT leaves_;
			uint32_t label_;
			VoxelData centroid_;
			SupervoxelClustering* parent_;
		public:
			//Type VoxelData may have fixed-size Eigen objects inside
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		//Make boost::ptr_list can access the private class SupervoxelHelper
		friend void boost::checked_delete<>(const typename e57::SupervoxelClustering<PointT>::SupervoxelHelper *);

		typedef boost::ptr_list<SupervoxelHelper> HelperListT;
		HelperListT supervoxel_helpers_;

		//TODO DEBUG REMOVE
		pcl::StopWatch timer_;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	};
}

#include "SupervoxelClustering.hpp"
