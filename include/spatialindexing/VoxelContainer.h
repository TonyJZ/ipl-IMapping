#pragma once
#include <Eigen/dense>
//#include <pcl/octree/octree_key.h>
#include "spatialindexing/iploctree_key.h"

namespace ipl
{
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief voxel container class that can serve as a base to construct own leaf node container classes.
	*  \
	*/
	enum VOXEL_OCCUPYFLAG
	{
		VOXEL_NULL = 0,
		VOXEL_OCCUPIED = 1,
		VOXEL_INNER = 2
	};

	struct  VoxelFeature
	{//voxel features
		Eigen::Vector3f vRefPos; //voxel centre pos
		int		ptsNum;

		float	fProjDens;  //水平投影点密度

		/** \brief Stores the mean value (center of mass) of the cloud */
		Eigen::Vector3f centroid;  //点云的重心 

	   /** \brief Major eigen vector */
		Eigen::Vector3f major_axis_;

		/** \brief Middle eigen vector */
		Eigen::Vector3f middle_axis_;

		/** \brief Minor eigen vector */
		Eigen::Vector3f minor_axis_;

		/** \brief Major eigen value */
		float major_value_;

		/** \brief Middle eigen value */
		float middle_value_;

		/** \brief Minor eigen value */
		float minor_value_;
	};

	class VoxelContainerBase
	{

	public:
		/** \brief Empty constructor. */
		VoxelContainerBase()
		{
		}

		/** \brief Empty constructor. */
		VoxelContainerBase(const VoxelContainerBase&)
		{
		}

		/** \brief Empty deconstructor. */
		virtual
			~VoxelContainerBase()
		{
		}

		/** \brief Equal comparison operator
		*/
		virtual bool
			operator== (const VoxelContainerBase&) const
		{
			return false;
		}

		/** \brief Inequal comparison operator
		* \param[in] other VoxelContainerBase to compare with
		*/
		virtual bool
			operator!= (const VoxelContainerBase& other) const
		{
			return (!operator== (other));
		}

		/** \brief Pure abstract method to get size of container (number of indices)
		* \return number of points/indices stored in leaf node container.
		*/
		virtual size_t
			getSize() const
		{
			return 0u;
		}

		/** \brief Pure abstract reset leaf node implementation. */
		virtual void
			reset() 
		{
		}

		/** \brief Empty addPointIndex implementation. This leaf node does not store any point indices.
		*/
		virtual void
			addPointIndex(const int&)
		{
		}

		/** \brief Empty getPointIndex implementation as this leaf node does not store any point indices.
		*/
		virtual void
			getPointIndex(int&) const
		{
		}

		/** \brief Empty getPointIndices implementation as this leaf node does not store any data. \
		*/
		// 			void
		// 				getPointIndices (std::vector<int>&) const
		// 			{
		// 			}

		std::vector<int>*
			getPointIndices() const
		{
			//std::vector<int> p;
			return 0;
		}

		virtual VoxelContainerBase* getFather() { return NULL; };
		virtual iplOctreeKey getFatherID() { return iplOctreeKey(); };
		virtual VoxelFeature* getFeature() { return NULL; };
	};

	class VoxelContainerPointIndices : public VoxelContainerBase
	{
	public:
		/** \brief Empty constructor. */
		VoxelContainerPointIndices() :
			VoxelContainerBase(), leafDataTVector_()
		{
		}

		/** \brief Empty constructor. */
		VoxelContainerPointIndices(const VoxelContainerPointIndices& source) :
			VoxelContainerBase(), leafDataTVector_(source.leafDataTVector_)
		{
		}

		/** \brief Empty deconstructor. */
		virtual
			~VoxelContainerPointIndices()
		{
		}

		/** \brief Octree deep copy method */
// 		virtual VoxelContainerPointIndices *
// 			deepCopy() const
// 		{
// 			return (new VoxelContainerPointIndices(*this));
// 		}

		/** \brief Equal comparison operator
		* \param[in] other VoxelContainerDataTVector to compare with
		*/
		virtual bool
			operator== (const VoxelContainerBase& other) const
		{
			const VoxelContainerPointIndices* otherConDataTVec = dynamic_cast<const VoxelContainerPointIndices*> (&other);

			return (this->leafDataTVector_ == otherConDataTVec->leafDataTVector_);
		}

		/** \brief Add point index to container memory. This container stores a vector of point indices.
		* \param[in] data_arg index to be stored within leaf node.
		*/
		virtual void
			addPointIndex(int data_arg)
		{
			leafDataTVector_.push_back(data_arg);
		}

		/** \brief Retrieve point index from container. This container stores a vector of point indices.
		* \return index stored within container.
		*/
		virtual int
			getPointIndex() const
		{
			return leafDataTVector_.back();
		}

		/** \brief Retrieve point indices from container. This container stores a vector of point indices.
		* \param[out] data_vector_arg vector of point indices to be stored within data vector
		*/
		// 			  void
		// 				  getPointIndices (std::vector<int>& data_vector_arg) const
		// 			  {
		// 				  data_vector_arg.insert (data_vector_arg.end (), leafDataTVector_.begin (), leafDataTVector_.end ());
		// 			  }

		/** \brief Retrieve reference to point indices vector. This container stores a vector of point indices.
		* \return reference to vector of point indices to be stored within data vector
		*/
		std::vector<int>*
			getPointIndices()
		{
			return &leafDataTVector_;
		}

		/** \brief Get size of container (number of indices)
		* \return number of point indices in container.
		*/
		virtual size_t
			getSize() const
		{
			return leafDataTVector_.size();
		}

		/** \brief Reset leaf node. Clear DataT vector.*/
		virtual void
			reset()
		{
			leafDataTVector_.clear();
		}

		// 		public:
		// 			int		label;     //-1 unlabeled
		// 			int  occupyFlag;  //0: null, 1: occupied, 2: inner
		// 			int     vNo;      //编号，辅助算法用
		// 
		// 			pcl::octree::OctreeKey	voxel_key;  //ix, iy, iz

		virtual VoxelContainerBase* getFather() { return fatherptr; };
		virtual iplOctreeKey getFatherID() { return fatherindice; };
		virtual VoxelFeature* getFeature() { return &feat; };

	private:
		VoxelContainerPointIndices *fatherptr;
		iplOctreeKey   fatherindice;

	protected:
		/** \brief Leaf node DataT vector. */
		std::vector<int> leafDataTVector_;

	protected:
		//voxel features
		VoxelFeature feat;
	};
}

