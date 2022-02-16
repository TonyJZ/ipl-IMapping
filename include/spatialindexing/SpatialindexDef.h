#pragma once

#include "core/iplcore.h"
#include "spatialindexing/VoxelContainer.h"
#include "spatialindexing/iploctree_key.h"

//#include <pcl/octree/octree_key.h>
#include <boost/unordered_map.hpp>
//#include <boost/functional/hash/hash.hpp>

namespace ipl
{
	//VoxelContainerPointIndices需要进行接口抽象
//	typedef std::vector<ipl::VoxelContainerPointIndices> VoxelList;
	//voxel的octree编码，对应于VoxelList中的ID (note：这两个结构需要进一步重构，将内存结构直接放到map中，相关函数实现都需要修改 2017-10-14)
//	typedef boost::unordered_map<pcl::octree::OctreeKey, int> VoxelKeyMap;

	//2018-04-03 利用操作符重载合并VoxelList和VoxelKeyMap
	typedef boost::unordered_map<iplOctreeKey, VoxelContainerPointIndices> VoxelKeyMap;

	//boost::unordered_map中对自定义类型，必须自己实现hash函数
	static size_t hash_value(const iplOctreeKey& b)
	{
		return boost::hash_value(b.key_);
	}
}

namespace ipl
{
	class QuadtreeKey
	{
	public:

		/** \brief Empty constructor. */
		QuadtreeKey() :
			x(0), y(0)
		{
		}

		/** \brief Constructor for key initialization. */
		QuadtreeKey(unsigned int keyX, unsigned int keyY) :
			x(keyX), y(keyY)
		{
		}

		/** \brief Copy constructor. */
		QuadtreeKey(const QuadtreeKey& source)
		{
			memcpy(key_, source.key_, sizeof(key_));
		}

		/** \brief Operator== for comparing octree keys with each other.
		*  \return "true" if leaf node indices are identical; "false" otherwise.
		* */
		bool
			operator == (const QuadtreeKey& b) const
		{
			return ((b.x == this->x) && (b.y == this->y));
		}

		/** \brief Operator<= for comparing octree keys with each other.
		*  \return "true" if key indices are not greater than the key indices of b  ; "false" otherwise.
		* */
		bool
			operator <= (const QuadtreeKey& b) const
		{
			return ((b.x >= this->x) && (b.y >= this->y));
		}

		/** \brief Operator>= for comparing octree keys with each other.
		*  \return "true" if key indices are not smaller than the key indices of b  ; "false" otherwise.
		* */
		bool
			operator >= (const QuadtreeKey& b) const
		{
			return ((b.x <= this->x) && (b.y <= this->y));
		}

		QuadtreeKey
			operator + (const QuadtreeKey& b) const
		{
			QuadtreeKey c;
			c.x = this->x + b.x;
			c.y = this->y + b.y;
			return c;
		}

		/** \brief push a child node to the octree key
		*  \param[in] childIndex index of child node to be added (0-7)
		* */
// 		inline void
// 			pushBranch(unsigned char childIndex)
// 		{
// 			this->x = (this->x << 1) | (!!(childIndex & (1 << 2)));
// 			this->y = (this->y << 1) | (!!(childIndex & (1 << 1)));
// 			this->z = (this->z << 1) | (!!(childIndex & (1 << 0)));
// 		}
// 
// 		/** \brief pop child node from octree key
// 		* */
// 		inline void
// 			popBranch()
// 		{
// 			this->x >>= 1;
// 			this->y >>= 1;
// 			this->z >>= 1;
// 		}

		/** \brief get child node index using depthMask
		*  \param[in] depthMask bit mask with single bit set at query depth
		*  \return child node index
		* */
// 		inline unsigned char
// 			getChildIdxWithDepthMask(unsigned int depthMask) const
// 		{
// 			return static_cast<unsigned char> (((!!(this->x & depthMask)) << 2)
// 				| ((!!(this->y & depthMask)) << 1)
// 				| (!!(this->z & depthMask)));
// 		}
// 
// 		/* \brief maximum depth that can be addressed */
// 		static const unsigned char maxDepth = static_cast<const unsigned char>(sizeof(uint32_t) * 8);

		// Indices addressing a voxel at (X, Y, Z)

		union
		{
			struct
			{
				int32_t x;
				int32_t y;
			};
			int32_t key_[2];
		};

	};
}


namespace ipl
{
	static size_t hash_value(const QuadtreeKey& b)
	{
		return boost::hash_value(b.key_);
	}

//	typedef std::vector<ipl::VoxelContainerPointIndices> CellList;
//	typedef boost::unordered_map<ipl::QuadtreeKey, int> CellKeyMap;  //first: gridcell index,  second: points in this cell
	typedef boost::unordered_map<ipl::QuadtreeKey, ipl::VoxelContainerPointIndices> CellKeyMap;
}


namespace ipl
{
	//std::pair<int, int> :  connected cluster ID pair;   int: overlapping degree 
	typedef boost::unordered_map<std::pair<int, int>, int> OVERLAPMap;

	static std::size_t hash_value(std::pair<int, int> const& v)
	{
		std::size_t seed = 0;
		boost::hash_combine(seed, v.first);
		boost::hash_combine(seed, v.second);
		return seed;
	}
}

