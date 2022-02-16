#pragma once
#include <cstdint>
#include <string>

namespace ipl
{
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class iplOctreeKey
	{
	public:

		/** \brief Empty constructor. */
		iplOctreeKey() :
			x(0), y(0), z(0)
		{
		}

		/** \brief Constructor for key initialization. */
		iplOctreeKey(int32_t keyX, int32_t keyY, int32_t keyZ) :
			x(keyX), y(keyY), z(keyZ)
		{
		}

		/** \brief Copy constructor. */
		iplOctreeKey(const iplOctreeKey& source)
		{
			memcpy(key_, source.key_, sizeof(key_));
		}

		/** \brief Operator== for comparing octree keys with each other.
		*  \return "true" if leaf node indices are identical; "false" otherwise.
		* */
		bool
			operator == (const iplOctreeKey& b) const
		{
			return ((b.x == this->x) && (b.y == this->y) && (b.z == this->z));
		}

		/** \brief Operator<= for comparing octree keys with each other.
		*  \return "true" if key indices are not greater than the key indices of b  ; "false" otherwise.
		* */
		bool
			operator <= (const iplOctreeKey& b) const
		{
			return ((b.x >= this->x) && (b.y >= this->y) && (b.z >= this->z));
		}

		/** \brief Operator>= for comparing octree keys with each other.
		*  \return "true" if key indices are not smaller than the key indices of b  ; "false" otherwise.
		* */
		bool
			operator >= (const iplOctreeKey& b) const
		{
			return ((b.x <= this->x) && (b.y <= this->y) && (b.z <= this->z));
		}

		
		iplOctreeKey
			operator + (const iplOctreeKey& b) 
		{
			iplOctreeKey c;
			c.x = this->x + b.x;
			c.y = this->y + b.y;
			c.z = this->z + b.z;
			return c;
		}

		iplOctreeKey
			operator - (const iplOctreeKey& b) 
		{
			iplOctreeKey c;
			c.x = this->x - b.x;
			c.y = this->y - b.y;
			c.z = this->z - b.z;
			return c;
		}

		iplOctreeKey& 
			operator += (const iplOctreeKey& b) 
		{
			this->x += b.x;
			this->y += b.y;
			this->z += b.z;
			return *this;
		}

		iplOctreeKey&
			operator -= (const iplOctreeKey& b)
		{
			this->x -= b.x;
			this->y -= b.y;
			this->z -= b.z;
			return *this;
		}
		/** \brief push a child node to the octree key
		*  \param[in] childIndex index of child node to be added (0-7)
		* */
		inline void
			pushBranch(unsigned char childIndex)
		{
			this->x = (this->x << 1) | (!!(childIndex & (1 << 2)));
			this->y = (this->y << 1) | (!!(childIndex & (1 << 1)));
			this->z = (this->z << 1) | (!!(childIndex & (1 << 0)));
		}

		/** \brief pop child node from octree key
		* */
		inline void
			popBranch()
		{
			this->x >>= 1;
			this->y >>= 1;
			this->z >>= 1;
		}

		/** \brief get child node index using depthMask
		*  \param[in] depthMask bit mask with single bit set at query depth
		*  \return child node index
		* */
		inline unsigned char
			getChildIdxWithDepthMask(unsigned int depthMask) const
		{
			return static_cast<unsigned char> (((!!(this->x & depthMask)) << 2)
				| ((!!(this->y & depthMask)) << 1)
				| (!!(this->z & depthMask)));
		}

		/* \brief maximum depth that can be addressed */
		static const unsigned char maxDepth = static_cast<const unsigned char>(sizeof(int32_t) * 8);

		// Indices addressing a voxel at (X, Y, Z)

		union
		{
			struct
			{
				int32_t x;
				int32_t y;
				int32_t z;
			};
			int32_t key_[3];
		};
	};
}


