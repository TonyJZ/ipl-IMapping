#pragma once

#include <core/ipldef.h>
#include <core/iplversion.h>
#include <core/iplstd.h>
// #include "core/base.hpp"
// #include "opencv2/core/traits.hpp"
// #include "opencv2/core/matx.hpp"
// #include "opencv2/core/types.hpp"
// #include "opencv2/core/mat.hpp"
// #include "opencv2/core/persistence.hpp"

namespace ipl
{
	/* \brief Class passed to an error.
	*
	* This class encapsulates all or almost all necessary
	* information about the error happened in the program.
	*/
	class iplException : public std::exception
	{
	public:
		/*!
		Default constructor
		*/
		iplException();
		/*!
		Full constructor. Normally the constructor is not called explicitly.
		Instead, the macros IPL_Error(), IPL_Error_() and IPL_Assert() are used.
		*/
		iplException(int _code, const std::string& _err, const std::string& _func, const std::string& _file, int _line);
		virtual ~iplException() throw();

		/*!
		\return the error description and the context as a text string.
		*/
		virtual const char *what() const throw();
		void formatMessage();

		std::string msg; ///< the formatted error message

		int code; ///< error code @see IPLStatus
		std::string err; ///< error description
		std::string func; ///< function name. Available only when the compiler supports getting it
		std::string file; ///< source file name where the error has occurred
		int line; ///< line number in the source file where the error has occurred
	};

	//	IPL_EXPORTS void error(const Exception& exc);

}

// #include <core/iplfiles.h>
// #include <core/iplstring.h>

// data structures definition
#if HAVE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/pcl_base.h>
//#include <pcl/octree/octree_key.h>
//#include <pcl/search/search.h>
//	typedef	pcl::PointCloud		iplPointCloud;
//	using pcl::PointCloud<pcl::PointXYZRGBNormal>;
#else
// struct iplPointCloud
// {
// 
// 	};
#endif // HAVE_PCL

#ifdef HAVE_OPENCV

#else

#endif // HAVE_OPENCV

#ifdef HAVE_CGAL

#else

#endif // HAVE_CGAL


namespace ipl
{
#if HAVE_PCL
	//using pcl::PointCloud;
	/*template <typename PointT>
	class iplPointCloud : public pcl::PointCloud<PointT>
	{
	};*/
	template <typename PointT>
	using iplPointCloud = pcl::PointCloud<PointT>; //container of a set of points

	template <typename PointT>
	using iplPointCluster = pcl::PCLBase<PointT>; //point cluster 

	//supported point structure 
	typedef	pcl::PointXY		iplPointXY;    //x,y
	typedef pcl::PointXYZ		iplPointXYZ;   //x,y,z
	typedef pcl::PointXYZI		iplPointXYZI;  //x,y,z,intensity
	typedef	pcl::PointXYZL		iplPointXYZL;  //x,y,z,label
	typedef	pcl::PointXYZRGBA	iplPointXYZRGBA;//x,y,z,RGB,A
	typedef	pcl::PointXYZRGB	iplPointXYZRGB; //x,y,z,RGB
	typedef	pcl::PointXYZRGBL	iplPointXYZRGBL;//x,y,z,RGB,label
	typedef	pcl::PointXYZHSV	iplPointXYZHSV;//x,y,z,HSV
	typedef	pcl::PointXYZRGBNormal		iplPointXYZRGBNormal;//x,y,z,RGB,normal
	typedef	pcl::PointXYZINormal		iplPointXYZINormal;//x,y,z,intensity,normal
	/* typedef	pcl::PointXYZLNormal		iplPointXYZLNormal; */ //x,y,z,label,normal

	typedef pcl::ModelCoefficients		iplModelCoeffs;    //model's coefficients

//	typedef pcl::octree::OctreeKey		iplOctreeKey;

#else
	//	template <typename PointT>
	//	class iplPointCloud : public pcl::PointCloud<PointT>
	//	{
	//	};

#endif

	//translation definition
#if  HAVE_OPENCV

#else

#endif

#if  HAVE_CGAL

#else

#endif

}

