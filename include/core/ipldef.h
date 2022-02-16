#pragma once

// #if !defined IPL_DOXYGEN && !defined IPL_IGNORE_DEBUG_BUILD_GUARD
// #if (defined(_MSC_VER) && (defined(DEBUG) || defined(_DEBUG))) || \
//     (defined(_GLIBCXX_DEBUG) || defined(_GLIBCXX_DEBUG_PEDANTIC))
// // Guard to prevent using of binary incompatible binaries / runtimes
// // https://github.com/opencv/opencv/pull/9161
// #define IPL__DEBUG_NS_BEGIN namespace debug_build_guard {
// #define IPL__DEBUG_NS_END }
// namespace cv { namespace debug_build_guard {} using namespace debug_build_guard; }
// #endif
// #endif

// #ifndef IPL__DEBUG_NS_BEGIN
// #define IPL__DEBUG_NS_BEGIN
// #define IPL__DEBUG_NS_END
// #endif


// #ifdef __OPENCV_BUILD
// #include "cvconfig.h"
// #endif

// #ifndef __IPL_EXPAND
// #define __IPL_EXPAND(x) x
// #endif
// 
// #ifndef __IPL_CAT
// #define __IPL_CAT__(x, y) x ## y
// #define __IPL_CAT_(x, y) __IPL_CAT__(x, y)
// #define __IPL_CAT(x, y) __IPL_CAT_(x, y)
// #endif

// define supported libraries
#define	HAVE_PCL		1	
#define	HAVE_OPENCV		1	
#define HAVE_CGAL		1	
#define HAVE_OpenMP		1

// undef problematic defines sometimes defined by system headers (windows.h in particular)
#undef small
#undef min
#undef max
#undef abs
#undef Complex

#include <cmath>
#include <limits.h>
//#include "interface.h"

// #if defined __ICL
// #  define IPL_ICC   __ICL
// #elif defined __ICC
// #  define IPL_ICC   __ICC
// #elif defined __ECL
// #  define IPL_ICC   __ECL
// #elif defined __ECC
// #  define IPL_ICC   __ECC
// #elif defined __INTEL_COMPILER
// #  define IPL_ICC   __INTEL_COMPILER
// #endif

#ifndef IPL_INLINE
#  if defined __cplusplus
#    define IPL_INLINE static inline
#  elif defined _MSC_VER
#    define IPL_INLINE __inline
#  else
#    define IPL_INLINE static
#  endif
#endif

#if defined IPL_DISABLE_OPTIMIZATION || (defined IPL_ICC && !defined IPL_ENABLE_UNROLLED)
#  define IPL_ENABLE_UNROLLED 0
#else
#  define IPL_ENABLE_UNROLLED 1
#endif

#ifdef __GNUC__
#  define IPL_DECL_ALIGNED(x) __attribute__ ((aligned (x)))
#elif defined _MSC_VER
#  define IPL_DECL_ALIGNED(x) __declspec(align(x))
#else
#  define IPL_DECL_ALIGNED(x)
#endif

/* CPU features and intrinsics support */
#define IPL_CPU_NONE             0
#define IPL_CPU_MMX              1
#define IPL_CPU_SSE              2
#define IPL_CPU_SSE2             3
#define IPL_CPU_SSE3             4
#define IPL_CPU_SSSE3            5
#define IPL_CPU_SSE4_1           6
#define IPL_CPU_SSE4_2           7
#define IPL_CPU_POPCNT           8
#define IPL_CPU_FP16             9
#define IPL_CPU_AVX              10
#define IPL_CPU_AVX2             11
#define IPL_CPU_FMA3             12

#define IPL_CPU_AVX_512F         13
#define IPL_CPU_AVX_512BW        14
#define IPL_CPU_AVX_512CD        15
#define IPL_CPU_AVX_512DQ        16
#define IPL_CPU_AVX_512ER        17
#define IPL_CPU_AVX_512IFMA512   18
#define IPL_CPU_AVX_512PF        19
#define IPL_CPU_AVX_512VBMI      20
#define IPL_CPU_AVX_512VL        21

#define IPL_CPU_NEON   100

// when adding to this list remember to update the following enum
#define IPL_HARDWARE_MAX_FEATURE 255


/* fundamental constants */
#define IPL_PI   3.1415926535897932384626433832795
#define IPL_2PI  6.283185307179586476925286766559
#define IPL_LOG2 0.69314718055994530941723212145818


// #if (defined _WIN32 || defined WINCE || defined __CYGWIN__) && defined IPLAPI_EXPORTS
// #  define IPL_EXPORTS __declspec(dllexport)
// #elif defined __GNUC__ && __GNUC__ >= 4
// #  define IPL_EXPORTS __attribute__ ((visibility ("default")))
// #else
// #  define IPL_EXPORTS
// #endif

#ifndef IPL_DEPRECATED
#  if defined(__GNUC__)
#    define IPL_DEPRECATED __attribute__ ((deprecated))
#  elif defined(_MSC_VER)
#    define IPL_DEPRECATED __declspec(deprecated)
#  else
#    define IPL_DEPRECATED
#  endif
#endif

#ifndef IPL_EXTERN_C
#  ifdef __cplusplus
#    define IPL_EXTERN_C extern "C"
#  else
#    define IPL_EXTERN_C
#  endif
#endif

/* special informative macros for wrapper generators */
// #define IPL_EXPORTS_W IPL_EXPORTS
// #define IPL_EXPORTS_W_SIMPLE IPL_EXPORTS
// #define IPL_EXPORTS_AS(synonym) IPL_EXPORTS
// #define IPL_EXPORTS_W_MAP IPL_EXPORTS
// #define IPL_IN_OUT
// #define IPL_OUT
// #define IPL_PROP
// #define IPL_PROP_RW
// #define IPL_WRAP
// #define IPL_WRAP_AS(synonym)

/****************************************************************************************\
*                                  Matrix type (Mat)                                     *
\****************************************************************************************/

// #define IPL_MAT_CN_MASK          ((IPL_CN_MAX - 1) << IPL_CN_SHIFT)
// #define IPL_MAT_CN(flags)        ((((flags) & IPL_MAT_CN_MASK) >> IPL_CN_SHIFT) + 1)
// #define IPL_MAT_TYPE_MASK        (IPL_DEPTH_MAX*IPL_CN_MAX - 1)
// #define IPL_MAT_TYPE(flags)      ((flags) & IPL_MAT_TYPE_MASK)
// #define IPL_MAT_CONT_FLAG_SHIFT  14
// #define IPL_MAT_CONT_FLAG        (1 << IPL_MAT_CONT_FLAG_SHIFT)
// #define IPL_IS_MAT_CONT(flags)   ((flags) & IPL_MAT_CONT_FLAG)
// #define IPL_IS_CONT_MAT          IPL_IS_MAT_CONT
// #define IPL_SUBMAT_FLAG_SHIFT    15
// #define IPL_SUBMAT_FLAG          (1 << IPL_SUBMAT_FLAG_SHIFT)
// #define IPL_IS_SUBMAT(flags)     ((flags) & IPL_MAT_SUBMAT_FLAG)

/** Size of each channel item,
0x8442211 = 1000 0100 0100 0010 0010 0001 0001 ~ array of sizeof(arr_type_elem) */
// #define IPL_ELEM_SIZE1(type) \
//     ((((sizeof(size_t)<<28)|0x8442211) >> IPL_MAT_DEPTH(type)*4) & 15)

/** 0x3a50 = 11 10 10 01 01 00 00 ~ array of log2(sizeof(arr_type_elem)) */
// #define IPL_ELEM_SIZE(type) \
//     (IPL_MAT_CN(type) << ((((sizeof(size_t)/4+1)*16384|0x3a50) >> IPL_MAT_DEPTH(type)*2) & 3))

// #ifndef MIN
// #  define MIN(a,b)  ((a) > (b) ? (b) : (a))
// #endif
// 
// #ifndef MAX
// #  define MAX(a,b)  ((a) < (b) ? (b) : (a))
// #endif

/****************************************************************************************\
*                                    static analysys                                     *
\****************************************************************************************/

// In practice, some macro are not processed correctly (noreturn is not detected).
// We need to use simplified definition for them.
// #ifndef IPL_STATIC_ANALYSIS
// # if defined(__KLOCWORK__) || defined(__clang_analyzer__) || defined(__COVERITY__)
// #   define IPL_STATIC_ANALYSIS
// # endif
// #endif

/****************************************************************************************\
*                                    Thread sanitizer                                    *
\****************************************************************************************/
// #ifndef IPL_THREAD_SANITIZER
// # if defined(__has_feature)
// #   if __has_feature(thread_sanitizer)
// #     define IPL_THREAD_SANITIZER
// #   endif
// # endif
// #endif

/****************************************************************************************\
*          exchange-add operation for atomic operations on reference counters            *
\****************************************************************************************/

// #ifdef IPL_XADD
// // allow to use user-defined macro
// #elif defined __GNUC__ || defined __clang__
// #  if defined __clang__ && __clang_major__ >= 3 && !defined __ANDROID__ && !defined __EMSCRIPTEN__ && !defined(__CUDACC__)
// #    ifdef __ATOMIC_ACQ_REL
// #      define IPL_XADD(addr, delta) __c11_atomic_fetch_add((_Atomic(int)*)(addr), delta, __ATOMIC_ACQ_REL)
// #    else
// #      define IPL_XADD(addr, delta) __atomic_fetch_add((_Atomic(int)*)(addr), delta, 4)
// #    endif
// #  else
// #    if defined __ATOMIC_ACQ_REL && !defined __clang__
// // version for gcc >= 4.7
// #      define IPL_XADD(addr, delta) (int)__atomic_fetch_add((unsigned*)(addr), (unsigned)(delta), __ATOMIC_ACQ_REL)
// #    else
// #      define IPL_XADD(addr, delta) (int)__sync_fetch_and_add((unsigned*)(addr), (unsigned)(delta))
// #    endif
// #  endif
// #elif defined _MSC_VER && !defined RC_INVOKED
// #  include <intrin.h>
// #  define IPL_XADD(addr, delta) (int)_InterlockedExchangeAdd((long volatile*)addr, delta)
// #else
// IPL_INLINE IPL_XADD(int* addr, int delta) { int tmp = *addr; *addr += delta; return tmp; }
// #endif


/****************************************************************************************\
*                                  CV_NORETURN attribute                                 *
\****************************************************************************************/

// #ifndef IPL_NORETURN
// #  if defined(__GNUC__)
// #    define IPL_NORETURN __attribute__((__noreturn__))
// #  elif defined(_MSC_VER) && (_MSC_VER >= 1300)
// #    define IPL_NORETURN __declspec(noreturn)
// #  else
// #    define IPL_NORETURN /* nothing by default */
// #  endif
// #endif


/****************************************************************************************\
*                                    C++ 11                                              *
\****************************************************************************************/
// #ifndef IPL_CXX11
// #  if __cplusplus >= 201103L || (defined(_MSC_VER) && _MSC_VER >= 1800)
// #    define IPL_CXX11 1
// #  endif
// #else
// #  if IPL_CXX11 == 0
// #    undef IPL_CXX11
// #  endif
// #endif


/****************************************************************************************\
*                                    C++ Move semantics                                  *
\****************************************************************************************/

// #ifndef IPL_CXX_MOVE_SEMANTICS
// #  if __cplusplus >= 201103L || defined(__GXX_EXPERIMENTAL_CXX0X__) || (defined(_MSC_VER) && _MSC_VER >= 1600)
// #    define IPL_CXX_MOVE_SEMANTICS 1
// #  elif defined(__clang)
// #    if __has_feature(cxx_rvalue_references)
// #      define IPL_CXX_MOVE_SEMANTICS 1
// #    endif
// #  endif
// #else
// #  if IPL_CXX_MOVE_SEMANTICS == 0
// #    undef IPL_CXX_MOVE_SEMANTICS
// #  endif
// #endif

/****************************************************************************************\
*                                    C++11 std::array                                    *
\****************************************************************************************/

// #ifndef IPL_CXX_STD_ARRAY
// #  if __cplusplus >= 201103L
// #    define IPL_CXX_STD_ARRAY 1
// #    include <array>
// #  endif
// #else
// #  if IPL_CXX_STD_ARRAY == 0
// #    undef IPL_CXX_STD_ARRAY
// #  endif
// #endif

//! @}


#ifdef WIN32
#include <tchar.h>
#else
#if defined(_UNICODE) || defined(UNICODE)
#include <wchar.h>
#define _T(x) L##x
#else
#define _T(x) x
#endif

// ignore case
#define stricmp strcasecmp

#if defined(_UNICODE) || defined(UNICODE)
#define _tcschr     wcschr
#define _tcsstr     wcsstr

// 实际的字符数，不是字节数
#define _tcsclen    wcslen
#define _tcsicmp	wcsicmp
#define _tcscmp		wcscmp
#else
#define _tcschr     strchr
#define _tcsstr     strstr

// 实际的字符数，不是字节数
#define _tcsclen    strlen
#define _tcsicmp	stricmp
#define _tcscmp		strcmp
#endif
#endif

#if defined(_UNICODE) || defined(UNICODE)
typedef wchar_t				iplChar;

// 字符数，是字节数的一半
#define	iplStrLen			wcslen
#else
typedef char				iplChar;

// 按英文计算的字符数，中文算两个，就是实际的字节数
#define	iplStrLen			strlen
#endif


#if defined(WIN64) || defined(_WIN64)
#define IPL_PLATFORM_64
#define IPL_PLATFORM_WINDOWS
#elif defined(WIN32) || defined(_WIN32)
#define IPL_PLATFORM_32
#define IPL_PLATFORM_WINDOWS
#elif defined(LINUX64) || defined(_LINUX64)
#define IPL_PLATFORM_64
#define IPL_PLATFORM_LINUX
#elif defined(LINUX32) || defined(_LINUX32)
#define IPL_PLATFORM_32
#define IPL_PLATFORM_LINUX
#else
#error "Platform not Specified!"
#endif

#ifndef IPL_PLATFORM_WINDOWS
typedef unsigned int UINT;
typedef unsigned char BYTE;
#define stricmp strcasecmp
#endif

#include <float.h>

#define ipl_max(a,b)    (((a) > (b)) ? (a) : (b))
#define ipl_min(a,b)    (((a) < (b)) ? (a) : (b))


/* Basic types */
typedef  bool			ipl_bool;

typedef  char           ipl_int8;
typedef  unsigned char  ipl_uint8;
typedef  unsigned char  ipl_byte;
typedef  unsigned int	ipl_uint;

typedef  short			ipl_int16;
typedef  unsigned short ipl_uint16;

typedef  float          ipl_float32;
typedef  double         ipl_float64;


#if defined(IPL_PLATFORM_WINDOWS )
typedef  long             ipl_int32;
typedef  unsigned long    ipl_uint32;
typedef  __int64          ipl_int64;
typedef  unsigned __int64	ipl_uint64;
#elif defined(IPL_PLATFORM_LINUX)
#ifdef IPL_PLATFORM_32
typedef  int             ipl_int32;
typedef  unsigned int    ipl_uint32;
typedef  long long       ipl_int64;
typedef  unsigned long long   ipl_uint64;
#else
typedef  int             ipl_int32;
typedef  unsigned int    ipl_uint32;
typedef  long            ipl_int64;
typedef  unsigned long   ipl_uint64;
#endif
#endif



// 改为ipl_uint64在linux下会出错
#ifdef IPL_PLATFORM_32
typedef ipl_uint32	ipl_ptr2int;
typedef ipl_uint32	ipl_registerInt;
#else
typedef ipl_uint64	ipl_ptr2int;
typedef ipl_uint64	ipl_registerInt;
#endif

#ifndef BYTE
typedef unsigned char BYTE;
#endif

//////////////////////////////////////////////////////////////////////////
//由于C语言不支持对象，所以库中的对象为了调用其功能，转成void*类型的指针
typedef void*		iplHandle;
typedef void*		iplHDC;
typedef void*		iplWnd;


typedef long iplID;

#ifndef interface
#define interface struct
#endif

//////////////////////////////////////////////////////////////////////////
#define IPL_EVENT_OBJECT_DESTRUCTING_ID     1
#define IPL_EVENT_CONNECTION_DISCONNECT_ID	2
#define IPL_EVENT_CONNECTION_CONNECT_ID		3

#ifndef iplZERO
#define iplZERO 1e-6
#endif
#ifndef NULL
#define NULL 0
#endif

#define IPL_DBL_NAN    ((ipl_float64)-1.0/DBL_EPSILON)
#define IPL_FLT_NAN    ((ipl_float32)-1.0/FLT_EPSILON)
#define IPL_LONG_NAN   ((ipl_int32)0x80000000)
#define IPL_INT_NAN    ((ipl_int32)0x80000000)
#define IPL_ULONG_NAN  ((ipl_uint32)0x0)
#define IPL_UINT_NAN   ((ipl_uint32)0x0)
#define IPL_SSHORT_NAN ((ipl_int16)0x8000)
#define IPL_USHORT_NAN ((ipl_uint16)0)

#define IPL_DEFAULT_MIN_PIX_UCHAR ((ipl_byte)1)
#define IPL_DEFAULT_MAX_PIX_UCHAR ((ipl_byte)255)
#define IPL_DEFAULT_MIN_PIX_CHAR ((ipl_byte)-128)
#define IPL_DEFAULT_MAX_PIX_CHAR ((ipl_byte)127)
#define IPL_DEFAULT_MIN_PIX_UINT8 ((ipl_byte)1)
#define IPL_DEFAULT_MAX_PIX_UINT8 ((ipl_byte)255)
#define IPL_DEFAULT_MIN_PIX_INT8 ((ipl_int8)0x81)
#define IPL_DEFAULT_MAX_PIX_INT8 ((ipl_int8)0x7F)
#define IPL_DEFAULT_MIN_PIX_INT16 ((ipl_int16)0x8001)
#define IPL_DEFAULT_MAX_PIX_INT16 ((ipl_int16)0x7FFF)
#define IPL_DEFAULT_MIN_PIX_UINT16 ((ipl_uint16)1)
#define IPL_DEFAULT_MAX_PIX_UINT16 ((ipl_uint16)0xFFFF)
#define IPL_DEFAULT_MIN_PIX_INT32 ((ipl_int32)0x80000001)
#define IPL_DEFAULT_MAX_PIX_INT32 ((ipl_int32)0x7FFFFFFF)
#define IPL_DEFAULT_MIN_PIX_UINT32 ((ipl_uint32)1)
#define IPL_DEFAULT_MAX_PIX_UINT32 ((ipl_uint32)0xFFFFFFFF)

#define IPL_DEFAULT_MIN_PIX_UINT11 ((ipl_uint16)1)
#define IPL_DEFAULT_MAX_PIX_UINT11 ((ipl_uint16)0x07FF)

#define IPL_DEFAULT_MIN_PIX_FLOAT ((ipl_float32)((-1.0/FLT_EPSILON) + 1))
#define IPL_DEFAULT_MAX_PIX_FLOAT  ((ipl_float32)((1.0/FLT_EPSILON)))
#define IPL_DEFAULT_MIN_PIX_NORM_FLOAT ((ipl_float32)((2*FLT_EPSILON)))
#define IPL_DEFAULT_MAX_PIX_NORM_FLOAT ((ipl_float32)1.0)
#define IPL_DEFAULT_MIN_PIX_DOUBLE ((ipl_float64)((-1.0/DBL_EPSILON) + 1))
#define IPL_DEFAULT_MAX_PIX_DOUBLE ((ipl_float64)((1.0/DBL_EPSILON)))
#define IPL_DEFAULT_MIN_PIX_NORM_DOUBLE ((ipl_float64)((2*DBL_EPSILON)))
#define IPL_DEFAULT_MAX_PIX_NORM_DOUBLE ((ipl_float64)(1.0))

/*! Pixel data types */
// 与GDAL对应，但存在不支持hdf有符号8bits整数的问题
// 修改：增加ipl_DT_INT8，假定是有符号8bits整数整数时，最小值为负数，即读写的时候利用最小值进行转换
//		
enum iplDataTYPE
{
	IPL_DT_UnKNOWN = 0,
	IPL_DT_BYTE = 1,	/*! 8 bit unsigned INTeger        */
						//IPL_DT_INT8		=  2,	/*! 8 bit signed INTeger          */
						IPL_DT_UINT16 = 2,	/*! 16 bit unsigned INTeger       */
						IPL_DT_INT16 = 3,	/*! 16 bit signed INTeger         */
						IPL_DT_UINT32 = 4,	/*! 32 bit unsigned INTeger       */
						IPL_DT_INT32 = 5,	/*! 32 bit signed INTeger         */
						IPL_DT_FLOAT32 = 6,	/*! 32 bit FLOATing poINT         */
						IPL_DT_FLOAT64 = 7,	/*! 64 bit FLOATing poINT         */
						IPL_DT_CINT16 = 8,	/*! Complex INT16  */
						IPL_DT_CINT32 = 9,	/*! Complex INT32       */
						IPL_DT_CFLOAT32 = 10,	/*! Complex FLOAT32     */
						IPL_DT_CFLOAT64 = 11,	/*! Complex FLOAT64        */
						IPL_DT_INT8 = 12,	/*! 8 bit signed INTeger， gdal不支持，需要特殊处理  */
						IPL_DT_COUNT = 13		/* maximum type # + 1 */
};


//////////////////////////////////////////////////////////////////////////


struct iplPOINT2Df
{
	float x;
	float y;
};


struct iplPOINT2Di
{
	ipl_int32 x;
	ipl_int32 y;

	iplPOINT2Di() { x = y = 0; };
	iplPOINT2Di(ipl_int32 xIn, ipl_int32 yIn) : x(xIn), y(yIn) {};
};


struct iplPOINT3Df
{
	float X;
	float Y;
	float Z;
};


struct iplLINE2Df
{
	iplPOINT2Df startPoint;
	iplPOINT2Df endPoint;
};
//////////////////////////////////////////////////////////////////////////

struct iplPOINT2D
{
	union { double x; double samp; double u; double lon; };
	union { double y; double line; double v; double lat; };

	iplPOINT2D() { x = y = 0; };
	iplPOINT2D(double x0, double y0) { x = x0; y = y0; }

	/*!
	* METHOD: length()
	* Returns the RSS of the components.
	*/
	double length() const { return std::sqrt(x*x + y*y); }

	//***
	// OPERATipl: +, -, +=, -=
	// Point add/subtract with other point:
	//***
	iplPOINT2D operator+(const iplPOINT2D& p) const
	{
		return iplPOINT2D(x + p.x, y + p.y);
	}
	iplPOINT2D operator-(const iplPOINT2D& p) const
	{
		return iplPOINT2D(x - p.x, y - p.y);
	}
	const iplPOINT2D& operator+=(const iplPOINT2D& p)
	{
		x += p.x; y += p.y; return *this;
	}
	const iplPOINT2D& operator-=(const iplPOINT2D& p)
	{
		x -= p.x; y -= p.y; return *this;
	}

	//***
	// OPERATipl: *, /
	// Scale point components by scalar:
	//***
	iplPOINT2D operator*(const double& d) const
	{
		return iplPOINT2D(d*x, d*y);
	}
	iplPOINT2D operator/(const double& d) const
	{
		return iplPOINT2D(x / d, y / d);
	}
};

struct iplNORMAL3D
{
	union
	{
		struct
		{
			double nx;
			double ny;
			double nz;
		};
		double normal[3];
	};
};


struct iplPOINT3D
{
	union { double X; double lon; };
	union { double Y; double lat; };
	union { double Z; double h; };

	// 说明: 结构不需要写赋值运算符，iplPOINT3D point应为const iplPOINT3D &point
	// 	inline iplPOINT3D& operator=(iplPOINT3D point)	//
	// 	{
	// 		X = point.X;
	// 		Y = point.Y;
	// 		X = point.Z;
	// 		return *this;
	// 	}

	iplPOINT3D() { X = 0; Y = 0; Z = 0.0; }
	iplPOINT3D(double x, double y, double z) { X = x; Y = y; Z = z; }

	bool operator == (const iplPOINT3D &point)
	{
		return (point.X == X && point.Y == Y && point.Z == Z);
	}

	bool operator != (const iplPOINT3D &point)
	{
		return (point.X != X || point.Y != Y || point.Z != Z);
	}

	/*!
	* METHOD: length()
	* Returns the RSS of the components.
	*/
	double length() const { return std::sqrt(X*X + Y*Y + Z*Z); }
	double length2() const { return X*X + Y*Y + Z*Z; }

	//***
	// OPERATipl: +, -, +=, -=
	// Point add/subtract with other point:
	//***
	iplPOINT3D operator+(const iplPOINT3D& p) const
	{
		return iplPOINT3D(X + p.X, Y + p.Y, Z + p.Z);
	}
	iplPOINT3D operator-(const iplPOINT3D& p) const
	{
		return iplPOINT3D(X - p.X, Y - p.Y, Z - p.Z);
	}
	const iplPOINT3D& operator+=(const iplPOINT3D& p)
	{
		X += p.X; Y += p.Y; Z += p.Z; return *this;
	}
	const iplPOINT3D& operator-=(const iplPOINT3D& p)
	{
		X -= p.X; Y -= p.Y; Z -= p.Z; return *this;
	}

	//***
	// OPERATipl: *, /
	// Scale point components by scalar:
	//***
	iplPOINT3D operator*(const double& d) const
	{
		return iplPOINT3D(d*X, d*Y, d*Z);
	}
	iplPOINT3D operator/(const double& d) const
	{
		return iplPOINT3D(X / d, Y / d, Z / d);
	}
	void operator /=(double value)
	{
		X /= value;
		Y /= value;
		Z /= value;
	}
	void operator *=(double value)
	{
		X *= value;
		Y *= value;
		Z *= value;
	}

	double operator *(const iplPOINT3D& src)const
	{
		return (X*src.X + Y*src.Y + Z*src.Z);
	}

	inline const iplPOINT3D operator ^ (const iplPOINT3D& rhs) const
	{
		return iplPOINT3D(Y*rhs.Z - Z*rhs.Y, Z*rhs.X - X*rhs.Z, X*rhs.Y - Y*rhs.X);
	}

	friend const iplPOINT3D operator *(const double &scale, const iplPOINT3D& rhs)
	{
		return rhs*scale;
	}

};

template <typename Scale>
class iplRECT
{
public:
	Scale	m_xmin;
	Scale	m_ymin;
	Scale	m_xmax;
	Scale	m_ymax;

	iplRECT() :m_xmin(0), m_ymin(0), m_xmax(0), m_ymax(0) {};

	// 容易出错, 换成左上角和右下角点
// 	iplRECT(orsPOINT2Di &tl, orsPOINT2Di &br) : m_xmin(tl.x), m_ymin(tl.y), m_xmax(br.x), m_ymax(br.y) {};
// 	iplRECT(const orsRect_i& w) : m_xmin(w.m_xmin), m_ymin(w.m_ymin), m_xmax(w.m_xmax), m_ymax(w.m_ymax) {};
// 	iplRECT(int xmin, int ymin, int xmax, int ymax) : m_xmin(xmin), m_ymin(ymin), m_xmax(xmax), m_ymax(ymax) {};


	//~orsRect_i()	{;};

	Scale height() const { return m_ymax - m_ymin; }	// 与MFC一致
	Scale width() const { return m_xmax - m_xmin; }	// 与MFC一致

	iplRECT& operator = (const iplRECT &rect)
	{
		if (this != &rect)
		{
			m_ymin = rect.m_ymin; m_ymax = rect.m_ymax; m_xmin = rect.m_xmin; m_xmax = rect.m_xmax;
		}
		return *this;
	}

	bool operator == (iplRECT &rect) const
	{
		return (m_ymin == rect.m_ymin&&m_ymax == rect.m_ymax &&m_xmin == rect.m_xmin && m_xmax == rect.m_xmax) ? true : false;
	}

	bool operator != (iplRECT &rect) const
	{
		return (m_ymin == rect.m_ymin&&m_ymax == rect.m_ymax &&m_xmin == rect.m_xmin && m_xmax == rect.m_xmax) ? false : true;
	}

	bool isIntersect(iplRECT& w) const
	{
		Scale xmax_min = ipl_max(w.m_xmin, m_xmin);
		Scale xmin_max = ipl_min(w.m_xmax, m_xmax);

		Scale ymax_min = ipl_max(w.m_ymin, m_ymin);
		Scale ymin_max = ipl_min(w.m_ymax, m_ymax);

		return ((xmax_min < xmin_max) && (ymax_min < ymin_max));
	}

	// 裁剪当前矩形并返回
	iplRECT clipToRect(iplRECT& w)
	{

		m_xmin = ipl_max(w.m_xmin, m_xmin);
		m_xmax = ipl_min(w.m_xmax, m_xmax);

		m_ymin = ipl_max(w.m_ymin, m_ymin);
		m_ymax = ipl_min(w.m_ymax, m_ymax);

		if (width() <= 0 || height() <= 0) {
			m_xmin = m_xmax = 0;
			m_ymin = m_ymax = 0;

			return orsRect_i();
		}

		return *this;
	}

	bool isEmpty() const
	{
		if (m_ymax == m_ymin && m_xmax == m_xmin)
			return true;
		else
			return false;
	}

	void Inflate(Scale dx, Scale dy)
	{
		m_xmin -= dx;	m_xmax += dx;
		m_ymin -= dy;	m_ymax += dy;
	}

	void Deflate(Scale dx, Scale dy)
	{
		m_xmin += dx;	m_xmax -= dx;
		m_ymin += dy;	m_ymax -= dy;
	}

	void OffsetRect(Scale dx, Scale dy)
	{
		m_xmin += dx;	m_xmax += dx;
		m_ymin += dy;	m_ymax += dy;
	}

	bool PtInRect(Scale px, Scale py)
	{
		if (px >= m_xmin && px <= m_xmax - 1e-6 && py >= m_ymin && py <= m_ymax - 1e-6)
		{
			return true;
		}
		else
			return false;
	}

// 	bool PtInRect(Scale px, Scale py)
// 	{
// 		if (px >= m_xmin && px <= m_xmax - 1 && py >= m_ymin && py <= m_ymax - 1)
// 		{
// 			return true;
// 		}
// 		else
// 			return false;
// 	}

	void SetRect(Scale xLeft, Scale yTop, Scale xRight, Scale yBottom)
	{
		m_xmin = xLeft;
		m_xmax = xRight;
		m_ymin = yTop;
		m_ymax = yBottom;
	}

};

// 三维直线
struct	iplLINE3D {
public:

	double X0, Y0, Z0;
	double dX, dY, dZ;				// 方向矢量

public:

	iplLINE3D() {};

	iplLINE3D(double X01, double Y01, double Z01, double dX1, double dY1, double dZ1)
		: X0(X01), Y0(Y01), Z0(Z01), dX(dX1), dY(dY1), dZ(dZ1) {};

	iplLINE3D(iplPOINT3D &pos, iplPOINT3D &vec)
		: X0(pos.X), Y0(pos.Y), Z0(pos.Z), dX(vec.X), dY(vec.Y), dZ(vec.Z) {};

	iplPOINT3D origin() { return iplPOINT3D(X0, Y0, Z0); };
	iplPOINT3D direction() { return iplPOINT3D(dX, dY, dZ); };

	void IntersectWithZ(double Z, double *X, double *Y)
	{
		double t = (Z - Z0) / dZ;
		*X = X0 + t*dX;
		*Y = Y0 + t*dY;
	}
};
// 光束
typedef iplLINE3D iplRAY;



struct	iplPLANE {
public:
	double X0, Y0, Z0;			// origin
	double A, B, C;				// normal line，法线的方向余弦A*A+B*B+C*C=1

public:

	bool IntersectWithLine(const iplLINE3D &line, double *X, double *Y, double *Z)
	{
		double s = A*line.dX + B*line.dY + C*line.dZ;

		if (fabs(s) < 1e-16)
			return false;

		double t = (A*(X0 - line.X0) + B*(Y0 - line.Y0) + C*(Z0 - line.Z0)) / s;

		*X = line.X0 + t*line.dX;
		*Y = line.Y0 + t*line.dY;
		*Z = line.Z0 + t*line.dZ;

		return true;
	}
};

struct iplSIZE {
	ipl_int32 cx;
	ipl_int32 cy;

	iplSIZE() :cx(0), cy(0) {};
	iplSIZE(ipl_int32 cx1, ipl_int32 cy1) :cx(cx1), cy(cy1) {};
};


// 预定义变形模式
enum iplWarpMODE {
	IPL_wmNONE = 0,				// 不做变形
	IPL_wmTRANSLATION,			// 平移				"ipl.image.source.warp.translation"
	IPL_wmRIGID,				// 刚性				"ipl.image.source.warp.rigid"
	IPL_wmSIMILARITY,			// 相似变换			"ipl.image.source.warp.similarity"
	IPL_wmAFFINE,				// 仿射变换			"ipl.image.source.warp.affine"
	IPL_wmPROJECTIVE,			// 二维DLT			"ipl.image.source.warp.projective"
	IPL_wmBILINEAR,				// 双线性			"ipl.image.source.warp.bilinear"
	IPL_wmQuadPOLY				// 二次多项式		"ipl.image.source.warp.quadpoly"
};


// 预定义采样模式
enum iplResampleMODE {
	IPL_rsmNEAREST = 0,	// 最邻近	"ipl.image.resample.neareast"
	IPL_rsmBILINEAR,	// 双线性	"ipl.image.resample.bilinear"
	IPL_rsmBICUBIC,		// 双三次	"ipl.image.resample.bicubic", 4点, 双三次
	IPL_rsmBICUBIC_6P,			// 6点, 双三次
	IPL_rsmTruncted_SINC_6P,	// 6点截断sinc
	IPL_rsmKNAB_6P,			// 6点Knab
	IPL_rsmRaised_COS_6P,		// 6点升余弦
	IPL_rsmBSPLINE 			// UNSER B样条
							//ipl_rsmBSPLINE	// B样条	"ipl.image.resample.bspline"
};

#ifndef SWIG
#ifdef IPL_PLATFORM_WINDOWS
#define IPL_EXPORT __declspec(dllexport)
#define IPL_IMPORT __declspec(dllimport)
#define IPL_HIDDEN
#else
#define IPL_EXPORT __attribute__ ((visibility ("default")))
#define IPL_HIDDEN __attribute__ ((visibility ("hidden")))
#define IPL_IMPORT
#endif
#else
#define IPL_HIDDEN
#define IPL_EXPORT
#define IPL_IMPORT
#endif

#ifdef IPLBASE_EXPORTS   //for system developers or api developers
#define IPL_BASE_API					IPL_EXPORT
#else
#define IPL_BASE_API					IPL_IMPORT
#endif




//file format
#include "core/iplstd.h"
struct iplFileFormat
	{
		std::string name;
		std::string ext;
	};

typedef std::vector<iplFileFormat> iplFileFormatList;


//log
enum iplLogLEVEL {
	IPL_LOG_DEBUG,
	IPL_LOG_INFO,
	IPL_LOG_WARNING,
	IPL_LOG_ERROR,
	IPL_LOG_FATAL
};

#define  IPL_MAX_LOG_ITEM_SIZE  2048

//data source type
enum iplDSType {
	IPL_DS_POINTCLOUD,
	IPL_DS_IMAGE,
	IPL_DS_SIMPLEFEATURE
};

namespace ipl
{
#undef iplDEG2RAD
#define iplDEG2RAD 0.01745329251994329576923690768489
//#endif

#undef iplRAD2DEG
#define iplRAD2DEG 57.295779513082320876846364344191
//#endif
}
