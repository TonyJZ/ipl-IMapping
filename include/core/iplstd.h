#pragma once

// #include <cstddef>
// #include <cstring>
// #include <cctype>

#include <string>
#include <vector>

// import useful primitives from stl
#include <algorithm>
#include <utility>
#include <cstdlib> //for abs(int)
#include <cmath>

// import useful primitives from boost
#include <boost/shared_ptr.hpp>


// import useful primitives from Eigen
#include <Eigen/Dense>

namespace ipl
{
// 	static inline uchar abs(uchar a) { return a; }
// 	static inline ushort abs(ushort a) { return a; }
// 	static inline unsigned abs(unsigned a) { return a; }
// 	static inline uint64 abs(uint64 a) { return a; }

	using std::min;
	using std::max;
	using std::abs;
	using std::swap;
	using std::sqrt;
	using std::exp;
	using std::pow;
	using std::log;

// #ifndef sqr
// #define sqr(x) (x*(x))
// #endif

#if defined(_UNICODE) || defined(UNICODE)
	using iplString = std::wstring;
#else
	using iplString = std::string;
#endif

	
	template<typename _T>
	using iplArray = std::vector<_T>;

	template<typename _T>
	using ref_ptr = boost::shared_ptr<_T>;

	template<typename T, typename U> ref_ptr<T> ipl_static_pointer_cast(ref_ptr<U> const & r)
	{
		(void) static_cast<T*>(static_cast<U*>(0));

		typedef typename ref_ptr<T>::element_type E;

		E * p = static_cast<E*>(r.get());
		return ref_ptr<T>(r, p);
	};

	template<typename T, typename U> ref_ptr<T> ipl_dynamic_pointer_cast(ref_ptr<U> const & r)
	{
		(void) dynamic_cast<T*>(static_cast<U*>(0));

		typedef typename ref_ptr<T>::element_type E;

		E * p = dynamic_cast<E*>(r.get());
		return p ? ref_ptr<T>(r, p) : ref_ptr<T>();
	};

};

