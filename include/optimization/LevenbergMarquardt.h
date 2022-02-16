#pragma once
#include <unsupported/Eigen/NonLinearOptimization>

namespace ipl
{
	// Generic functor
	template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
	struct LM_Functor
	{
		typedef _Scalar Scalar;
		enum {
			InputsAtCompileTime = NX,
			ValuesAtCompileTime = NY
		};
		typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
		typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
		typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

		const int m_inputs, m_values;

		LM_Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
		LM_Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

		int inputs() const { return m_inputs; }   //unknowns
		int values() const { return m_values; }   //observations

		// you should define that in the subclass :
		//  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
	};

#ifndef iplsqr
#define iplsqr(x) ((x)*(x))
#endif
}
