#pragma once

#include "mLibInclude.h"
#include "algo/ceres_math.h"
#include <vector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct RotationCostFunction {
	RotationCostFunction()
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create() {
		return (new ceres::AutoDiffCostFunction<RotationCostFunction, 1, 9>(new RotationCostFunction()));
	}

	// E_fit = sum_{i} (c1*c2)^2+(c1*c3)^2+(c2*c3)^2+(c1*c1-1)^2+(c2*c2-1)^2+(c3*c3-1)^2
	template <typename T>
	bool operator()(const T* const rotation_matrix, T* residuals) const
	{
		const T *c1 = rotation_matrix;
		const T *c2 = rotation_matrix + 3;
		const T *c3 = rotation_matrix + 6;
		T one = T(1.);
		T dot_c1_c2 = dot(c1, c2);

		residuals[0] = pow2(dot(c1, c2)) + pow2(dot(c1, c3)) + pow2(dot(c1, c3))
			+ pow2(dot(c1, c1) - one) + pow2(dot(c2, c2) - one) + pow2(dot(c3, c3) - one);		
		return true;
	}
};

struct SmoothCostFunction {
	const ml::vec3f& _vi;
	const ml::vec3f& _vj;

	SmoothCostFunction(const ml::vec3f & vi, const ml::vec3f & vj)
		: _vi(vi)
		, _vj(vj)
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f & vi, const ml::vec3f & vj) {
		return (new ceres::AutoDiffCostFunction<SmoothCostFunction, 3, 9, 3, 3>(new SmoothCostFunction(vi, vj)));
	}

	// Esmooth = sum_i sum_j || Ai(xj-xi) + xi + bi - (xj + bj) ||^2
	template <typename T>
	bool operator()(const T* const rotation_matrix, const T* const translation_i, const T* const translation_j, T* residuals) const
	{
		T vi[3];
		T vj[3];
		vec3f_to_T(_vi, vi);
		vec3f_to_T(_vj, vj);
		T edge[3];
		T vi_t[3];
		T vj_t[3];
		T tmp[3];

		substract(vj, vi, edge);
		matrix_multiplication(rotation_matrix, edge, tmp);

		addition(vi, translation_i, vi_t);
		addition(vj, translation_j, vj_t);

		addition(tmp, vi_t, tmp);
		substract(tmp, vj_t, residuals);

		//addition(result, vi, result);
		//addition(result, translation_i, result);
		//substract(result, vj, result);
		//substract(result, translation_j, result);

		//residuals[0] = result[0];
		//residuals[1] = result[1];
		//residuals[2] = result[2];
		return true;
	}
};
