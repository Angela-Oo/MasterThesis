#pragma once

#include "algo/registration/util/ceres_math.h"
#include <vector>
#include "util/ceres_include.h"
#include <ceres/rotation.h>

namespace Registration {
namespace ED
{


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
		//const T *c1 = rotation_matrix;
		//const T *c2 = rotation_matrix + 3;
		//const T *c3 = rotation_matrix + 6;
		T c1[3];
		T c2[3];
		T c3[3];
		c1[0] = rotation_matrix[0];
		c1[1] = rotation_matrix[3];
		c1[2] = rotation_matrix[6];

		c2[0] = rotation_matrix[1];
		c2[1] = rotation_matrix[4];
		c2[2] = rotation_matrix[7];

		c3[0] = rotation_matrix[2];
		c3[1] = rotation_matrix[5];
		c3[2] = rotation_matrix[8];
		T one = T(1.);
		T dot_c1_c2 = dot(c1, c2);
		T dot_c1_c3 = dot(c1, c3);
		T dot_c2_c3 = dot(c2, c3);
		T dot_c1_c1 = dot(c1, c1);
		T dot_c2_c2 = dot(c2, c2);
		T dot_c3_c3 = dot(c3, c3);

		residuals[0] = pow2(dot_c1_c2) + pow2(dot_c1_c3) + pow2(dot_c2_c3)
			+ pow2(dot_c1_c1 - one) + pow2(dot_c2_c2 - one) + pow2(dot_c3_c3 - one);
		return true;
	}
};


struct SmoothCostFunction {
	const Point _vi;
	const Point _vj;

	SmoothCostFunction(const Point & vi, const Point & vj)
		: _vi(vi)
		, _vj(vj)
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point & vi, const Point & vj) {
		return (new ceres::AutoDiffCostFunction<SmoothCostFunction, 3, 9, 3, 3>(new SmoothCostFunction(vi, vj)));
	}

	// Esmooth = sum_i sum_j || Ai(xj-xi) + xi + ti - (xj + tj) ||^2
	template <typename T>
	bool operator()(const T* const rotation_matrix, const T* const translation_i, const T* const translation_j, T* residuals) const
	{
		T vi[3];
		T vj[3];
		point_to_T(_vi, vi);
		point_to_T(_vj, vj);
		T edge_rotated[3];
		T edge_translated[3];

		substract(vi, vj, edge_rotated);
		matrix_multiplication(rotation_matrix, edge_rotated, edge_rotated);

		addition(vi, translation_i, vi);
		addition(vj, translation_j, vj);

		substract(vi, vj, edge_translated);

		substract(edge_rotated, edge_translated, residuals);

		return true;
	}
};




struct ConfCostFunction {

	ConfCostFunction()
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create() {
		return (new ceres::AutoDiffCostFunction<ConfCostFunction, 1, 1>(new ConfCostFunction()));
	}

	template <typename T>
	bool operator()(const T* const weight, T* residuals) const {

		T one{ 1. };
		residuals[0] = one - (weight[0] * weight[0]);
		return true;
	}
};


}
}