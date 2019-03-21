#pragma once

#include "../mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>
//#include <ceres/internal/eigen.h>
#include <ceres/rotation.h>


struct EmbeddedDeformationCostFunction {
	const ml::vec3f& _edge_src; // vi - vj
	const ml::vec3f& _edge_dst; // vi' - vj'

	EmbeddedDeformationCostFunction(const ml::vec3f & dst_i, const ml::vec3f & dst_j, const ml::vec3f & src_i, const ml::vec3f & src_j) :
		_edge_dst(dst_i - dst_j), _edge_src(src_i - src_j)
	{
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f & dst_i, const ml::vec3f & dst_j, const ml::vec3f & src_i, const ml::vec3f & src_j) {
		return (new ceres::AutoDiffCostFunction<EmbeddedDeformationCostFunction, 9, 3>(new EmbeddedDeformationCostFunction(dst_i, dst_j, src_i, src_j)));
	}

	// E_arap = sum_{i} sum_{j in N} |(vi-vj) - Ri(vi' - vj')|^2
	template <typename T>
	bool operator()(const T* const deformation_matrix, T* residuals) const 
	{
		T edge_dst[3] = { T(_edge_dst[0]), T(_edge_dst[1]), T(_edge_dst[2]) };
		edge_dst = deformation_matrix * edge_dst; // todo array[9] to matrix

		// The error is the difference between the predicted and observed position.
		residuals[0] = T(_edge_src[0]) - edge_dst[0];
		residuals[1] = T(_edge_src[1]) - edge_dst[1];
		residuals[2] = T(_edge_src[2]) - edge_dst[2];

		return true;
	}
};


struct RotationCostFunction {
	RotationCostFunction()
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f & v_dst, const ml::vec3f & v_src) {
		return (new ceres::AutoDiffCostFunction<RotationCostFunction, 9, 1>(new RotationCostFunction()));
	}
	template <typename T>
	T dot(const T* const v1, const T* const v2) {
		return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
	}

	// E_fit = sum_{i} (c1*c2)^2+(c1*c3)^2+(c2*c3)^2+(c1*c1-1)^2+(c2*c2-1)^2+(c3*c3-1)^2
	template <typename T>
	bool operator()(const T* const rotation_matrix, T* residuals) const
	{
		ceres::ColumnMajorAdapter3x3 matrix(rotation_matrix);
		T c1[3] = matrix[0];
		T c2[3] = matrix[1];
		T c3[3] = matrix[2];

		residuals[0] = pow(dot(c1, c2), 2) + pow(dot(c1, c3), 2) + pow(dot(c2, c3), 2) 
			+ pow((dot(c1, c1) -1), 2) + pow((dot(c2, c2) - 1), 2) + pow((dot(c3, c3) - 1), 2);

		return true;
	}
};

