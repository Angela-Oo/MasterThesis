#pragma once

#include "../mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "ceres_math.h"


struct EmbeddedDeformationCostFunction {
	const ml::vec3f& _edge_src; // vi - vj
	const ml::vec3f& _edge_dst; // vi' - vj'

	EmbeddedDeformationCostFunction(const ml::vec3f & dst_i, const ml::vec3f & dst_j, const ml::vec3f & src_i, const ml::vec3f & src_j) :
		_edge_dst(dst_i - dst_j), _edge_src(src_i - src_j)
	{
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f & dst_i, const ml::vec3f & dst_j, const ml::vec3f & src_i, const ml::vec3f & src_j) {
		return (new ceres::AutoDiffCostFunction<EmbeddedDeformationCostFunction, 3, 9>(new EmbeddedDeformationCostFunction(dst_i, dst_j, src_i, src_j)));
	}

	// E_arap = sum_{i} sum_{j in N} |(vi-vj) - Ri(vi' - vj')|^2
	template <typename T>
	bool operator()(const T* const deformation_matrix, T* residuals) const 
	{
		T edge_dst[3] = { T(_edge_dst[0]), T(_edge_dst[1]), T(_edge_dst[2]) };
		multiply(deformation_matrix, edge_dst, edge_dst);

		// The error is the difference between the predicted and observed position.
		residuals[0] = T(_edge_src[0]) - edge_dst[0];
		residuals[1] = T(_edge_src[1]) - edge_dst[1];
		residuals[2] = T(_edge_src[2]) - edge_dst[2];

		return true;
	}
};


struct EmbeddedDeformationPointsCostFunction {
	const ml::vec3f _edge_src; // vi - vj
	const ml::mat3d & _matrix;
	const ml::vec3f _dst_j; // vj'

	EmbeddedDeformationPointsCostFunction(const ml::vec3f & src_i, const ml::vec3f & src_j, const ml::mat3d& matrix, const ml::vec3f& dst_j)
		: _edge_src(src_i - src_j)
		, _dst_j(dst_j)
		, _matrix(matrix)
	{
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f & src_i, const ml::vec3f & src_j, const ml::mat3d& matrix, const ml::vec3f& dst_j) {
		return (new ceres::AutoDiffCostFunction<EmbeddedDeformationPointsCostFunction, 3, 3>(new EmbeddedDeformationPointsCostFunction(src_i, src_j, matrix, dst_j)));
	}

	// E_arap = sum_{i} sum_{j in N} |(vi-vj) - Ri(vi' - vj')|^2
	template <typename T>
	bool operator()(const T* const point, T* residuals) const
	{
		T dst_j[3];
		vec3f_to_T<T>(_dst_j, dst_j);
		T edge_dst[3];
		edge_dst[0] = point[0] - dst_j[0];
		edge_dst[1] = point[1] - dst_j[1];
		edge_dst[2] = point[2] - dst_j[2];

		T deformation_matrix[9];
		mat3d_to_T<T>(_matrix.getData(), deformation_matrix);
		multiply(deformation_matrix, edge_dst, edge_dst);

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


	template <typename T>
	bool operator()(const T* const rotation_matrix, const T* const bi, const T* const bj, T* residuals) const
	{
		T vi[3];
		T vj[3];
		vec3f_to_T(_vi, vi);
		vec3f_to_T(_vj, vj);
		T edge[3];
		T result[3];

		substract(vj, vi, edge);
		multiply(rotation_matrix, edge, result);
		addition(result, vi, result);
		addition(result, bj, result);
		substract(result, vj, result);
		substract(result, bj, result);

		residuals[0] = result[0];
		residuals[1] = result[1];
		residuals[2] = result[2];
		return true;
	}
};


struct FitStarCostFunction {
	const ml::vec3f& _v;
	const ml::vec3f& _n;
	const ml::vec3f& _g;

	FitStarCostFunction(const ml::vec3f &v, const ml::vec3f &n, const ml::vec3f &g) :
		_v(v), _n(n), _g(g)
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f &v, const ml::vec3f &n, const ml::vec3f &g) {
		return (new ceres::AutoDiffCostFunction<FitStarCostFunction, 3, 9, 3, 3>(new FitStarCostFunction(v, n, g)));
	}

	template <typename T>
	bool operator()(const T* const global_rotation, const T* const global_translation, const T* const translation, T* residuals) const
	{
		T n[3] = { T(_n[0]), T(_n[1]), T(_n[2]) };
		T g[3] = { T(_g[0]), T(_g[1]), T(_g[2]) };

		// local deformation of node position
		addition(n, translation, n);

		// global deformation of node position
		substract(n, g, n);
		multiply(global_rotation, n, n);
		addition(n, g, n);
		addition(n, global_translation, n);

		// The error is the difference between the predicted and observed position.
		residuals[0] = n[0] - T(_v[0]);
		residuals[1] = n[1] - T(_v[1]);
		residuals[2] = n[2] - T(_v[2]);

		return true;
	}
};


template<typename T>
void deform(const T * const p, const T * const g, const T * const r, const T * const t, const T const w, T* result)
{
	substract(p, g, result);
	multiply(r, result, result);
	addition(result, g, result);
	addition(result, t, result);
	result[0] *= w;
	result[1] *= w;
	result[2] *= w;
}


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
		residuals[0] = one - weight[0] * weight[0];
		return true;
	}
};