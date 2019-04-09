#pragma once

#include "../../mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>


struct AsRigidAsPossibleCostFunction {
	const ml::vec3f _edge_src; // vi - vj
	const ml::vec3f _edge_dst; // vi' - vj'

	AsRigidAsPossibleCostFunction(const ml::vec3f & dst_i, const ml::vec3f & dst_j, const ml::vec3f & src_i, const ml::vec3f & src_j) :
		_edge_dst(dst_i - dst_j), _edge_src(src_i - src_j)
	{
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f & dst_i, const ml::vec3f & dst_j, const ml::vec3f & src_i, const ml::vec3f & src_j) {
		return (new ceres::AutoDiffCostFunction<AsRigidAsPossibleCostFunction, 3, 3>(new AsRigidAsPossibleCostFunction(dst_i, dst_j, src_i, src_j)));
	}

	// E_arap = sum_{i} sum_{j in N} |(vi-vj) - Ri(vi' - vj')|^2
	template <typename T>
	bool operator()(const T* const rotation, T* residuals) const 
	{
		T edge_src[3] = { T(_edge_src[0]), T(_edge_src[1]), T(_edge_src[2]) };
		ceres::AngleAxisRotatePoint(rotation, edge_src, edge_src);

		// The error is the difference between the predicted and observed position.
		residuals[0] = T(_edge_dst[0]) - edge_src[0];
		residuals[1] = T(_edge_dst[1]) - edge_src[1];
		residuals[2] = T(_edge_dst[2]) - edge_src[2];

		return true;
	}
};


struct AsRigidAsPossiblePointCostFunction {
	const ml::vec3f _edge_src; // vi - vj
	const ml::vec3f _dst_j; // vj'
	const ml::vec3d _rotation;

	AsRigidAsPossiblePointCostFunction(const ml::vec3f & src_i, const ml::vec3f & src_j, const ml::vec3d & rotation, const ml::vec3f & dst_j) :
		_dst_j(dst_j), _edge_src(src_i - src_j), _rotation(rotation)
	{
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f & src_i, const ml::vec3f & src_j, const ml::vec3d & rotation, const ml::vec3f & dst_j) {
		return (new ceres::AutoDiffCostFunction<AsRigidAsPossiblePointCostFunction, 3, 3>(new AsRigidAsPossiblePointCostFunction(src_i, src_j, rotation, dst_j)));
	}

	// E_arap = sum_{i} sum_{j in N} |(vi-vj) - Ri(vi' - vj')|^2
	template <typename T>
	bool operator()(const T* const point, T* residuals) const
	{
		T edge_src[3] = { T(_edge_src[0]), T(_edge_src[1]), T(_edge_src[2]) };
		T rotation[3] = { T(_rotation[0]), T(_rotation[1]), T(_rotation[2]) };
		ceres::AngleAxisRotatePoint(rotation, edge_src, edge_src);

		// The error is the difference between the predicted and observed position.
		residuals[0] = (point[0] - T(_dst_j[0])) - edge_src[0];
		residuals[1] = (point[1] - T(_dst_j[1])) - edge_src[1];
		residuals[2] = (point[2] - T(_dst_j[2])) - edge_src[2];

		return true;
	}
};


struct FitCostFunction {
	const ml::vec3f& _v_src; // vi

	FitCostFunction(const ml::vec3f & v_src) :
		_v_src(v_src)
	{
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f & v_src) {
		return (new ceres::AutoDiffCostFunction<FitCostFunction, 3, 3>(new FitCostFunction(v_src)));
	}

	// E_fit = sum_{i in C} |vi'-vi|^2
	template <typename T>
	bool operator()(const T* const point, T* residuals) const
	{
		// The error is the difference between the predicted and observed position.
		residuals[0] = T(point[0]) - T(_v_src[0]);
		residuals[1] = T(point[1]) - T(_v_src[1]);
		residuals[2] = T(point[2]) - T(_v_src[2]);

		return true;
	}
};

