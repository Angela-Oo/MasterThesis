#pragma once

#include "../../mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>


struct AsRigidAsPossibleCostFunction {
	const ml::vec3f _v_i; // vi
	const ml::vec3f _v_j; // vj

	AsRigidAsPossibleCostFunction(const ml::vec3f & v_i, const ml::vec3f & v_j)
		: _v_i(v_i)
		, _v_j(v_j)
	{
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f & v_i, const ml::vec3f & v_j) {
		return (new ceres::AutoDiffCostFunction<AsRigidAsPossibleCostFunction, 3, 3, 3, 3>(new AsRigidAsPossibleCostFunction(v_i, v_j)));
	}

	// E_arap = sum_{i} sum_{j in N} |(vi-vj) - Ri(vi' - vj')|^2
	// E_arap = sum_{i} sum_{j in N} |Ri(vj-vi) - ((vj + tj) - (vi + ti))|^2
	template <typename T>
	bool operator()(const T* const rotation, const T* const translation_i, const T* const translation_j, T* residuals) const
	{
		T vi[3];
		T vj[3];
		vec3f_to_T(_v_i, vi);
		vec3f_to_T(_v_j, vj);		
		T edge[3];
		T rotated_edge[3];
		T vi_t[3];
		T vj_t[3];
		T transformed_edge[3];

		substract(vj, vi, edge);

		ceres::AngleAxisRotatePoint(rotation, edge, rotated_edge);

		addition(vi, translation_i, vi_t);
		addition(vj, translation_j, vj_t);

		substract(vj_t, vi_t, transformed_edge);
		substract(rotated_edge, transformed_edge, residuals);

		return true;
	}
};

