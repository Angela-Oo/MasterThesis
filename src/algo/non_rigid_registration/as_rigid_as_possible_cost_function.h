#pragma once

#include "../../mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>


struct AsRigidAsPossibleCostFunction {
	const ml::vec3f & _v_i; // vi
	const ml::vec3f & _v_j; // vj

	AsRigidAsPossibleCostFunction(const ml::vec3f & v_i, const ml::vec3f & v_j)
		: _v_i(v_i)
		, _v_j(v_j)
	{
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f & v_i, const ml::vec3f & v_j) {
		return (new ceres::AutoDiffCostFunction<AsRigidAsPossibleCostFunction, 3, 9, 3, 3>(new AsRigidAsPossibleCostFunction(v_i, v_j)));
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
		T tmp[3];

		substract(vj, vi, edge);

		T rotation_angle_axis[3];//
		ceres::RotationMatrixToAngleAxis(rotation, rotation_angle_axis);//
		ceres::AngleAxisRotatePoint(rotation_angle_axis, edge, rotated_edge);//
		//ceres::AngleAxisRotatePoint(rotation, edge, rotated_edge);

		addition(vi, translation_i, vi_t);
		addition(vj, translation_j, vj_t);

		substract(vj_t, vi_t, tmp);
		substract(rotated_edge, tmp, residuals);

		//T edge_src[3];		
		//substract(vi, vj, edge_src);

		//ceres::AngleAxisRotatePoint(rotation, edge_src, edge_src);
		//
		//T tj[3];
		//vec3f_to_T(_t_j, tj);
		//T v_dst_i[3];
		//T v_dst_j[3];
		//T edge_dst[3];
		//addition(vi, translation, v_dst_i);
		//addition(vj, tj, v_dst_j);
		//substract(v_dst_i, v_dst_j, edge_dst);

		//// The error is the difference between the predicted and observed position.
		//residuals[0] = edge_dst[0] - edge_src[0];
		//residuals[1] = edge_dst[1] - edge_src[1];
		//residuals[2] = edge_dst[2] - edge_src[2];

		return true;
	}
};

