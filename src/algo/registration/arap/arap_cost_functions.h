#pragma once

#include "algo/surface_mesh/mesh_definition.h"
#include "algo/ceres_math.h"
#include <vector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace ARAP
{

struct AsRigidAsPossibleCostFunction {
	const Point _v_i; // vi
	const Point _v_j; // vj

	AsRigidAsPossibleCostFunction(const Point & v_i, const Point & v_j)
		: _v_i(v_i)
		, _v_j(v_j)
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point & v_i, const Point & v_j) {
		return (new ceres::AutoDiffCostFunction<AsRigidAsPossibleCostFunction, 3, 3, 3, 3>(new AsRigidAsPossibleCostFunction(v_i, v_j)));
	}

	// E_arap = sum_{i} sum_{j in N} |(vi-vj) - Ri(vi' - vj')|^2
	// E_arap = sum_{i} sum_{j in N} |Ri(vj-vi) - ((vj + tj) - (vi + ti))|^2
	template <typename T>
	bool operator()(const T* const rotation, const T* const translation_i, const T* const translation_j, T* residuals) const
	{
		T vi[3];
		T vj[3];
		point_to_T(_v_i, vi);
		point_to_T(_v_j, vj);
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





struct FitStarPointToPointAngleAxisCostFunction {
	const Point _point;
	const Point _node_g;
	const Point _global_g;

	FitStarPointToPointAngleAxisCostFunction(const Point &point, const Point &node_g, const Point &global_g) :
		_point(point), _node_g(node_g), _global_g(global_g)
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point &point, const Point &node_g, const Point &global_g) {
		return (new ceres::AutoDiffCostFunction<FitStarPointToPointAngleAxisCostFunction, 3, 3, 3, 3, 1>(new FitStarPointToPointAngleAxisCostFunction(point, node_g, global_g)));
	}

	template <typename T>
	bool operator()(const T* const global_rotation, const T* const global_translation, const T* const translation, const T* const w, T* residuals) const
	{
		T node_g[3];
		T global_g[3];
		T point[3];
		point_to_T(_node_g, node_g);
		point_to_T(_global_g, global_g);
		point_to_T(_point, point);

		T deformed_node[3];
		// local deformation of node position
		addition(node_g, translation, deformed_node);

		// global deformation of node position
		substract(deformed_node, global_g, deformed_node);
		ceres::AngleAxisRotatePoint(global_rotation, deformed_node, deformed_node);
		addition(deformed_node, global_g, deformed_node);
		addition(deformed_node, global_translation, deformed_node);

		// The error is the difference between the predicted and observed position multiplied with the weight
		substract(deformed_node, point, residuals);
		scalar_multiply(residuals, w[0], residuals);
		return true;
	}
};




struct FitStarPointToPlaneAngleAxisCostFunction {
	const Point _target_point;
	const Vector _target_normal;
	const Point _node_g;	
	const Point _global_g;

	FitStarPointToPlaneAngleAxisCostFunction(const Point& target_point, const Vector & target_normal, const Point& node_g, const Point &global_g) :
		_target_point(target_point), _target_normal(target_normal), _node_g(node_g),  _global_g(global_g)
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point& target_position, const Vector & target_normal, const Point& node_g, const Point &global_g)
	{
		return (new ceres::AutoDiffCostFunction<FitStarPointToPlaneAngleAxisCostFunction, 1, 3, 3, 3, 3, 1>(new FitStarPointToPlaneAngleAxisCostFunction(target_position, target_normal, node_g, global_g)));
	}

	template <typename T>
	bool operator()(const T* const global_rotation, const T* const global_translation, const T* const rotation, const T* const translation, const T* const w, T* residuals) const
	{
		T node_g[3];
		T global_g[3];
		T target_point[3];
		T target_normal[3];
		point_to_T(_node_g, node_g);
		point_to_T(_global_g, global_g);
		point_to_T(_target_point, target_point);
		point_to_T(_target_normal, target_normal);

		T deformed_node[3];
		// local deformation (only translation) of node position
		addition(node_g, translation, deformed_node);

		// global deformation (rotation and translation) of node position
		// rotation
		substract(deformed_node, global_g, deformed_node);
		ceres::AngleAxisRotatePoint(global_rotation, deformed_node, deformed_node);
		addition(deformed_node, global_g, deformed_node);
		// translation
		addition(deformed_node, global_translation, deformed_node);

		// deform the node normal
		//T rotation_t[9];
		//T global_rotation_t[9];
		//ceres::AngleAxisToRotationMatrix(rotation, rotation_t);
		//ceres::AngleAxisToRotationMatrix(global_rotation, global_rotation_t);
		//matrix_transpose(rotation_t, rotation_t);
		//matrix_transpose(global_rotation_t, global_rotation_t);
		//matrix_multiplication(rotation_t, normal, normal);
		//matrix_multiplication(global_rotation_t, normal, normal);
		//normalize(normal, normal);
		//matrix_transpose()

		// The error is the difference between the predicted and observed position.
		T difference[3];
		substract(deformed_node, target_point, difference);
		residuals[0] = dot(difference, target_normal) * w[0];

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