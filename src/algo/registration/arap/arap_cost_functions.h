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
		return (new ceres::AutoDiffCostFunction<AsRigidAsPossibleCostFunction, 3, 6, 6>(new AsRigidAsPossibleCostFunction(v_i, v_j)));
	}

	// E_arap = sum_{i} sum_{j in N} |(vi-vj) - Ri(vi' - vj')|^2
	// E_arap = sum_{i} sum_{j in N} |Ri(vj-vi) - ((vj + tj) - (vi + ti))|^2
	template <typename T>
	bool operator()(const T* const deformation_i, const T* const deformation_j, T* residuals) const
	{
		T vi[3];
		T vj[3];
		point_to_T(_v_i, vi);
		point_to_T(_v_j, vj);

		T rotation_i[3];
		rotation_i[0] = deformation_i[0];
		rotation_i[1] = deformation_i[1];
		rotation_i[2] = deformation_i[2];

		T translation_i[3];
		translation_i[0] = deformation_i[3];
		translation_i[1] = deformation_i[4];
		translation_i[2] = deformation_i[5];

		T translation_j[3];
		translation_j[0] = deformation_j[3];
		translation_j[1] = deformation_j[4];
		translation_j[2] = deformation_j[5];

		T edge[3];
		T rotated_edge[3];
		T vi_t[3];
		T vj_t[3];
		T transformed_edge[3];

		substract(vj, vi, edge);
		ceres::AngleAxisRotatePoint(rotation_i, edge, rotated_edge);

		addition(vi, translation_i, vi_t);
		addition(vj, translation_j, vj_t);

		substract(vj_t, vi_t, transformed_edge);

		//normalize(rotated_edge, rotated_edge);
		//normalize(transformed_edge, transformed_edge);
		
		substract(rotated_edge, transformed_edge, residuals);
		return true;
	}
};


// deformation of vi at node j = (Rj(vi-gj) + gj + tj)
template<typename T>
void deform_point_at_node(const T * const point, const T * const node_pos, const T * const deformation, T* result)
{
	T rotation[3];
	rotation[0] = deformation[0];
	rotation[1] = deformation[1];
	rotation[2] = deformation[2];

	T translation[3];
	translation[0] = deformation[3];
	translation[1] = deformation[4];
	translation[2] = deformation[5];

	T rotated_point[3];
	T moved_point[3];

	// edge rotation = Rj(vi - gj)
	substract(point, node_pos, rotated_point);
	ceres::AngleAxisRotatePoint(rotation, rotated_point, rotated_point);

	// translation of node j = gj + tj
	addition(node_pos, translation, moved_point);

	// deformation of vi at node j = Rj(vi-gj) + gj + tj
	addition(moved_point, rotated_point, result);
}


// deformation of vi at node j = wj(vi) * (Rj(vi-gj) + gj + tj)
template<typename T>
void deform_point_at_node(const T * const point, const T * const node_pos, const T * const deformation, T weight, T* result)
{
	T deformed_point[3];

	deform_point_at_node(point, node_pos, deformation, deformed_point);

	// weighted deformation
	scalar_multiply(deformed_point, weight, result);
}

template<typename T>
void defom_point(const T * const point, 
				 const T* const global_pos,
				 const T* const global_deformation,
				 const T* const n1_pos,
				 const T* const n1_deformation,
				 T n1_weight,
				 const T* const n2_pos,
				 const T* const n2_deformation,
				 T n2_weight,
				 const T* const n3_pos,
				 const T* const n3_deformation,
				 T n3_weight,
				 const T* const n4_pos,
				 const T* const n4_deformation,
				 T n4_weight,
				 T* result)
{
	T deformed_point[3];
	T weighted_deformed_point[3];

	// local deformation of node position
	deform_point_at_node(point, n1_pos, n1_deformation, n1_weight, deformed_point);

	deform_point_at_node(point, n2_pos, n2_deformation, n2_weight, weighted_deformed_point);
	addition(deformed_point, weighted_deformed_point, deformed_point);

	deform_point_at_node(point, n3_pos, n3_deformation, n3_weight, weighted_deformed_point);
	addition(deformed_point, weighted_deformed_point, deformed_point);

	deform_point_at_node(point, n4_pos, n4_deformation, n4_weight, weighted_deformed_point);
	addition(deformed_point, weighted_deformed_point, deformed_point);

	// global deformation of position
	deform_point_at_node(deformed_point, global_pos, global_deformation, result);
}



struct FitStarPointToPointAngleAxisCostFunction {
	const Point _target;
	const Point _source;
	const Point _global_pos;
	const Point _n1_pos;
	const Point _n2_pos;
	const Point _n3_pos;
	const Point _n4_pos;
	const double _n1_weight;
	const double _n2_weight;
	const double _n3_weight;
	const double _n4_weight;

	FitStarPointToPointAngleAxisCostFunction(const Point &target, const Point &source, const Point &global_pos,
											 const Point n1_pos, const Point n2_pos, const Point n3_pos, const Point n4_pos,
											 double n1_weight, double n2_weight, double n3_weight, double n4_weight)
		: _target(target), _source(source), _global_pos(global_pos)
		, _n1_pos(n1_pos), _n2_pos(n2_pos), _n3_pos(n3_pos), _n4_pos(n4_pos)
		, _n1_weight(n1_weight), _n2_weight(n2_weight), _n3_weight(n3_weight), _n4_weight(n4_weight)
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point &target, const Point &source, const Point &global_pos,
									   const Point n1_pos, const Point n2_pos, const Point n3_pos, const Point n4_pos,
									   double n1_weight, double n2_weight, double n3_weight, double n4_weight) {
		return (new ceres::AutoDiffCostFunction<FitStarPointToPointAngleAxisCostFunction, 3, 6, 6, 6, 6, 6>(
			new FitStarPointToPointAngleAxisCostFunction(target, source, global_pos, n1_pos, n2_pos, n3_pos, n4_pos, n1_weight, n2_weight, n3_weight, n4_weight)));
	}

	template <typename T>
	bool operator()(const T* const global_deformation, 
					const T* const n1_deformation,
					const T* const n2_deformation,
					const T* const n3_deformation,
					const T* const n4_deformation,
					T* residuals) const
	{
		T source[3];
		T global_pos[3];
		T n1_pos[3];
		T n2_pos[3];
		T n3_pos[3];
		T n4_pos[3];
		T target[3];
		point_to_T(_source, source);
		point_to_T(_global_pos, global_pos);
		point_to_T(_n1_pos, n1_pos);
		point_to_T(_n2_pos, n2_pos);
		point_to_T(_n3_pos, n3_pos);
		point_to_T(_n4_pos, n4_pos);
		point_to_T(_target, target);

		T deformed_point[3];
		defom_point(source,
					global_pos, global_deformation,
					n1_pos, n1_deformation, T(_n1_weight),
					n2_pos, n2_deformation, T(_n2_weight),
					n3_pos, n3_deformation, T(_n3_weight),
					n4_pos, n4_deformation, T(_n4_weight),
					deformed_point);

		// The error is the difference between the deformed source position and the target position multiplied with the weight
		substract(deformed_point, target, residuals);
		//scalar_multiply(residuals, w[0], residuals);
		return true;
	}
};


struct FitStarPointToPlaneAngleAxisCostFunction {
	const Point _target_pos;
	const Vector _target_normal;
	const Point _source_pos;
	const Point _global_pos;
	const Point _n1_pos;
	const Point _n2_pos;
	const Point _n3_pos;
	const Point _n4_pos;
	const double _n1_weight;
	const double _n2_weight;
	const double _n3_weight;
	const double _n4_weight;

	FitStarPointToPlaneAngleAxisCostFunction(const Point& target_pos, const Vector & target_normal,
											 const Point& source_pos, const Point &global_pos,
											 const Point n1_pos, const Point n2_pos, const Point n3_pos, const Point n4_pos,
											 double n1_weight, double n2_weight, double n3_weight, double n4_weight)
		: _target_pos(target_pos), _target_normal(target_normal)
		, _source_pos(source_pos), _global_pos(global_pos)
		, _n1_pos(n1_pos), _n2_pos(n2_pos), _n3_pos(n3_pos), _n4_pos(n4_pos)
		, _n1_weight(n1_weight), _n2_weight(n2_weight), _n3_weight(n3_weight), _n4_weight(n4_weight)
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point& target_pos, const Vector & target_normal,
									   const Point& source_pos, const Point &global_pos,
									   const Point n1_pos, const Point n2_pos, const Point n3_pos, const Point n4_pos,
									   double n1_weight, double n2_weight, double n3_weight, double n4_weight)
	{
		return (new ceres::AutoDiffCostFunction<FitStarPointToPlaneAngleAxisCostFunction, 1, 6, 6, 6, 6, 6>(
			new FitStarPointToPlaneAngleAxisCostFunction(target_pos, target_normal, source_pos, global_pos, n1_pos, n2_pos, n3_pos, n4_pos, n1_weight, n2_weight, n3_weight, n4_weight)));
	}

	template <typename T>
	bool operator()(const T* const global_deformation,
					const T* const n1_deformation,
					const T* const n2_deformation,
					const T* const n3_deformation,
					const T* const n4_deformation,
					T* residuals) const
	{
		T source_pos[3];
		T global_pos[3];
		T n1_pos[3];
		T n2_pos[3];
		T n3_pos[3];
		T n4_pos[3];
		T target_pos[3];
		T target_normal[3];

		point_to_T(_source_pos, source_pos);
		point_to_T(_global_pos, global_pos);
		point_to_T(_n1_pos, n1_pos);
		point_to_T(_n2_pos, n2_pos);
		point_to_T(_n3_pos, n3_pos);
		point_to_T(_n4_pos, n4_pos);
		point_to_T(_target_pos, target_pos);
		point_to_T(_target_normal, target_normal);

		T deformed_point[3];
		defom_point(source_pos,
					global_pos, global_deformation,
					n1_pos, n1_deformation, T(_n1_weight),
					n2_pos, n2_deformation, T(_n2_weight),
					n3_pos, n3_deformation, T(_n3_weight),
					n4_pos, n4_deformation, T(_n4_weight),
					deformed_point);



		// Point to plane error = dot((deformed position - target_point), target_normal 
		T difference[3];
		substract(deformed_point, target_pos, difference);
		residuals[0] = dot(difference, target_normal);// *w[0];

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