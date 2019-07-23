#pragma once

#include "mLibInclude.h"
#include "algo/ceres_math.h"
#include <vector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace Registration {
namespace ED
{

// deformation of vi at node j = (Rj(vi-gj) + gj + tj)
template<typename T>
void deform_point_at_node_embedded_deformation(const T * const point, const T * const node_pos, const T * const deformation, T* result)
{
	T rotation_matrix[9];
	rotation_matrix[0] = deformation[0];
	rotation_matrix[1] = deformation[1];
	rotation_matrix[2] = deformation[2];
	rotation_matrix[3] = deformation[3];
	rotation_matrix[4] = deformation[4];
	rotation_matrix[5] = deformation[5];
	rotation_matrix[6] = deformation[6];
	rotation_matrix[7] = deformation[7];
	rotation_matrix[8] = deformation[8];

	T translation[3];
	translation[0] = deformation[9];
	translation[1] = deformation[10];
	translation[2] = deformation[11];

	T rotated_point[3];
	T moved_point[3];

	// edge rotation = Rj(vi - gj)
	substract(point, node_pos, rotated_point);
	matrix_multiplication(rotation_matrix, rotated_point, rotated_point);

	// translation of node j = gj + tj
	addition(node_pos, translation, moved_point);

	// deformation of vi at node j = Rj(vi-gj) + gj + tj
	addition(moved_point, rotated_point, result);
}

// deformation of vi at node j = (Rj(vi-gj) + gj + tj)
template<typename T>
void deform_point_at_node_embedded_deformation(const T * const point, const Point & node_pos, const T * const deformation, T* result)
{
	T node[3];
	point_to_T(node_pos, node);
	deform_point_at_node_embedded_deformation(point, node, deformation, result);
}


// deformation of vi at node j = wj(vi) * (Rj(vi-gj) + gj + tj)
template<typename T>
void deform_point_at_node_embedded_deformation(const T * const point, const Point & node_pos, const T * const deformation, double weight, T* result)
{
	T node[3];
	point_to_T(node_pos, node);
	T deformed_point[3];
	deform_point_at_node_embedded_deformation(point, node, deformation, deformed_point);
	// weighted deformation
	scalar_multiply(deformed_point, T(weight), result);
}


template<typename T>
void deform_point_embedded_deformation(const T* const point,
									   const Point & global_pos,
									   const T* const global_deformation,
									   const Point & n1_pos,
									   const T* const n1_deformation,
									   double n1_weight,
									   const Point & n2_pos,
									   const T* const n2_deformation,
									   double n2_weight,
									   const Point & n3_pos,
									   const T* const n3_deformation,
									   double n3_weight,
									   const Point & n4_pos,
									   const T* const n4_deformation,
									   double n4_weight,
									   T* result)
{
	T deformed_point[3];
	T weighted_deformed_point[3];

	// local deformation of node position
	deform_point_at_node_embedded_deformation(point, n1_pos, n1_deformation, n1_weight, deformed_point);

	deform_point_at_node_embedded_deformation(point, n2_pos, n2_deformation, n2_weight, weighted_deformed_point);
	addition(deformed_point, weighted_deformed_point, deformed_point);

	deform_point_at_node_embedded_deformation(point, n3_pos, n3_deformation, n3_weight, weighted_deformed_point);
	addition(deformed_point, weighted_deformed_point, deformed_point);

	deform_point_at_node_embedded_deformation(point, n4_pos, n4_deformation, n4_weight, weighted_deformed_point);
	addition(deformed_point, weighted_deformed_point, deformed_point);

	// global deformation of position
	deform_point_at_node_embedded_deformation(deformed_point, global_pos, global_deformation, result);
}

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



struct FitStarPointToPointCostFunction {
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

	FitStarPointToPointCostFunction(const Point &target, const Point &source, const Point &global_pos,
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
		return (new ceres::AutoDiffCostFunction<FitStarPointToPointCostFunction, 3, 12, 12, 12, 12, 12>(
			new FitStarPointToPointCostFunction(target, source, global_pos, n1_pos, n2_pos, n3_pos, n4_pos, n1_weight, n2_weight, n3_weight, n4_weight)));
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
		T target[3];
		point_to_T(_source, source);
		point_to_T(_target, target);

		T deformed_point[3];
		deform_point_embedded_deformation(source,
					                      _global_pos, global_deformation,
					                      _n1_pos, n1_deformation, _n1_weight,
					                      _n2_pos, n2_deformation, _n2_weight,
					                      _n3_pos, n3_deformation, _n3_weight,
					                      _n4_pos, n4_deformation, _n4_weight,
					                      deformed_point);

		// The error is the difference between the deformed source position and the target position multiplied with the weight
		substract(deformed_point, target, residuals);
		//scalar_multiply(residuals, w[0], residuals);
		return true;
	}

	//template <typename T>
	//bool operator()(const T* const global_rotation, const T* const global_translation, const T* const translation, const T* const w, T* residuals) const
	//{
	//	T node_g[3];
	//	T global_g[3];
	//	T target_point[3];
	//	point_to_T(_node_g, node_g);
	//	point_to_T(_global_g, global_g);
	//	point_to_T(_target_point, target_point);

	//	T deformed_node[3];
	//	// local deformation of node position
	//	addition(node_g, translation, deformed_node);

	//	// global deformation of node position
	//	substract(deformed_node, global_g, deformed_node);
	//	matrix_multiplication(global_rotation, deformed_node, deformed_node);
	//	addition(deformed_node, global_g, deformed_node);
	//	addition(deformed_node, global_translation, deformed_node);

	//	// The error is the difference between the predicted and observed position multiplied with the weight
	//	substract(deformed_node, target_point, residuals);
	//	scalar_multiply(residuals, w[0], residuals);

	//	return true;
	//}
};




struct FitStarPointToPlaneCostFunction {
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

	FitStarPointToPlaneCostFunction(const Point& target_pos, const Vector & target_normal,
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
		return (new ceres::AutoDiffCostFunction<FitStarPointToPlaneCostFunction, 1, 12, 12, 12, 12, 12>(
			new FitStarPointToPlaneCostFunction(target_pos, target_normal, source_pos, global_pos, n1_pos, n2_pos, n3_pos, n4_pos, n1_weight, n2_weight, n3_weight, n4_weight)));
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
		T target_pos[3];
		T target_normal[3];

		point_to_T(_source_pos, source_pos);
		point_to_T(_target_pos, target_pos);
		point_to_T(_target_normal, target_normal);

		T deformed_point[3];
		deform_point_embedded_deformation(source_pos,
					                      _global_pos, global_deformation,
					                      _n1_pos, n1_deformation, _n1_weight,
					                      _n2_pos, n2_deformation, _n2_weight,
					                      _n3_pos, n3_deformation, _n3_weight,
					                      _n4_pos, n4_deformation, _n4_weight,
					                      deformed_point);

		// Point to plane error = dot((deformed position - target_point), target_normal 
		T difference[3];
		substract(deformed_point, target_pos, difference);
		residuals[0] = dot(difference, target_normal);// *w[0];

		return true;
	}

	//template <typename T>
	//bool operator()(const T* const global_rotation, const T* const global_translation, const T* const rotation, const T* const translation, const T* const w, T* residuals) const
	//{
	//	T node_g[3];
	//	T global_g[3];
	//	T target_point[3];
	//	T target_normal[3];
	//	point_to_T(_node_g, node_g);
	//	point_to_T(_global_g, global_g);
	//	point_to_T(_target_point, target_point);
	//	point_to_T(_target_normal, target_normal);

	//	T deformed_node[3];
	//	// local deformation (only translation) of node position
	//	addition(node_g, translation, deformed_node);

	//	// global deformation (rotation and translation) of node position
	//	// rotation
	//	substract(deformed_node, global_g, deformed_node);
	//	matrix_multiplication(global_rotation, deformed_node, deformed_node);
	//	addition(deformed_node, global_g, deformed_node);
	//	// translation
	//	addition(deformed_node, global_translation, deformed_node);

	//	//// deform the node normal
	//	//T rotation_t[9];
	//	//T global_rotation_t[9];
	//	//matrix_transpose(rotation, rotation_t);
	//	//matrix_transpose(global_rotation, global_rotation_t);
	//	//matrix_multiplication(rotation_t, normal, normal);
	//	//matrix_multiplication(global_rotation_t, normal, normal);
	//	//normalize(normal, normal);

	//	// The error is the difference between the predicted and observed position.
	//	T difference[3];
	//	substract(deformed_node, target_point, difference);
	//	//substract(point, node_g, difference);
	//	residuals[0] = dot(difference, target_normal) *w[0];

	//	return true;
	//}
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