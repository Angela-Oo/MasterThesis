#pragma once

#include "algo/registration/util/ceres_math.h"
#include <vector>
#include "util/ceres_include.h"
#include <ceres/rotation.h>

namespace Registration {
namespace ED
{
//
//// deformation of vi at node j = (Rj(vi-gj) + gj + tj)
//template<typename T>
//void deform_point_at_node_embedded_deformation(const T * const point, const T * const node_pos, const T * const deformation, T* result)
//{
//	T rotation_matrix[9];
//	rotation_matrix[0] = deformation[0];
//	rotation_matrix[1] = deformation[1];
//	rotation_matrix[2] = deformation[2];
//	rotation_matrix[3] = deformation[3];
//	rotation_matrix[4] = deformation[4];
//	rotation_matrix[5] = deformation[5];
//	rotation_matrix[6] = deformation[6];
//	rotation_matrix[7] = deformation[7];
//	rotation_matrix[8] = deformation[8];
//
//	T translation[3];
//	translation[0] = deformation[9];
//	translation[1] = deformation[10];
//	translation[2] = deformation[11];
//
//	T rotated_point[3];
//	T moved_point[3];
//
//	// edge rotation = Rj(vi - gj)
//	substract(point, node_pos, rotated_point);
//	matrix_multiplication(rotation_matrix, rotated_point, rotated_point);
//
//	// translation of node j = gj + tj
//	addition(node_pos, translation, moved_point);
//
//	// deformation of vi at node j = Rj(vi-gj) + gj + tj
//	addition(moved_point, rotated_point, result);
//}
//
//// deformation of vi at node j = (Rj(vi-gj) + gj + tj)
//template<typename T>
//void deform_point_at_node_embedded_deformation(const T * const point, const Point & node_pos, const T * const deformation, T* result)
//{
//	T node[3];
//	point_to_T(node_pos, node);
//	deform_point_at_node_embedded_deformation(point, node, deformation, result);
//}
//
//
//// deformation of vi at node j = wj(vi) * (Rj(vi-gj) + gj + tj)
//template<typename T>
//void deform_point_at_node_embedded_deformation(const T * const point, const Point & node_pos, const T * const deformation, double weight, T* result)
//{
//	T node[3];
//	point_to_T(node_pos, node);
//	T deformed_point[3];
//	deform_point_at_node_embedded_deformation(point, node, deformation, deformed_point);
//	// weighted deformation
//	scalar_multiply(deformed_point, T(weight), result);
//}
//
//
//template<typename T>
//void deform_point_embedded_deformation(const T* const point,
//									   const Point & global_pos,
//									   const T* const global_deformation,
//									   const Point & n1_pos,
//									   const T* const n1_deformation,
//									   double n1_weight,
//									   const Point & n2_pos,
//									   const T* const n2_deformation,
//									   double n2_weight,
//									   const Point & n3_pos,
//									   const T* const n3_deformation,
//									   double n3_weight,
//									   const Point & n4_pos,
//									   const T* const n4_deformation,
//									   double n4_weight,
//									   T* result)
//{
//	T deformed_point[3];
//	T weighted_deformed_point[3];
//
//	// local deformation of node position
//	deform_point_at_node_embedded_deformation(point, n1_pos, n1_deformation, n1_weight, deformed_point);
//
//	deform_point_at_node_embedded_deformation(point, n2_pos, n2_deformation, n2_weight, weighted_deformed_point);
//	addition(deformed_point, weighted_deformed_point, deformed_point);
//
//	deform_point_at_node_embedded_deformation(point, n3_pos, n3_deformation, n3_weight, weighted_deformed_point);
//	addition(deformed_point, weighted_deformed_point, deformed_point);
//
//	deform_point_at_node_embedded_deformation(point, n4_pos, n4_deformation, n4_weight, weighted_deformed_point);
//	addition(deformed_point, weighted_deformed_point, deformed_point);
//
//	// global deformation of position
//	deform_point_at_node_embedded_deformation(deformed_point, global_pos, global_deformation, result);
//}

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