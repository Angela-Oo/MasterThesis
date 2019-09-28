#pragma once

#include "algo/registration/util/ceres_math.h"
#include "mesh/mesh_definition.h"
#include "arap_point_to_point_cost_function.h"
#include "util/ceres_include.h"
#include <ceres/rotation.h>

namespace Registration
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
		rotation_vector_from_deformation(deformation_i, rotation_i);
		T translation_i[3];
		translation_vector_from_deformation(deformation_i, translation_i);
		T translation_j[3];
		translation_vector_from_deformation(deformation_j, translation_j);

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



struct AdaptableRigidityWeightCostFunction
{
	double _rigidity_weight;
	AdaptableRigidityWeightCostFunction(double rigidity_weight)
		: _rigidity_weight(rigidity_weight)
	{}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(double rigidity_weight)
	{
		return (new ceres::AutoDiffCostFunction<AdaptableRigidityWeightCostFunction, 1, 1>(new AdaptableRigidityWeightCostFunction(rigidity_weight)));
	}

	template <typename T>
	bool operator()(const T* const weight, T* residuals) const
	{
		T expected{ _rigidity_weight };
		if (weight[0] > expected)
			residuals[0] = T{ 0 };
		else {
			residuals[0] = ceres::pow((T{ 1. } - (weight[0] / expected)), T{ 3 });
		}
		return true;
	}
};


struct RigidityWeightRegularizationCostFunction
{
	double _rigidity_weight;
	RigidityWeightRegularizationCostFunction(double rigidity_weight)
		: _rigidity_weight(rigidity_weight)
	{}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(double rigidity_weight)
	{
		return (new ceres::AutoDiffCostFunction<RigidityWeightRegularizationCostFunction, 1, 1>(new RigidityWeightRegularizationCostFunction(rigidity_weight)));
	}

	template <typename T>
	bool operator()(const T* const weight, T* residuals) const
	{
		T expected{ _rigidity_weight };
		if (weight[0] > expected)
			residuals[0] = T{ 0 };
		else
			residuals[0] = expected - weight[0];
		return true;
	}
};




struct AsRigidAsPossibleAdaptableRigidityCostFunction {
	const Point _v_i; // vi
	const Point _v_j; // vj
	double _minimal_rigidity_weight;

	AsRigidAsPossibleAdaptableRigidityCostFunction(const Point & v_i, const Point & v_j, double minimal_rigidity_weight)
		: _v_i(v_i)
		, _v_j(v_j)
		, _minimal_rigidity_weight(minimal_rigidity_weight)
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point & v_i, const Point & v_j, double minimal_rigidity_weight) {
		return (new ceres::AutoDiffCostFunction<AsRigidAsPossibleAdaptableRigidityCostFunction, 3, 6, 6, 1>(new AsRigidAsPossibleAdaptableRigidityCostFunction(v_i, v_j, minimal_rigidity_weight)));
	}

	// E_arap = sum_{i} sum_{j in N} |(vi-vj) - Ri(vi' - vj')|^2
	// E_arap = sum_{i} sum_{j in N} |Ri(vj-vi) - ((vj + tj) - (vi + ti))|^2
	template <typename T>
	bool operator()(const T* const deformation_i, const T* const deformation_j, const T* const w, T* residuals) const
	{
		T vi[3];
		T vj[3];
		point_to_T(_v_i, vi);
		point_to_T(_v_j, vj);

		T rotation_i[3];
		rotation_vector_from_deformation(deformation_i, rotation_i);
		T translation_i[3];
		translation_vector_from_deformation(deformation_i, translation_i);
		T translation_j[3];
		translation_vector_from_deformation(deformation_j, translation_j);

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

		substract(rotated_edge, transformed_edge, residuals);

		T minimal_weight{ _minimal_rigidity_weight };
		T weight = w[0];
		if (weight < minimal_weight)
			weight = minimal_weight;
		scalar_multiply(residuals, weight, residuals);
		return true;
	}
};






struct ARAPAdaptiveRigidityVertexCostFunction
{
	const Point _v_i; // vi
	const Point _v_j; // vj
	double _minimal_rigidity_weight;

	ARAPAdaptiveRigidityVertexCostFunction(const Point & v_i, const Point & v_j, double minimal_rigidity_weight)
		: _v_i(v_i)
		, _v_j(v_j)
		, _minimal_rigidity_weight(minimal_rigidity_weight)
	{}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point & v_i, const Point & v_j, double minimal_rigidity_weight)
	{
		return (new ceres::AutoDiffCostFunction<ARAPAdaptiveRigidityVertexCostFunction, 3, 6, 6, 1, 1>(new ARAPAdaptiveRigidityVertexCostFunction(v_i, v_j, minimal_rigidity_weight)));
	}

	// E_arap = sum_{i} sum_{j in N} | w * [Ri(vj-vi) - ((vj + tj) - (vi + ti))] |^2
	template <typename T>
	bool operator()(const T* const deformation_i, const T* const deformation_j, const T* const w_i, const T* const w_j, T* residuals) const
	{
		T vi[3];
		T vj[3];
		point_to_T(_v_i, vi);
		point_to_T(_v_j, vj);

		T rotation_i[3];
		rotation_vector_from_deformation(deformation_i, rotation_i);
		T translation_i[3];
		translation_vector_from_deformation(deformation_i, translation_i);
		T translation_j[3];
		translation_vector_from_deformation(deformation_j, translation_j);

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
		substract(rotated_edge, transformed_edge, residuals);

		T min{ _minimal_rigidity_weight };
		T wi = w_i[0];
		if (wi < min)
			wi = min;
		T wj = w_j[0];
		if (wj < min)
			wj = min;
		
		T weight = (wi + wj) * T { 0.5 };
		scalar_multiply(residuals, weight, residuals);
		
		return true;
	}
};


}