#pragma once

#include "mLibInclude.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <functional>
#include "algo/registration/util/ceres_math.h"
#include "algo/surface_mesh/mesh_definition.h"

namespace Registration {

// deformation of vi at node j = (Rj(vi-gj) + gj + tj)
template<typename T>
void deform_point(const Point & point_, const Point & global_center_, const T * const rotation, const T * const translation, T* result)
{
	T point[3];
	T global_center[3];
	T rotated_point[3];
	T moved_point[3];
	point_to_T(point_, point);
	point_to_T(global_center_, global_center);

	// edge rotation = Rj(vi - gj)
	substract(point, global_center, rotated_point);
	ceres::AngleAxisRotatePoint(rotation, rotated_point, rotated_point);

	// translation of node j = gj + tj
	addition(global_center, translation, moved_point);

	// deformation of vi at node j = Rj(vi-gj) + gj + tj
	addition(moved_point, rotated_point, result);
}

struct FitPointToPointAngleAxisCostFunction {
	const Point _target;
	const Point _source;
	const Point _global_center;
	FitPointToPointAngleAxisCostFunction(const Point & source, const Point &target, const Point &global_center)
		: _source(source), _target(target), _global_center(global_center)
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point & source, const Point &target, const Point &global_center) {
		return (new ceres::AutoDiffCostFunction<FitPointToPointAngleAxisCostFunction, 3, 3, 3>(new FitPointToPointAngleAxisCostFunction(source, target, global_center)));
	}

	template <typename T>
	bool operator()(const T* const rotation, const T* const translation, T* residuals) const {
		T target[3];
		point_to_T(_target, target);

		T deformed_point[3];
		deform_point(_source, _global_center, rotation, translation, deformed_point);

		// The error is the difference between the predicted and observed position.
		substract(deformed_point, target, residuals);
		return true;
	}
};



struct FitPointToPlaneAngleAxisCostFunction {
	const Point _target;
	const Point _source;
	const Vector _target_normal;
	const Point _global_center;

	FitPointToPlaneAngleAxisCostFunction(const Point& source, const Point& target, const Vector& target_normal, const Point &global_center)
		: _source(source), _target(target), _target_normal(target_normal), _global_center(global_center)
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point& source, const Point& target, const Vector& target_normal, const Point &global_center)
	{
		return (new ceres::AutoDiffCostFunction<FitPointToPlaneAngleAxisCostFunction, 1, 3, 3>(new FitPointToPlaneAngleAxisCostFunction(source, target, target_normal, global_center)));
	}

	template <typename T>
	bool operator()(const T* const rotation, const T* const translation, T* residuals) const
	{
		T target[3];
		T target_normal[3];
		point_to_T(_target, target);
		point_to_T(_target_normal, target_normal);

		T deformed_point[3];
		deform_point(_source, _global_center, rotation, translation, deformed_point);

		// Point to plane error = dot((deformed position - target_point), target_normal 
		T difference[3];
		substract(deformed_point, target, difference);
		residuals[0] = dot(difference, target_normal);

		return true;
	}
};

}