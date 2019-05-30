#pragma once

#include "mLibInclude.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <functional>
#include "algo/ceres_math.h"
#include "algo/surface_mesh/mesh_definition.h"


struct FitPointToPointAngleAxisCostFunction {
	const Point _target;
	const Point _source;
	FitPointToPointAngleAxisCostFunction(const Point & source, const Point &target)
		: _source(source), _target(target)
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point & source, const Point &target) {
		return (new ceres::AutoDiffCostFunction<FitPointToPointAngleAxisCostFunction, 3, 3, 3>(new FitPointToPointAngleAxisCostFunction(source, target)));
	}

	template <typename T>
	bool operator()(const T* const rotation, const T* const translation, T* residuals) const {
		T source[3];
		T target[3];		
		point_to_T(_source, source);
		point_to_T(_target, target);

		T deformed_point[3];
		ceres::AngleAxisRotatePoint(rotation, source, deformed_point);
		addition(deformed_point, translation, deformed_point);

		// The error is the difference between the predicted and observed position.
		substract(deformed_point, target, residuals);
		return true;
	}
};



struct FitPointToPlaneAngleAxisCostFunction {
	const Point _target;
	const Point _source;
	const Vector _target_normal;

	FitPointToPlaneAngleAxisCostFunction(const Point& source, const Point& target, const Vector& target_normal)
		: _source(source), _target(target), _target_normal(target_normal)
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point& source, const Point& target, const Vector& target_normal)
	{
		return (new ceres::AutoDiffCostFunction<FitPointToPlaneAngleAxisCostFunction, 1, 3, 3>(new FitPointToPlaneAngleAxisCostFunction(source, target, target_normal)));
	}

	template <typename T>
	bool operator()(const T* const rotation, const T* const translation, T* residuals) const
	{
		T source[3];
		T target[3];
		T target_normal[3];
		point_to_T(_source, source);
		point_to_T(_target, target);
		point_to_T(_target_normal, target_normal);

		T deformed_point[3];
		ceres::AngleAxisRotatePoint(rotation, source, deformed_point);
		addition(deformed_point, translation, deformed_point);

		// The error is the difference between the predicted and observed position.
		T difference[3];
		//substract(target, deformed_point, difference);
		substract(deformed_point, target, difference);
		residuals[0] = dot(difference, target_normal);

		return true;
	}
};
