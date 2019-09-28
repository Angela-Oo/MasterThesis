#pragma once

#include "algo/registration/util/ceres_math.h"
#include "mesh/mesh_definition.h"
#include "util/ceres_include.h"
#include "arap_point_to_point_cost_function.h"

namespace Registration {



struct FitPointToPlaneAngleAxisCostFunction
{
	const Point _target_pos;
	const Vector _target_normal;
	const Point _source_pos;
	const Point _global_pos;
	const Point _n1_pos;
	const Point _n2_pos;
	const Point _n3_pos;
	Point _n4_pos;
	Point _n5_pos;
	Point _n6_pos;
	const double _n1_weight;
	const double _n2_weight;
	const double _n3_weight;
	double _n4_weight;
	double _n5_weight;
	double _n6_weight;


	FitPointToPlaneAngleAxisCostFunction(const Point& target_pos, const Vector & target_normal,
											 const Point& source_pos, const Point &global_pos,
											 const Point n1_pos, const Point n2_pos, const Point n3_pos, const Point n4_pos, const Point n5_pos, const Point n6_pos,
											 double n1_weight, double n2_weight, double n3_weight, double n4_weight, double n5_weight, double n6_weight)
		: _target_pos(target_pos), _target_normal(target_normal)
		, _source_pos(source_pos), _global_pos(global_pos)
		, _n1_pos(n1_pos), _n2_pos(n2_pos), _n3_pos(n3_pos), _n4_pos(n4_pos), _n5_pos(n5_pos), _n6_pos(n6_pos)
		, _n1_weight(n1_weight), _n2_weight(n2_weight), _n3_weight(n3_weight), _n4_weight(n4_weight), _n5_weight(n5_weight), _n6_weight(n6_weight)
	{}

	FitPointToPlaneAngleAxisCostFunction(const Point& target_pos, const Vector & target_normal,
											 const Point& source_pos, const Point &global_pos,
											 const Point n1_pos, const Point n2_pos, const Point n3_pos, const Point n4_pos, const Point n5_pos,
											 double n1_weight, double n2_weight, double n3_weight, double n4_weight, double n5_weight)
		: _target_pos(target_pos), _target_normal(target_normal)
		, _source_pos(source_pos), _global_pos(global_pos)
		, _n1_pos(n1_pos), _n2_pos(n2_pos), _n3_pos(n3_pos), _n4_pos(n4_pos), _n5_pos(n5_pos)
		, _n1_weight(n1_weight), _n2_weight(n2_weight), _n3_weight(n3_weight), _n4_weight(n4_weight), _n5_weight(n5_weight)
	{}

	FitPointToPlaneAngleAxisCostFunction(const Point& target_pos, const Vector & target_normal,
											 const Point& source_pos, const Point &global_pos,
											 const Point n1_pos, const Point n2_pos, const Point n3_pos, const Point n4_pos,
											 double n1_weight, double n2_weight, double n3_weight, double n4_weight)
		: _target_pos(target_pos), _target_normal(target_normal)
		, _source_pos(source_pos), _global_pos(global_pos)
		, _n1_pos(n1_pos), _n2_pos(n2_pos), _n3_pos(n3_pos), _n4_pos(n4_pos)
		, _n1_weight(n1_weight), _n2_weight(n2_weight), _n3_weight(n3_weight), _n4_weight(n4_weight)
	{}

	FitPointToPlaneAngleAxisCostFunction(const Point& target_pos, const Vector & target_normal,
											 const Point& source_pos, const Point &global_pos,
											 const Point n1_pos, const Point n2_pos, const Point n3_pos,
											 double n1_weight, double n2_weight, double n3_weight)
		: _target_pos(target_pos), _target_normal(target_normal)
		, _source_pos(source_pos), _global_pos(global_pos)
		, _n1_pos(n1_pos), _n2_pos(n2_pos), _n3_pos(n3_pos)
		, _n1_weight(n1_weight), _n2_weight(n2_weight), _n3_weight(n3_weight)
	{}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point& target_pos, const Vector & target_normal,
									   const Point& source_pos, const Point &global_pos,
									   const Point n1_pos, const Point n2_pos, const Point n3_pos, const Point n4_pos, const Point n5_pos, const Point n6_pos,
									   double n1_weight, double n2_weight, double n3_weight, double n4_weight, double n5_weight, double n6_weight)
	{
		return (new ceres::AutoDiffCostFunction<FitPointToPlaneAngleAxisCostFunction, 1, 6, 6, 6, 6, 6, 6, 6>(
			new FitPointToPlaneAngleAxisCostFunction(target_pos, target_normal, source_pos, global_pos, n1_pos, n2_pos, n3_pos, n4_pos, n5_pos, n6_pos, n1_weight, n2_weight, n3_weight, n4_weight, n5_weight, n6_weight)));
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point& target_pos, const Vector & target_normal,
									   const Point& source_pos, const Point &global_pos,
									   const Point n1_pos, const Point n2_pos, const Point n3_pos, const Point n4_pos, const Point n5_pos,
									   double n1_weight, double n2_weight, double n3_weight, double n4_weight, double n5_weight)
	{
		return (new ceres::AutoDiffCostFunction<FitPointToPlaneAngleAxisCostFunction, 1, 6, 6, 6, 6, 6, 6>(
			new FitPointToPlaneAngleAxisCostFunction(target_pos, target_normal, source_pos, global_pos, n1_pos, n2_pos, n3_pos, n4_pos, n5_pos, n1_weight, n2_weight, n3_weight, n4_weight, n5_weight)));
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point& target_pos, const Vector & target_normal,
									   const Point& source_pos, const Point &global_pos,
									   const Point n1_pos, const Point n2_pos, const Point n3_pos, const Point n4_pos,
									   double n1_weight, double n2_weight, double n3_weight, double n4_weight)
	{
		return (new ceres::AutoDiffCostFunction<FitPointToPlaneAngleAxisCostFunction, 1, 6, 6, 6, 6, 6>(
			new FitPointToPlaneAngleAxisCostFunction(target_pos, target_normal, source_pos, global_pos, n1_pos, n2_pos, n3_pos, n4_pos, n1_weight, n2_weight, n3_weight, n4_weight)));
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const Point& target_pos, const Vector & target_normal,
									   const Point& source_pos, const Point &global_pos,
									   const Point n1_pos, const Point n2_pos, const Point n3_pos,
									   double n1_weight, double n2_weight, double n3_weight)
	{
		return (new ceres::AutoDiffCostFunction<FitPointToPlaneAngleAxisCostFunction, 1, 6, 6, 6, 6>(
			new FitPointToPlaneAngleAxisCostFunction(target_pos, target_normal, source_pos, global_pos, n1_pos, n2_pos, n3_pos, n1_weight, n2_weight, n3_weight)));
	}



	template <typename T>
	bool operator()(const T* const global_deformation,
					const T* const n1_deformation,
					const T* const n2_deformation,
					const T* const n3_deformation,
					const T* const n4_deformation,
					const T* const n5_deformation,
					const T* const n6_deformation,
					T* residuals) const
	{
		T source_pos[3];
		T target_pos[3];
		T target_normal[3];

		point_to_T(_source_pos, source_pos);
		point_to_T(_target_pos, target_pos);
		point_to_T(_target_normal, target_normal);

		T deformed_point[3];
		defom_point(source_pos,
					_global_pos, global_deformation,
					_n1_pos, n1_deformation, _n1_weight,
					_n2_pos, n2_deformation, _n2_weight,
					_n3_pos, n3_deformation, _n3_weight,
					_n4_pos, n4_deformation, _n4_weight,
					_n5_pos, n5_deformation, _n5_weight,
					_n6_pos, n6_deformation, _n6_weight,
					deformed_point);

		// Point to plane error = dot((deformed position - target_point), target_normal 
		T difference[3];
		substract(deformed_point, target_pos, difference);
		residuals[0] = dot(difference, target_normal);// *w[0];

		return true;
	}

	template <typename T>
	bool operator()(const T* const global_deformation,
					const T* const n1_deformation,
					const T* const n2_deformation,
					const T* const n3_deformation,
					const T* const n4_deformation,
					const T* const n5_deformation,
					T* residuals) const
	{
		T source_pos[3];
		T target_pos[3];
		T target_normal[3];

		point_to_T(_source_pos, source_pos);
		point_to_T(_target_pos, target_pos);
		point_to_T(_target_normal, target_normal);

		T deformed_point[3];
		defom_point(source_pos,
					_global_pos, global_deformation,
					_n1_pos, n1_deformation, _n1_weight,
					_n2_pos, n2_deformation, _n2_weight,
					_n3_pos, n3_deformation, _n3_weight,
					_n4_pos, n4_deformation, _n4_weight,
					_n5_pos, n5_deformation, _n5_weight,
					deformed_point);

		// Point to plane error = dot((deformed position - target_point), target_normal 
		T difference[3];
		substract(deformed_point, target_pos, difference);
		residuals[0] = dot(difference, target_normal);// *w[0];

		return true;
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
		defom_point(source_pos,
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


	template <typename T>
	bool operator()(const T* const global_deformation,
					const T* const n1_deformation,
					const T* const n2_deformation,
					const T* const n3_deformation,
					T* residuals) const
	{
		T source_pos[3];
		T target_pos[3];
		T target_normal[3];

		point_to_T(_source_pos, source_pos);
		point_to_T(_target_pos, target_pos);
		point_to_T(_target_normal, target_normal);

		T deformed_point[3];
		defom_point(source_pos,
					_global_pos, global_deformation,
					_n1_pos, n1_deformation, _n1_weight,
					_n2_pos, n2_deformation, _n2_weight,
					_n3_pos, n3_deformation, _n3_weight,
					deformed_point);

		// Point to plane error = dot((deformed position - target_point), target_normal 
		T difference[3];
		substract(deformed_point, target_pos, difference);
		residuals[0] = dot(difference, target_normal);// *w[0];

		return true;
	}
};


}