#pragma once

#include "../../mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "../ceres_math.h"

//
//struct RotationCostFunction {
//	RotationCostFunction()
//	{ }
//
//	// Factory to hide the construction of the CostFunction object from the client code.
//	static ceres::CostFunction* Create() {
//		return (new ceres::AutoDiffCostFunction<RotationCostFunction, 1, 9>(new RotationCostFunction()));
//	}
//
//	// E_fit = sum_{i} (c1*c2)^2+(c1*c3)^2+(c2*c3)^2+(c1*c1-1)^2+(c2*c2-1)^2+(c3*c3-1)^2
//	template <typename T>
//	bool operator()(const T* const rotation_matrix, T* residuals) const
//	{
//		const T *c1 = rotation_matrix;
//		const T *c2 = rotation_matrix + 3;
//		const T *c3 = rotation_matrix + 6;
//		T one = T(1.);
//		T dot_c1_c2 = dot(c1, c2);
//
//		residuals[0] = pow2(dot(c1, c2)) + pow2(dot(c1, c3)) + pow2(dot(c1, c3))
//			+ pow2(dot(c1, c1) - one) + pow2(dot(c2, c2) - one) + pow2(dot(c3, c3) - one);
//		return true;
//	}
//};
//
//
//
//struct SmoothCostFunction {
//	const ml::vec3f& _vi;
//	const ml::vec3f& _vj;
//
//	SmoothCostFunction(const ml::vec3f & vi, const ml::vec3f & vj)
//		: _vi(vi)
//		, _vj(vj)
//	{ }
//
//	// Factory to hide the construction of the CostFunction object from the client code.
//	static ceres::CostFunction* Create(const ml::vec3f & vi, const ml::vec3f & vj) {
//		return (new ceres::AutoDiffCostFunction<SmoothCostFunction, 3, 9, 3, 3>(new SmoothCostFunction(vi, vj)));
//	}
//
//
//	template <typename T>
//	bool operator()(const T* const rotation_matrix, const T* const bi, const T* const bj, T* residuals) const
//	{
//		T vi[3];
//		T vj[3];
//		vec3f_to_T(_vi, vi);
//		vec3f_to_T(_vj, vj);
//		T edge[3];
//		T result[3];
//
//		substract(vj, vi, edge);
//		matrix_multiplication(rotation_matrix, edge, result);
//		addition(result, vi, result);
//		addition(result, bi, result);
//		substract(result, vj, result);
//		substract(result, bj, result);
//
//		residuals[0] = result[0];
//		residuals[1] = result[1];
//		residuals[2] = result[2];
//		return true;
//	}
//};


struct FitStarPointToPointCostFunction {
	const ml::vec3f& _point;
	const ml::vec3f& _node_g;
	const ml::vec3f& _global_g;

	FitStarPointToPointCostFunction(const ml::vec3f &point, const ml::vec3f &node_g, const ml::vec3f &global_g) :
		_point(point), _node_g(node_g), _global_g(global_g)
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f &point, const ml::vec3f &node_g, const ml::vec3f &global_g) {
		return (new ceres::AutoDiffCostFunction<FitStarPointToPointCostFunction, 3, 9, 3, 3, 1>(new FitStarPointToPointCostFunction(point, node_g, global_g)));
	}

	template <typename T>
	bool operator()(const T* const global_rotation, const T* const global_translation, const T* const translation, const T* const w, T* residuals) const
	{
		T n[3];
		T g[3];
		T point[3];
		vec3f_to_T(_node_g, n);
		vec3f_to_T(_global_g, g);
		vec3f_to_T(_point, point);

		// local deformation of node position
		addition(n, translation, n);

		// global deformation of node position
		substract(n, g, n);
		matrix_multiplication(global_rotation, n, n);
		addition(n, g, n);
		addition(n, global_translation, n);

		// The error is the difference between the predicted and observed position multiplied with the weight
		substract(n, point, residuals);
		scalar_multiply(residuals, w[0], residuals);

		return true;
	}
};



struct FitStarPointToPointAngleAxisCostFunction {
	const ml::vec3f& _point;
	const ml::vec3f& _node_g;
	const ml::vec3f& _global_g;

	FitStarPointToPointAngleAxisCostFunction(const ml::vec3f &point, const ml::vec3f &node_g, const ml::vec3f &global_g) :
		_point(point), _node_g(node_g), _global_g(global_g)
	{ }

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f &point, const ml::vec3f &node_g, const ml::vec3f &global_g) {
		return (new ceres::AutoDiffCostFunction<FitStarPointToPointAngleAxisCostFunction, 3, 9, 3, 3, 1>(new FitStarPointToPointAngleAxisCostFunction(point, node_g, global_g)));
	}

	template <typename T>
	bool operator()(const T* const global_rotation, const T* const global_translation, const T* const translation, const T* const w, T* residuals) const
	{
		T n[3];
		T g[3];
		T point[3];
		vec3f_to_T(_node_g, n);
		vec3f_to_T(_global_g, g);
		vec3f_to_T(_point, point);

		// local deformation of node position
		addition(n, translation, n);

		// global deformation of node position
		substract(n, g, n);

		T global_rotation_angle_axis[3];//
		ceres::RotationMatrixToAngleAxis(global_rotation, global_rotation_angle_axis);//
		ceres::AngleAxisRotatePoint(global_rotation_angle_axis, n, n);//
		//ceres::AngleAxisRotatePoint(global_rotation, n, n);
		addition(n, g, n);
		addition(n, global_translation, n);

		// The error is the difference between the predicted and observed position multiplied with the weight
		substract(n, point, residuals);
		scalar_multiply(residuals, w[0], residuals);

		return true;
	}
};


struct FitStarPointToPlaneCostFunction {
	const ml::vec3f& _point;
	const ml::vec3f& _node_g;
	const ml::vec3f& _node_normal;
	const ml::vec3f& _global_g;

	FitStarPointToPlaneCostFunction(const ml::vec3f& point, const ml::vec3f& node_g, const ml::vec3f& node_normal, const ml::vec3f &global_g) :
		_point(point), _node_g(node_g), _node_normal(node_normal), _global_g(global_g)
	{
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f& point, const ml::vec3f& node_g, const ml::vec3f& node_normal, const ml::vec3f &global_g)
	{
		return (new ceres::AutoDiffCostFunction<FitStarPointToPlaneCostFunction, 1, 9, 3, 9, 3, 1>(new FitStarPointToPlaneCostFunction(point, node_g, node_normal, global_g)));
	}

	template <typename T>
	bool operator()(const T* const global_rotation, const T* const global_translation, const T* const rotation, const T* const translation, const T* const w, T* residuals) const
	{
		T node_g[3];
		T global_g[3];
		T point[3];
		T normal[3];
		vec3f_to_T(_node_g, node_g);
		vec3f_to_T(_global_g, global_g);
		vec3f_to_T(_point, point);
		vec3f_to_T(_node_normal, normal);

		// local deformation of node position
		addition(node_g, translation, node_g);

		// global deformation of node position
		substract(node_g, global_g, node_g);
		matrix_multiplication(global_rotation, node_g, node_g);
		addition(node_g, global_g, node_g);
		addition(node_g, global_translation, node_g);

		// deform the node normal
		T rotation_t[9];
		T global_rotation_t[9];
		matrix_transpose(rotation, rotation_t);
		matrix_transpose(global_rotation, global_rotation_t);
		matrix_multiplication(rotation_t, normal, normal);
		matrix_multiplication(global_rotation_t, normal, normal);
		normalize(normal, normal);
		//matrix_transpose()

		// The error is the difference between the predicted and observed position.
		T difference[3];
		substract(point, node_g, difference);
		residuals[0] = dot(difference, normal) * w[0];

		return true;
	}
};



struct FitStarPointToPlaneAngleAxisCostFunction {
	const ml::vec3f& _point;
	const ml::vec3f& _node_g;
	const ml::vec3f& _node_normal;
	const ml::vec3f& _global_g;

	FitStarPointToPlaneAngleAxisCostFunction(const ml::vec3f& point, const ml::vec3f& node_g, const ml::vec3f& node_normal, const ml::vec3f &global_g) :
		_point(point), _node_g(node_g), _node_normal(node_normal), _global_g(global_g)
	{
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f& point, const ml::vec3f& node_g, const ml::vec3f& node_normal, const ml::vec3f &global_g)
	{
		return (new ceres::AutoDiffCostFunction<FitStarPointToPlaneAngleAxisCostFunction, 1, 9, 3, 9, 3, 1>(new FitStarPointToPlaneAngleAxisCostFunction(point, node_g, node_normal, global_g)));
	}

	template <typename T>
	bool operator()(const T* const global_rotation, const T* const global_translation, const T* const rotation, const T* const translation, const T* const w, T* residuals) const
	{
		T node_g[3];
		T global_g[3];
		T point[3];
		T normal[3];
		vec3f_to_T(_node_g, node_g);
		vec3f_to_T(_global_g, global_g);
		vec3f_to_T(_point, point);
		vec3f_to_T(_node_normal, normal);

		// local deformation of node position
		addition(node_g, translation, node_g);

		// global deformation of node position
		substract(node_g, global_g, node_g);

		T global_rotation_angle_axis[3];//
		ceres::RotationMatrixToAngleAxis(global_rotation, global_rotation_angle_axis);//
		ceres::AngleAxisRotatePoint(global_rotation_angle_axis, node_g, node_g);//
		//ceres::AngleAxisRotatePoint(global_rotation, node_g, node_g);
		addition(node_g, global_g, node_g);
		addition(node_g, global_translation, node_g);

		// deform the node normal
		T rotation_t[9];
		T global_rotation_t[9];
		T rotation_angle_axis[3];//
		ceres::RotationMatrixToAngleAxis(rotation, rotation_angle_axis);//
		ceres::AngleAxisToRotationMatrix(rotation_angle_axis, rotation_t);//
		ceres::AngleAxisToRotationMatrix(global_rotation_angle_axis, global_rotation_t);//
		//ceres::AngleAxisToRotationMatrix(rotation, rotation_t);
		//ceres::AngleAxisToRotationMatrix(global_rotation, global_rotation_t);
		matrix_transpose(rotation_t, rotation_t);
		matrix_transpose(global_rotation_t, global_rotation_t);
		matrix_multiplication(rotation_t, normal, normal);
		matrix_multiplication(global_rotation_t, normal, normal);
		normalize(normal, normal);
		//matrix_transpose()

		// The error is the difference between the predicted and observed position.
		T difference[3];
		substract(point, node_g, difference);
		residuals[0] = dot(difference, normal) * w[0];

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
		residuals[0] = one - weight[0] * weight[0];
		return true;
	}
};