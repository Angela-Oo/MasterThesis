#pragma once


#include "../../mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <functional>
#include "../knn.h"


struct PointToPointErrorSE3 {
	const ml::vec3f& p_dst;
	const ml::vec3f& p_src;

	PointToPointErrorSE3(const ml::vec3f &dst, const ml::vec3f & src) :
		p_dst(dst), p_src(src)
	{
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f &observed, const ml::vec3f &worldPoint) {
		return (new ceres::AutoDiffCostFunction<PointToPointErrorSE3, 3, 6>(new PointToPointErrorSE3(observed, worldPoint)));
	}

	template <typename T>
	bool operator()(const T* const rotation_translation, T* residuals) const {

		T p[3] = { T(p_src[0]), T(p_src[1]), T(p_src[2]) };
		ceres::AngleAxisRotatePoint(rotation_translation, p, p);

		// camera[3,4,5] are the translation.
		p[0] += rotation_translation[3];
		p[1] += rotation_translation[4];
		p[2] += rotation_translation[5];

		// The error is the difference between the predicted and observed position.
		residuals[0] = p[0] - T(p_dst[0]);
		residuals[1] = p[1] - T(p_dst[1]);
		residuals[2] = p[2] - T(p_dst[2]);

		return true;
	}
};


struct PointToPointsErrorSE3LinearNNSearch {
	const ml::vec3f& _point;
	const std::vector<ml::vec3f>& _point_set;

	PointToPointsErrorSE3LinearNNSearch(const ml::vec3f &point, const std::vector<ml::vec3f> & point_set) :
		_point(point), _point_set(point_set)
	{
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f &observed, const std::vector<ml::vec3f> &worldPoint) {
		return (new ceres::AutoDiffCostFunction<PointToPointsErrorSE3LinearNNSearch, 3, 7>(new PointToPointsErrorSE3LinearNNSearch(observed, worldPoint)));
	}

	template <typename T>
	bool operator()(const T* const rotation_translation, T* residuals) const {
		T p[3] = { T(_point[0]), T(_point[1]), T(_point[2]) };
		ceres::AngleAxisRotatePoint(rotation_translation, p, p);

		// camera[3,4,5] are the translation.
		p[0] += rotation_translation[3];
		p[1] += rotation_translation[4];
		p[2] += rotation_translation[5];

		T distance;
		ml::vec3f p_dst;
		for (auto it = _point_set.begin(); it != _point_set.end(); it++) {
			T dst[3] = { T((*it)[0]), T((*it)[1]), T((*it)[2]) };
			T dir[3] = { dst[0] - p[0], dst[1] - p[1], dst[2] - p[2] };
			T dist = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
			if (it == _point_set.begin())
				distance = dist;
			if (dist < distance) {
				distance = dist;
				p_dst = *it;
			}
		}

		// The error is the difference between the predicted and observed position.
		residuals[0] = p[0] - T(p_dst[0]);
		residuals[1] = p[1] - T(p_dst[1]);
		residuals[2] = p[2] - T(p_dst[2]);

		return true;
	}
};


struct NearestPoint {
	std::function<ml::vec3f(const float *)> _nearest_point;
	NearestPoint(std::function<ml::vec3f(const float *)> nearest_point)
		: _nearest_point(nearest_point)
	{}

	bool operator()(const double* point,
					double* nn_point) const
	{
		ml::vec3f p = ml::vec3f(float(point[0]), point[1], point[2]);
		ml::vec3f nn_p = _nearest_point(p.array);
		nn_point[0] = double(nn_p[0]);
		nn_point[1] = double(nn_p[1]);
		nn_point[2] = double(nn_p[2]);
		return true;
	}
};

struct PointToPointsErrorSE3NNSearch {
private:
	const ml::vec3f& _point;
	ceres::CostFunctionToFunctor<3, 3> _nearest_point_cost;
public:
	PointToPointsErrorSE3NNSearch(const ml::vec3f &point, std::function<ml::vec3f(const float *)> nearest_point)
		: _point(point)
		, _nearest_point_cost(new ceres::NumericDiffCostFunction<NearestPoint, ceres::CENTRAL, 3, 3>(new NearestPoint(nearest_point)))
	{}

	static ceres::CostFunction* Create(const ml::vec3f &observed, std::function<ml::vec3f(const float *)> worldPoint) {
		return (new ceres::AutoDiffCostFunction<PointToPointsErrorSE3NNSearch, 3, 6>(new PointToPointsErrorSE3NNSearch(observed, worldPoint)));
	}

	template <typename T>
	bool operator()(const T* const rotation_translation, T* residuals) const {
		T p[3] = { T(_point[0]), T(_point[1]), T(_point[2]) };
		ceres::AngleAxisRotatePoint(rotation_translation, p, p);

		// camera[3,4,5] are the translation.
		p[0] += rotation_translation[3];
		p[1] += rotation_translation[4];
		p[2] += rotation_translation[5];

		T nn_point[3];
		_nearest_point_cost(p, nn_point);

		// The error is the difference between the predicted and observed position.
		residuals[0] = p[0] - T(nn_point[0]);
		residuals[1] = p[1] - T(nn_point[1]);
		residuals[2] = p[2] - T(nn_point[2]);

		return true;
	}
};