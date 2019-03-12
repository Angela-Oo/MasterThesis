#pragma once


#include "../mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>
#include "sophus_se3.h"

ml::mat4f pointToPointSE3(std::vector<ml::vec3f> &src, std::vector<ml::vec3f> &dst);

struct PointToPointErrorSE3 {
	const ml::vec3f& p_dst;
	const ml::vec3f& p_src;

	PointToPointErrorSE3(const ml::vec3f &dst, const ml::vec3f & src) :
		p_dst(dst), p_src(src)
	{
	}

	// Factory to hide the construction of the CostFunction object from the client code.
	static ceres::CostFunction* Create(const ml::vec3f &observed, const ml::vec3f &worldPoint) {
		return (new ceres::AutoDiffCostFunction<PointToPointErrorSE3, 3, 7>(new PointToPointErrorSE3(observed, worldPoint)));
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