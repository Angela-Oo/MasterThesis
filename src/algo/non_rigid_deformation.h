#pragma once

#include "../mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>


class AsRigidAsPossible
{
	std::vector<ml::vec3f> _src;
	std::vector<ml::vec3f> _dst;
	ceres::Solver::Options _options;
public:
	// expect src and dst points to match at the same array position
	AsRigidAsPossible(const std::vector<ml::vec3f>& src,
					  const std::vector<ml::vec3f>& dst,
					  ceres::Solver::Options option);

	ml::mat4f solve();
};