#pragma once

#include "../mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>


class AsRigidAsPossible
{
	std::vector<ml::vec3f> _src;
	std::vector<ml::vec3f> _dst;
	ceres::Solver::Options _options;
	std::vector<ml::vec3d> _rotations;
	std::vector<ml::vec3d> _solved_points;
private:
	std::vector<size_t> getNeighborIndices(size_t i);
public:
	// expect src and dst points to match at the same array position
	AsRigidAsPossible(const std::vector<ml::vec3f>& src,
					  const std::vector<ml::vec3f>& dst,
					  ceres::Solver::Options option);

	std::vector<ml::vec3f> solve();
};