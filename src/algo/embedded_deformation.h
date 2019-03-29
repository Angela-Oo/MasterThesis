#pragma once

#include "../mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>

class EmbeddedDeformation
{
	std::vector<ml::vec3f> _src;
	std::vector<ml::vec3f> _dst;
	ceres::Solver::Options _options;
	std::vector<ml::mat3d> _matrix;
	std::vector<ml::vec3d> _solved_points;
private:
	std::vector<size_t> getNeighborIndices(size_t i, size_t size);
public:
	// expect src and dst points to match at the same array position
	EmbeddedDeformation(const std::vector<ml::vec3f>& src,
						const std::vector<ml::vec3f>& dst,
						ceres::Solver::Options option);

	std::vector<ml::vec3f> solve();
};