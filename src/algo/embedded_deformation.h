#pragma once

#include "../mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>
#include "deformation_graph.h"
#include "knn.h"

class EmbeddedDeformationLine
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
	EmbeddedDeformationLine(const std::vector<ml::vec3f>& src,
							const std::vector<ml::vec3f>& dst,
							ceres::Solver::Options option);

	std::vector<ml::vec3f> solve();
};



class EmbeddedDeformation
{
	std::vector<ml::vec3f> _src;
	std::vector<ml::vec3f> _dst;
	ceres::Solver::Options _options;
	DeformationGraph _deformation_graph;
	KNN _nn_search;
	double _current_cost = 1.;
	double _current_tol = 1.;
	size_t _solve_iteration = 0;
	size_t _max_iterations = 50;
	long long _total_time_in_ms = 0;
public:
	std::vector<ml::vec3f> getDeformedPoints();
	bool finished();
	void solveIteration();
	std::vector<ml::vec3f> solve();
	std::vector<ml::vec3f> getDeformationGraph();
public:
	// expect src and dst points to match at the same array position
	EmbeddedDeformation(const std::vector<ml::vec3f>& src,
						const std::vector<ml::vec3f>& dst,
						ceres::Solver::Options option);	
};