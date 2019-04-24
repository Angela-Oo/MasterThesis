#pragma once

#include "mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>
#include "deformation_graph.h"
#include "../knn.h"
#include "../mesh_knn.h"

typedef ml::TriMeshf Mesh;
class EmbeddedDeformation
{
	Mesh _src;
	Mesh _dst;
	ceres::Solver::Options _options;
	DeformationGraph _deformation_graph;
	TriMeshKNN _nn_search;
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
	size_t _max_iterations = 100;
	long long _total_time_in_ms = 0;

	double a_rigid = 1000.;// 1.;// 1000;
	double a_smooth = 100.;// 0.1;// 100;
	double a_conf = 100.;// 1.;// 100;
	double a_fit = 0.1;
public:
	Mesh getDeformedPoints();
	bool finished();
	void solveIteration();
	Mesh solve();
	std::vector<ml::vec3f> getDeformationGraph();
public:
	// expect src and dst points to match at the same array position
	EmbeddedDeformation(const Mesh& src,
						const Mesh& dst,
						ceres::Solver::Options option,
						unsigned int number_of_deformation_nodes = 1000);
};