#pragma once

#include "mLibInclude.h"
#include "as_rigid_as_possible_node.h"
#include "deformation_graph.h"
#include "algo/file_writer.h"
#include "algo/mesh_knn.h"
#include <ceres/ceres.h>

typedef ml::TriMeshf Mesh;
typedef DeformationGraph<ARAPGraph, ARAPNode> ARAPDeformationGraph;

class AsRigidAsPossible
{
	Mesh _src;
	Mesh _dst;
	ceres::Solver::Options _options;
	ARAPDeformationGraph _deformation_graph;
	TriMeshKNN _nn_search;
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
	size_t _max_iterations = 200;
	long long _total_time_in_ms = 0;
	double a_smooth = 100.;// 0.1;// 100;
	double a_conf = 100.;// 1.;// 100;
	double a_fit = 0.1;
	std::shared_ptr<FileWriter> _logger;
public:
	Mesh getDeformedPoints();
	bool finished();
	void solveIteration();
	Mesh solve();
	ARAPDeformationGraph & getDeformationGraph();
public:
	AsRigidAsPossible(const Mesh& src,
					  const Mesh& dst,
					  ceres::Solver::Options option,
					  unsigned int number_of_deformation_nodes = 1000,
					  std::shared_ptr<FileWriter> logger = nullptr);
};