#pragma once

#include "../../mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>
#include "deformation_graph.h"
#include "../knn.h"
#include "../mesh_knn.h"

//class AsRigidAsPossible
//{
//	std::vector<ml::vec3f> _src;
//	std::vector<ml::vec3f> _dst;
//	ceres::Solver::Options _options;
//	std::vector<ml::vec3d> _rotations;
//	std::vector<ml::vec3d> _solved_points;
//private:
//	std::vector<size_t> getNeighborIndices(size_t i, size_t size);
//	void solvePositions();
//	void solveRotation();
//public:
//	std::vector<ml::vec3f> solve();
//	// expect src and dst points to match at the same array position
//	AsRigidAsPossible(const std::vector<ml::vec3f>& src,
//					  const std::vector<ml::vec3f>& dst,
//					  ceres::Solver::Options option);
//};

typedef ml::TriMeshf Mesh;

class AsRigidAsPossible
{
	Mesh _src;
	Mesh _dst;
	ceres::Solver::Options _options;
	DeformationGraph _deformation_graph;
	TriMeshKNN _nn_search;
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
	size_t _max_iterations = 50;
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
	AsRigidAsPossible(const Mesh& src,
					  const Mesh& dst,
					  ceres::Solver::Options option,
					  unsigned int number_of_deformation_nodes = 1000);
};