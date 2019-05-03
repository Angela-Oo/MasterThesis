#pragma once

#include "mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>
#include "deformation_graph.h"
#include "algo/file_writer.h"
#include "algo/knn.h"
#include "algo/mesh_knn.h"
#include <ceres/rotation.h>

#include "template_deformation_graph.h"

class ARAPNode
{
private:
	ml::vec3f _g; // node position
	ml::vec3d _n; // normal vector
	ml::vec3d _r; // rotation matrix
	ml::vec3d _t; // translation vector	
	double _w; // weight
public:
	ml::vec3f & g() { return _g; }
	ml::vec3d & n() { return _n; }
	double * r() { return (&_r)->getData(); }
	double * t() { return (&_t)->getData(); }
	double * w() { return &_w; }
public:
	const ml::vec3f & position() const { return _g; };
	const ml::mat3d & rotation() const;
	const ml::vec3d & translation() const { return _t; }
	double weight() const { return _w; };
public:
	ml::vec3d deformedPosition() const;
	ml::vec3d deformedNormal() const;
	ml::vec3d deformPosition(const ml::vec3f & pos) const;
public:
	ARAPNode(const ml::vec3f & g, const ml::vec3d & n);
	ARAPNode(const ml::vec3f & g, const ml::vec3d & n, const ml::vec3d & r, const ml::vec3d & t);
	ARAPNode();
	ARAPNode(const ARAPNode& node) = default;
};


typedef boost::adjacency_list<
	boost::vecS, 
	boost::vecS, 
	boost::undirectedS, 
	boost::property<node_t, ARAPNode>> ARAPGraph;



typedef ml::TriMeshf Mesh;

class AsRigidAsPossible
{
	Mesh _src;
	Mesh _dst;
	ceres::Solver::Options _options;
	//DeformationGraph _deformation_graph;
	TemplateDeformationGraph<ARAPGraph, ARAPNode> _deformation_graph;
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
	//DeformationGraph & getDeformationGraph();
	TemplateDeformationGraph<ARAPGraph, ARAPNode> & getDeformationGraph();
public:
	// expect src and dst points to match at the same array position
	AsRigidAsPossible(const Mesh& src,
					  const Mesh& dst,
					  ceres::Solver::Options option,
					  unsigned int number_of_deformation_nodes = 1000,
					  std::shared_ptr<FileWriter> logger = nullptr);
};