#pragma once

#include "mLibInclude.h"
#include "graph_node_type.h"
#include <vector>

#include "algo/deformation_graph/deformation_graph_cgal_mesh.h"
typedef ml::TriMeshf Mesh;

//struct NodeGradient
//{
//	ml::vec3d translation;
//	ml::vec3d rotation;
//	double w;
//};
//
//struct ARAPGradient
//{
//	std::vector<ml::vec3f> point;
//	std::vector<NodeGradient> fit_point_to_point_gradient;
//	std::vector<NodeGradient> fit_point_to_plane_gradient;
//	std::vector<NodeGradient> smooth_gradient;
//	std::vector<NodeGradient> all;
//};
//
//
struct Edge
{
	ml::vec3f source_point;
	ml::vec3f target_point;
	double cost = 0.;
};

class IRegistration
{
public:
	virtual bool finished() = 0;
	virtual bool solveIteration() = 0;
	virtual bool solve() = 0;

	//virtual std::map<std::string, std::map<vertex_index, std::vector<double>>> residuals()
	//{ return std::map<std::string, std::map<vertex_index, std::vector<double>>>(); };
public:
	virtual const Mesh & getSource() = 0;
	virtual const Mesh & getTarget() = 0;
	virtual Mesh getDeformedPoints() = 0;
	virtual Mesh getInverseDeformedPoints() = 0;
public:
	virtual std::vector<ml::vec3f> getFixedPostions() { return std::vector<ml::vec3f>(); }
	virtual std::vector<Edge> getDeformationGraph() { return std::vector<Edge>(); }
	virtual Mesh getDeformationGraphMesh() { return Mesh(); };
public:
	virtual ~IRegistration() = default;
};


class ITestRegistration
{
public:
	virtual bool finished() = 0;
	virtual bool solveIteration() = 0;
	virtual bool solve() = 0;
public:
	virtual const SurfaceMesh & getSource() = 0;
	virtual const SurfaceMesh & getTarget() = 0;
	virtual SurfaceMesh getDeformedPoints() = 0;
	//virtual SurfaceMesh getInverseDeformedPoints() = 0;
public:
	virtual std::vector<Point> getFixedPostions() { return std::vector<Point>(); }
	virtual const DG::DeformationGraphCgalMesh & getDeformationGraph() = 0;
	//virtual Mesh getDeformationGraphMesh() { return Mesh(); };
public:
	virtual ~ITestRegistration() = default;
};

