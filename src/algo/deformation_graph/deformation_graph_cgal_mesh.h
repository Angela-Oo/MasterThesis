#pragma once

#include "mLibInclude.h"
#include "algo/mesh_simplification/deformation_graph_mesh.h"
#include <vector>

#include "nearest_neighbor_search.h"
#include "i_node.h"
#include <CGAL/squared_distance_3.h> //for 3D functions

typedef ml::TriMeshf Mesh;

namespace DG
{

class NearestNodes
{
public:
	Point point;
	std::vector<vertex_descriptor> nodes;
	std::vector<double> weights;
public:
	NearestNodes() {}
	NearestNodes(Point p, const std::vector<vertex_descriptor> & n, const std::vector<double> & w)
		: point(p)
		, nodes(n)
		, weights(w)
	{}
};


class NodeAndPoint
{
public:
	std::shared_ptr<INode> _deformation;
	Point _point;
	Direction _normal;
};

Point deformNodePosition(Point point, Vector translation);
Direction deformNodeNormal(Direction normal, Matrix rotation);
Point deformPositionAtNode(Point point, Point node_position, Matrix node_rotation, Vector node_translation);
Direction deformNormalAtNode(Direction normal, Matrix node_rotation);

Point deformNodePosition(NodeAndPoint point);
Direction deformNodeNormal(NodeAndPoint point);
Point deformPositionAtNode(Point point, NodeAndPoint node);
Direction deformNormalAtNode(Direction normal, NodeAndPoint node);

class DeformationGraphCgalMesh
{
public:
	const int _k = 4;
	DeformationGraphMesh _mesh;
	Point _global_center;
	std::shared_ptr<INode> _global_deformation;
	std::unique_ptr<NearestNeighborSearch> _knn_search;
public:
	std::vector<double> weights(const Point & point, std::vector<vertex_descriptor>& nearest_nodes_indices) const;
	std::vector<vertex_descriptor> nearestNodes(const Point & point) const;
	Point deformPoint(const Point & point, const NearestNodes & nearest_nodes) const;
public:
	Point deformedPosition(vertex_descriptor vertex_index) const;
	Direction deformedNormal(vertex_descriptor vertex_index) const;
	Point deformedPositionAtNode(vertex_descriptor vertex_index, const Point & pos) const;
	Direction deformedNormalAtNode(vertex_descriptor vertex_index, const Direction & normal) const;
public:
	NodeAndPoint deformNode(vertex_descriptor node_index) const;
	NodeAndPoint getNode(vertex_descriptor node_index);
	//DeformationGraphMesh::Property_map<vertex_descriptor, std::shared_ptr<INode>> getDeformations();
public:
	DeformationGraphCgalMesh() = default;
	// all mesh vertices will be deformation nodes
	DeformationGraphCgalMesh(const SurfaceMesh & nodes, std::function<std::shared_ptr<INode>()> create_node);
	DeformationGraphCgalMesh(const DeformationGraphMesh & graph, const std::shared_ptr<INode> & global_rigid_deformation);
	DeformationGraphCgalMesh(const DeformationGraphCgalMesh & deformation_graph);
	DeformationGraphCgalMesh & operator=(DeformationGraphCgalMesh other);
};


SurfaceMesh deformationGraphToSurfaceMesh(const DeformationGraphCgalMesh & deformation_graph);

class DeformedMesh
{
private:
	const DeformationGraphCgalMesh & _deformation_graph;
	SurfaceMesh _mesh;
public:
	SurfaceMesh deformPoints();
public:
	DeformedMesh(const SurfaceMesh & mesh, const DeformationGraphCgalMesh & deformation_graph);
};


}