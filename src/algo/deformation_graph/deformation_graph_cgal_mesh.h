#pragma once

#include "mLibInclude.h"
#include "algo/mesh_simplification/deformation_graph_mesh.h"
#include <vector>
//#include "algo/registration/graph_node_type.h"

#include <CGAL/squared_distance_3.h> //for 3D functions

typedef ml::TriMeshf Mesh;

class NearestNodes
{
public:
	ml::vec3f point;
	std::vector<vertex_descriptor> nodes;
	std::vector<double> weights;
public:
	NearestNodes() {}
	NearestNodes(ml::vec3f p, const std::vector<vertex_descriptor> & n, const std::vector<double> & w)
		: point(p)
		, nodes(n)
		, weights(w)
	{}
};

class DeformationGraphCgalMesh
{
public:
	const int _k = 4;
	DeformationGraphMesh _mesh;
	//DeformationGraphMesh::Property_map<vertex_descriptor, std::shared_ptr<INode>> & _nodes;
	std::shared_ptr<INode> _global_rigid_deformation;
public:
	std::vector<double> weights(const ml::vec3f & point, std::vector<vertex_descriptor>& nearest_nodes_indices) const;
	std::vector<vertex_descriptor> nearestNodes(const ml::vec3f & point) const;
	ml::vec3f deformPoint(const ml::vec3f & point, const NearestNodes & nearest_nodes) const;
	Mesh::Vertex deformNode(vertex_descriptor node_index);
public:
	DeformationGraphCgalMesh() = default;
	// all mesh vertices will be deformation nodes
	DeformationGraphCgalMesh(const DeformationGraphMesh & nodes);
	DeformationGraphCgalMesh(const DeformationGraphMesh & graph, const std::shared_ptr<INode> & global_rigid_deformation);
	DeformationGraphCgalMesh(const DeformationGraphCgalMesh & deformation_graph);
	DeformationGraphCgalMesh & operator=(DeformationGraphCgalMesh other);
};

