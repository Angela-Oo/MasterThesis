#pragma once

#include "mLibInclude.h"
//#include "deformation_graph_knn.h"
#include "algo/mesh_knn.h"
#include <vector>

typedef ml::TriMeshf Mesh;

template<typename Graph, typename Node>
class DeformationGraphMesh
{
private:
	const int _k = 4;
public:
	Node _global_rigid_deformation;
	std::vector<Node> _nodes;
	Graph _graph;
	MeshKNearestNeighbor<Graph, Graph::VertexHandle, Graph::Vertex> _knn;
private:
	double weight(const ml::vec3f & point, Node & node, double dmax);
	std::vector<double> weights(const ml::vec3f & point, std::vector<Node>& k_plus_1_nearest_nodes);
	ml::vec3f deformPoint(const ml::vec3f & point);
public:
	Mesh deformPoints(const Mesh & points);
public:
	DeformationGraphMesh() = default;
	// all mesh vertices will be deformation nodes
	DeformationGraphMesh(const Graph & graph);
};



template<typename Graph, typename Node>
double DeformationGraphMesh<Graph, Node>::weight(const ml::vec3f & point, Node & node, double dmax)
{
	double normed_distance = ml::dist(point, node.position());
	double weight = std::pow(1. - (normed_distance / dmax), 2);
	return weight;
}

template<typename Mesh, typename Node>
std::vector<double> DeformationGraphMesh<Mesh, Node>::weights(const ml::vec3f & point, std::vector<Node>& k_plus_1_nearest_nodes)
{
	auto last_node = k_plus_1_nearest_nodes[k_plus_1_nearest_nodes.size() - 1];
	double d_max = ml::dist(point, last_node.position());

	std::vector<double> weights;
	for (size_t i = 0; i < k_plus_1_nearest_nodes.size() - 1; ++i)
	{
		weights.push_back(weight(point, k_plus_1_nearest_nodes[i], d_max));
	}

	double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
	std::for_each(weights.begin(), weights.end(), [sum](double & w) { w = w / sum; });
	return weights;
}

template<typename Graph, typename Node>
ml::vec3f DeformationGraphMesh<Graph, Node>::deformPoint(const ml::vec3f & point)
{
	//std::vector<Node> k_plus_1_nearest_nodes = _deformation_graph_knn->k_nearest(p.position, _k + 1);
	auto vertex_handle = _knn.nearest_index(point);
	// todo find adjacen vertex and select the k nearest

	std::vector<double> w = weights(point, k_plus_1_nearest_nodes);

	ml::vec3f deformed_point = ml::vec3f::origin;
	for (size_t i = 0; i < k_plus_1_nearest_nodes.size() - 1; ++i)
	{
		auto & node = k_plus_1_nearest_nodes[i];
		ml::vec3f transformed_point = node.deformPosition(point) * w[i];
		deformed_point += transformed_point;
	}

	ml::vec3f global_deformed_point = _global_rigid_deformation.deformPosition(deformed_point);
	return global_deformed_point;
}

template<typename Graph, typename Node>
Mesh DeformationGraphMesh<Graph, Node>::deformPoints(const Mesh & points)
{
	Mesh deformed_points = points;
	for (auto & p : deformed_points.getVertices())
	{
		p.position = deformPoint(p.position, knn_nodes);
	}
	return deformed_points;
}


template<typename Graph, typename Node>
DeformationGraphMesh<Graph, Node>::DeformationGraphMesh(const Graph & mesh)
	: _graph(mesh)
{
	ml::vec3f global_position = ml::vec3f::origin;
	int count = 0;

	auto & vertices = mesh.getVertices();
	for (int index = 0; index < vertices.size(); ++index)
	{
		auto position = vertices[index].position;
		auto normal = vertices[index].normal.getNormalized();
		_nodes.emplace_back(Node(index, position, normal);
		global_position += position;
		count++;
	}
	global_position /= static_cast<float>(count);
	_global_rigid_deformation = Node(-1, global_position, ml::vec3d::eZ);

	_knn = MeshKNearestNeighbor(_graph, _k + 1);
	//_deformation_graph_knn = std::make_unique<GraphKNN<Graph, Node>>(_graph, _k + 1);
}



