#pragma once

#include "mLibInclude.h"
#include "deformation_graph.h"
#include "deformation_graph_knn.h"
#include <vector>


typedef ml::TriMeshf Mesh;


class DeformedPoint
{
public:
	ml::vec3f point;
	std::vector<vertex_index> nodes;
	std::vector<double> weights;
public:
	DeformedPoint() {}
	DeformedPoint(ml::vec3f p, const std::vector<vertex_index> & n, const std::vector<double> & w)
		: point(p)
		, nodes(n)
		, weights(w)
	{}
};

template<typename Node>
double nodeWeighting(const ml::vec3f & point, Node & node, double dmax)
{
	double normed_distance = ml::dist(point, node.position());
	double weight = std::pow(1. - (normed_distance / dmax), 2);
	return weight;
}

template<typename Node>
std::vector<double> nodeWeights(const ml::vec3f & point, std::vector<Node>& k_plus_1_nearest_nodes)
{
	auto last_node = k_plus_1_nearest_nodes[k_plus_1_nearest_nodes.size() - 1];
	double d_max = ml::dist(point, last_node.position());

	std::vector<double> weights;
	for (size_t i = 0; i < k_plus_1_nearest_nodes.size() - 1; ++i)
	{
		weights.push_back(nodeWeighting(point, k_plus_1_nearest_nodes[i], d_max));
	}

	double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
	std::for_each(weights.begin(), weights.end(), [sum](double & w) { w = w / sum; });
	return weights;
}





template<typename Graph, typename Node>
class DeformedMesh
{
private:
	const DeformationGraph<Graph, Node> & _deformation_graph;
	Mesh _mesh;
	std::vector<DeformedPoint> _deformable_points;
	//std::unique_ptr<GraphKNN<Graph, Node>> _deformation_graph_knn;
private:
	ml::vec3f deformPoint(size_t i);
public:
	Mesh deformPoints();
public:
	DeformedMesh(const Mesh & mesh, const DeformationGraph<Graph, Node> & deformation_graph);
};

template<typename Graph, typename Node>
ml::vec3f DeformedMesh<Graph, Node>::deformPoint(size_t i)
{
	auto & deformable_point = _deformable_points[i];

	ml::vec3f deformed_point = ml::vec3f::origin;
	auto & nodes = boost::get(node_t(), _deformation_graph._graph);
	for (size_t i = 0; i < deformable_point.nodes.size() - 1; ++i)
	{
		auto & node = nodes[deformable_point.nodes[i]];
		double w = deformable_point.weights[i];
		ml::vec3f transformed_point = node.deformPosition(deformable_point.point) * w;
		deformed_point += transformed_point;
	}

	ml::vec3f global_deformed_point = _deformation_graph._global_rigid_deformation.deformPosition(deformed_point);
	return global_deformed_point;
}

template<typename Graph, typename Node>
Mesh DeformedMesh<Graph, Node>::deformPoints()
{
	Mesh deformed_points = _mesh;
	for (size_t i = 0; i < _mesh.m_vertices.size(); ++i)
	{
		deformed_points.m_vertices[i].position = deformPoint(i);
	}
	return deformed_points;
}


template<typename Graph, typename Node>
DeformedMesh<Graph, Node>::DeformedMesh(const Mesh & mesh, const DeformationGraph<Graph, Node> & deformation_graph)
	: _mesh(mesh)
	, _deformation_graph(deformation_graph)
{
	//_deformation_graph_knn = std::make_unique<GraphKNN<Graph, Node>>(_deformation_graph._graph, _k + 1);

	_deformable_points.resize(_mesh.m_vertices.size());
	for (size_t i = 0; i < _mesh.m_vertices.size(); ++i)
	{		
		auto & point = _mesh.m_vertices[i].position;
		std::vector<vertex_index> knn_nodes_indices = _deformation_graph._deformation_graph_knn->k_nearest_indices(point, _deformation_graph._k + 1);

		auto & nodes = boost::get(node_t(), _deformation_graph._graph);
		std::vector<Node> knn_nodes;
		for (auto & i : knn_nodes_indices) {
			knn_nodes.push_back(nodes[i]);
		}
		auto weights = nodeWeights(point, knn_nodes);

		_deformable_points[i] = DeformedPoint(point, knn_nodes_indices, weights);
	}
}

