#pragma once

#include "mLibInclude.h"
#include <vector>
#include "graph_node_type.h"

template <typename Graph, typename Node>
class GraphKNN {
	const Graph & _graph;
	std::vector<vertex_index> _node_graph_indices;
	ml::NearestNeighborSearchFLANN<float> _neares_neighbor_search;
public:
	GraphKNN(const Graph & graph, unsigned int max_k = 1);
	vertex_index nearest_index(const ml::vec3f & point);
	Node nearest(const ml::vec3f & point);
	std::vector<vertex_index> k_nearest_indices(const ml::vec3f & point, unsigned int k);
	std::vector<Node> k_nearest(const ml::vec3f & point, unsigned int k);
};


template <typename Graph, typename Node>
GraphKNN<Graph, Node>::GraphKNN(const Graph & graph, unsigned int max_k)
	: _graph(graph)
	, _neares_neighbor_search(50, 12)
{
	std::vector<const float*> nn_points;

	auto all_nodes = boost::get(node_t(), _graph);
	for (auto vp = boost::vertices(_graph); vp.first != vp.second; ++vp.first) {
		auto p = all_nodes[*vp.first].position();
		_node_graph_indices.push_back(*vp.first);
		float * point = new float[3];
		point[0] = p[0];
		point[1] = p[1];
		point[2] = p[2];
		nn_points.push_back(point);
	}
	_neares_neighbor_search.init(nn_points, 3, max_k);

	for (auto & p : nn_points) {
		delete[] p;
	}
	nn_points.clear();
}

template <typename Graph, typename Node>
Node GraphKNN<Graph, Node>::nearest(const ml::vec3f & p)
{
	auto index = nearest_index(p);
	return boost::get(node_t(), _graph)[index];
}

template <typename Graph, typename Node>
vertex_index GraphKNN<Graph, Node>::nearest_index(const ml::vec3f & p)
{
	unsigned int index = _neares_neighbor_search.nearest(p.array);
	return _node_graph_indices[index];
}

template <typename Graph, typename Node>
std::vector<vertex_index> GraphKNN<Graph, Node>::k_nearest_indices(const ml::vec3f & point, unsigned int k)
{
	std::vector<unsigned int> indices = _neares_neighbor_search.kNearest(point.array, k, 0.000001);
	std::vector<vertex_index> graph_indices;
	for (auto & i : indices) {
		graph_indices.push_back(_node_graph_indices[i]);
	}
	return graph_indices;
}

template <typename Graph, typename Node>
std::vector<Node> GraphKNN<Graph, Node>::k_nearest(const ml::vec3f & point, unsigned int k)
{
	std::vector<unsigned int> indices = _neares_neighbor_search.kNearest(point.array, k, 0.000001);
	std::vector<Node> graph_nodes;
	for (auto & i : indices) {
		auto graph_index = _node_graph_indices[i];
		graph_nodes.push_back(boost::get(node_t(), _graph)[graph_index]);
	}
	return graph_nodes;
}
