#include "deformation_graph_knn.h"


DeformationGraphKNN::DeformationGraphKNN(const Graph & graph, unsigned int max_k)
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

	for (auto & p : nn_points)
	{
		delete[] p;
	}
	nn_points.clear();

}

Node DeformationGraphKNN::nearest(const ml::vec3f & p)
{
	auto index = nearest_index(p);
	return boost::get(node_t(), _graph)[index];
}

vertex_index DeformationGraphKNN::nearest_index(const ml::vec3f & p)
{
	unsigned int index = _neares_neighbor_search.nearest(p.array);
	return _node_graph_indices[index];
}

std::vector<vertex_index> DeformationGraphKNN::k_nearest_indices(const ml::vec3f & point, unsigned int k)
{
	std::vector<unsigned int> indices = _neares_neighbor_search.kNearest(point.array, k, 0.000001);
	std::vector<vertex_index> graph_indices;
	for (auto & i : indices) {
		graph_indices.push_back(_node_graph_indices[i]);
	}
	return graph_indices;
}

std::vector<Node> DeformationGraphKNN::k_nearest(const ml::vec3f & point, unsigned int k)
{
	std::vector<unsigned int> indices = _neares_neighbor_search.kNearest(point.array, k, 0.000001);
	std::vector<Node> graph_nodes;
	for (auto & i : indices) {
		auto graph_index = _node_graph_indices[i];
		graph_nodes.push_back(boost::get(node_t(), _graph)[graph_index]);
	}
	return graph_nodes;
}
