#pragma once

#include "mLibInclude.h"
#include "deformation_graph_knn.h"
#include <vector>


typedef ml::TriMeshf Mesh;

template<typename Graph, typename Node>
class DeformationGraph
{
private:
	const int _k = 4;
public:
	Node _global_rigid_deformation;
	Graph _graph;
	std::unique_ptr<GraphKNN<Graph, Node>> _deformation_graph_knn;
private:
	double weight(const ml::vec3f & point, Node & node, double dmax);
	std::vector<double> weights(const ml::vec3f & point, std::vector<Node>& k_plus_1_nearest_nodes);
	ml::vec3f deformPoint(const ml::vec3f & point, std::vector<Node> & k_plus_1_nearest_nodes);
	std::vector<int> uniform_node_indices(size_t number_of_points, size_t number_of_nodes);
public:
	Mesh deformPoints(const Mesh & points);
	std::vector<ml::vec3f> getDeformationGraph();
public:
	DeformationGraph() = default;
	DeformationGraph(const Mesh & nodes, size_t number_of_nodes);
	DeformationGraph(const Graph & graph, const Node & global_rigid_deformation);
	DeformationGraph(const DeformationGraph<Graph, Node> & deformation_graph);
	DeformationGraph & operator=(DeformationGraph<Graph, Node> other);
};

template<typename Graph, typename Node>
std::vector<int> DeformationGraph<Graph, Node>::uniform_node_indices(size_t number_of_points, size_t number_of_nodes)
{
	int step_size = static_cast<int>(floor(number_of_points / number_of_nodes));

	std::vector<int> node_indices;
	for (int i = 0; i < number_of_nodes; ++i) {
		int x = step_size * i;
		node_indices.push_back(x);
	}
	return node_indices;
}


template<typename Graph, typename Node>
double DeformationGraph<Graph, Node>::weight(const ml::vec3f & point, Node & node, double dmax)
{
	double normed_distance = ml::dist(point, node.position());
	double weight = std::pow(1. - (normed_distance / dmax), 2);
	return weight;
}

template<typename Graph, typename Node>
std::vector<double> DeformationGraph<Graph, Node>::weights(const ml::vec3f & point, std::vector<Node>& k_plus_1_nearest_nodes)
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
ml::vec3f DeformationGraph<Graph, Node>::deformPoint(const ml::vec3f & point, std::vector<Node> & k_plus_1_nearest_nodes)
{
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
Mesh DeformationGraph<Graph, Node>::deformPoints(const Mesh & points)
{
	Mesh deformed_points = points;
	for (auto & p : deformed_points.getVertices())
	{
		std::vector<Node> knn_nodes = _deformation_graph_knn->k_nearest(p.position, _k + 1);
		p.position = deformPoint(p.position, knn_nodes);
	}
	return deformed_points;
}

template<typename Graph, typename Node>
std::vector<ml::vec3f> DeformationGraph<Graph, Node>::getDeformationGraph()
{
	std::vector<ml::vec3f> points;
	auto & nodes = boost::get(node_t(), _graph);

	for (auto vp = boost::vertices(_graph); vp.first != vp.second; ++vp.first) {
		Node& src_i = nodes[*vp.first];
		ml::vec3f pos = src_i.deformedPosition();
		ml::vec3f global_pos = _global_rigid_deformation.deformPosition(pos);
		points.push_back(global_pos);
	}
	return points;
}

template<typename Graph, typename Node>
DeformationGraph<Graph, Node>::DeformationGraph(const Mesh & points, size_t number_of_nodes)
	: _graph(0)
{
	auto & vertices = points.getVertices();
	std::vector<int> node_point_indices = uniform_node_indices(vertices.size(), number_of_nodes);

	for (int index : node_point_indices)
	{
		vertex_index v = boost::add_vertex(_graph);
		if (vertices.size() > index) {
			Node n(vertices[index].position, vertices[index].normal.getNormalized());
			boost::put(boost::get(node_t(), _graph), v, n);
		}
	}

	_deformation_graph_knn = std::make_unique<GraphKNN<Graph, Node>>(_graph, _k + 1);
	for (auto & p : vertices) {
		std::vector<vertex_index> node_indices = _deformation_graph_knn->k_nearest_indices(p.position, _k);

		for (size_t i = 0; i < node_indices.size() - 1; ++i) {
			auto n1 = node_indices[i];
			auto n2 = node_indices[i + 1];
			if (boost::edge(n1, n2, _graph).second == false)
				boost::add_edge(n1, n2, _graph);
		}
	}

	auto& nodes = boost::get(node_t(), _graph);
	int count = 0;
	ml::vec3f global_position = ml::vec3f::origin;
	for (auto vp = boost::vertices(_graph); vp.first != vp.second; ++vp.first) {
		Node& node = nodes[*vp.first];
		global_position += node.position();
		count++;
	}
	global_position /= static_cast<float>(count);
	_global_rigid_deformation = Node(global_position, ml::vec3d::eZ);
}



template<typename Graph, typename Node>
DeformationGraph<Graph, Node>::DeformationGraph(const Graph & graph, const Node & global_rigid_deformation)
	: _graph(graph)
	, _global_rigid_deformation(global_rigid_deformation)
{
	_deformation_graph_knn = std::make_unique<GraphKNN<Graph, Node>>(_graph, _k + 1);
}


template<typename Graph, typename Node>
DeformationGraph<Graph, Node>::DeformationGraph(const DeformationGraph<Graph, Node> & deformation_graph)
	: _global_rigid_deformation(deformation_graph._global_rigid_deformation)
	, _graph(deformation_graph._graph)
{
	_deformation_graph_knn = std::make_unique<GraphKNN<Graph, Node>>(_graph, _k + 1);
}

template<typename Graph, typename Node>
DeformationGraph<Graph, Node> & DeformationGraph<Graph, Node>::operator=(DeformationGraph<Graph, Node> other)
{
	if (&other == this)
		return *this;

	_global_rigid_deformation = other._global_rigid_deformation;
	_graph = other._graph;
	_deformation_graph_knn = std::make_unique<GraphKNN<Graph, Node>>(_graph, _k + 1);
	return *this;
}


template<typename Node>
Node inverseDeformationNode(const Node & node)
{
	Node inverse_deformation_node(node.deformedPosition(), node.deformedNormal(), node.rotation().getInverse(), -node.translation());
	return inverse_deformation_node;
}

template<typename Graph, typename Node>
DeformationGraph<Graph, Node> inverteDeformationGraph(const DeformationGraph<Graph, Node> & deformation_graph)
{
	Graph inverse_deformation_graph = deformation_graph._graph;

	auto& nodes = boost::get(node_t(), inverse_deformation_graph);

	for (auto vp = boost::vertices(inverse_deformation_graph); vp.first != vp.second; ++vp.first) {
		Node& node = nodes[*vp.first];
		node = inverseDeformationNode(node);
	}
	Node inverse_global_rigid_deformation = inverseDeformationNode(deformation_graph._global_rigid_deformation);

	return DeformationGraph<Graph, Node>(inverse_deformation_graph, inverse_global_rigid_deformation);
}
