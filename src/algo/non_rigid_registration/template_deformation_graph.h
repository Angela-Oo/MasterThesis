#pragma once

#include "mLibInclude.h"
#include <vector>
#include "graph_node_type.h"

typedef ml::TriMeshf Mesh;


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

	for (auto & p : nn_points){
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








typedef ml::TriMeshf Mesh;

template<typename Graph, typename Node>
class TemplateDeformationGraph
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
	TemplateDeformationGraph() = default;
	TemplateDeformationGraph(const Mesh & nodes, size_t number_of_nodes);
	TemplateDeformationGraph(const Graph & graph, const Node & global_rigid_deformation);
	TemplateDeformationGraph(const TemplateDeformationGraph<Graph, Node> & deformation_graph);
	TemplateDeformationGraph & operator=(TemplateDeformationGraph<Graph, Node> other);
};

template<typename Graph, typename Node>
std::vector<int> TemplateDeformationGraph<Graph, Node>::uniform_node_indices(size_t number_of_points, size_t number_of_nodes)
{
	size_t step_size = floor(number_of_points / number_of_nodes);

	std::vector<int> node_indices;
	for (size_t i = 0; i < number_of_nodes; ++i) {
		size_t x = step_size * i;
		node_indices.push_back(x);
	}
	return node_indices;
}


template<typename Graph, typename Node>
double TemplateDeformationGraph<Graph, Node>::weight(const ml::vec3f & point, Node & node, double dmax)
{
	double normed_distance = ml::dist(point, node.position());
	double weight = std::pow(1. - (normed_distance / dmax), 2);
	return weight;
}

template<typename Graph, typename Node>
std::vector<double> TemplateDeformationGraph<Graph, Node>::weights(const ml::vec3f & point, std::vector<Node>& k_plus_1_nearest_nodes)
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
ml::vec3f TemplateDeformationGraph<Graph, Node>::deformPoint(const ml::vec3f & point, std::vector<Node> & k_plus_1_nearest_nodes)
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
Mesh TemplateDeformationGraph<Graph, Node>::deformPoints(const Mesh & points)
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
std::vector<ml::vec3f> TemplateDeformationGraph<Graph, Node>::getDeformationGraph()
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
TemplateDeformationGraph<Graph, Node>::TemplateDeformationGraph<Graph, Node>(const Mesh & points, size_t number_of_nodes)
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

	//_deformation_graph_knn = std::make_unique<DeformationGraphKNN>(_graph, _k + 1);
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
		//_global_rigid_deformation.position() += node.position();
		count++;
	}
	//_global_rigid_deformation.position() /= count;
	global_position /= count;
	_global_rigid_deformation = Node(global_position, ml::vec3d::eZ);
}



template<typename Graph, typename Node>
TemplateDeformationGraph<Graph, Node>::TemplateDeformationGraph<Graph, Node>(const Graph & graph, const Node & global_rigid_deformation)
	: _graph(graph)
	, _global_rigid_deformation(global_rigid_deformation)
{
	_deformation_graph_knn = std::make_unique<DeformationGraphKNN>(_graph, _k + 1);
}


template<typename Graph, typename Node>
TemplateDeformationGraph<Graph, Node>::TemplateDeformationGraph<Graph, Node>(const TemplateDeformationGraph<Graph, Node> & deformation_graph)
	: _global_rigid_deformation(deformation_graph._global_rigid_deformation)
	, _graph(deformation_graph._graph)
{
	_deformation_graph_knn = std::make_unique<DeformationGraphKNN>(_graph, _k + 1);
}

template<typename Graph, typename Node>
TemplateDeformationGraph<Graph, Node> & TemplateDeformationGraph<Graph, Node>::operator=(TemplateDeformationGraph<Graph, Node> other)
{
	if (&other == this)
		return *this;

	_global_rigid_deformation = other._global_rigid_deformation;
	_graph = other._graph;
	_deformation_graph_knn = std::make_unique<DeformationGraphKNN>(_graph, _k + 1);
	return *this;
}

