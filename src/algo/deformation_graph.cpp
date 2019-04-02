#include "deformation_graph.h"
#include <random>
#include "knn.h"

ml::vec3d Node::deformedPosition()
{
	return (_r * _g) + _t;
}

Node::Node(ml::vec3f g)
	: _g(g)
	, _r(ml::mat3d::identity())
	, _t(ml::vec3d::origin)
{}

Node::Node()
	: _g(ml::vec3f::origin)
	, _r(ml::mat3d::identity())
	, _t(ml::vec3d::origin)
{}

std::vector<int> uniform_sample_node_indices(size_t number_of_points, size_t number_of_nodes)
{
	std::random_device rd;  // seed for the random number engine
	std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()

	std::uniform_int_distribution<> distribution(0, number_of_points - 1);
	std::vector<int> node_indices;
	for (size_t i = 0; i < number_of_nodes; ++i) {
		node_indices.push_back(distribution(gen));
	}
	return node_indices;
}

double DeformationGraph::weight(const ml::vec3f & point, Node & node, double dmax)
{
	double normed_distance = ml::dist(point, node._g);
	double weight = std::pow(1. - (normed_distance / dmax), 2);
	return weight;
}

std::vector<double> DeformationGraph::weights(const ml::vec3f & point, std::vector<Node>& nodes)
{
	auto last_node = nodes[nodes.size() - 1];
	double d_max = ml::dist(point, last_node._g);

	std::vector<double> weights;
	for (size_t i = 0; i < nodes.size() - 1; ++i)
	{
		weights.push_back(weight(point, nodes[i], d_max));
	}

	double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
	std::for_each(weights.begin(), weights.end(), [sum](double & w) { w = w / sum; });
	return weights;
}

ml::vec3f DeformationGraph::deformPoint(const ml::vec3f & point, std::vector<vertex_index> & k_nearest_node_indices)
{
	auto all_nodes = boost::get(node_t(), _graph);
	std::vector<Node> nodes;
	for (size_t i = 0; i < k_nearest_node_indices.size(); ++i)
	{
		nodes.push_back(all_nodes[k_nearest_node_indices[i]]);
	}

	std::vector<double> w = weights(point, nodes);

	ml::vec3f deformed_point = ml::vec3f::origin;
	for (size_t i = 0; i < nodes.size() - 1; ++i)
	{
		auto & node = nodes[i];
		ml::vec3f transformed_point = ((node._r *(point - node._g)) + node._g + node._t) * w[i];
		deformed_point += transformed_point;
	}
	return deformed_point;
}

std::vector<ml::vec3f> DeformationGraph::getDeformedPoints()
{
	std::vector<ml::vec3f> deformed_points;
	for (size_t i = 0; i < _points.size(); ++i) {
		deformed_points.push_back(deformPoint(_points[i], _k_nearest_nodes_of_points[i]));
	}
	return deformed_points;
}

DeformationGraph::DeformationGraph(const std::vector<ml::vec3f> & points, size_t number_of_nodes)
	: _points(points)
	, _graph(0)
{
	std::vector<int> node_point_indices = uniform_sample_node_indices(points.size(), number_of_nodes);

	std::vector<ml::vec3f> node_positions;
	std::vector<vertex_index> node_indices;
	for (int index : node_point_indices)
	{
		vertex_index v = boost::add_vertex(_graph);
		node_indices.push_back(v);
		node_positions.push_back(_points[index]);
		Node n(_points[index]);
		boost::put(boost::get(node_t(), _graph), v, n);
	}

	KNN knn(node_positions, k);
	for (auto & p : _points) {
		std::vector<unsigned int> nodes = knn.k_nearest_indices(p, k);
		std::vector<vertex_index> k_nearest_node_indices_of_p;
		for (auto n : nodes) {
			k_nearest_node_indices_of_p.push_back(node_indices[n]);
		}
		_k_nearest_nodes_of_points.push_back(k_nearest_node_indices_of_p);
		
		for (size_t i = 0; i < nodes.size() - 1; ++i) {
			auto n1 = node_indices[nodes[i]];
			auto n2 = node_indices[nodes[i + 1]];
			if(boost::edge(n1, n2, _graph).second == false)
				boost::add_edge(n1, n2, _graph);
		}
	}
}



