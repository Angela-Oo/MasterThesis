#include "deformation_graph.h"
#include <random>
#include "deformation_graph_knn.h"



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


std::vector<int> uniform_node_indices(size_t number_of_points, size_t number_of_nodes)
{
	size_t step_size = floor(number_of_points / number_of_nodes);

	std::vector<int> node_indices;
	for (size_t i = 0; i < number_of_nodes; ++i) {
		size_t x = step_size * i;
		node_indices.push_back(x);
	}
	return node_indices;
}
//
//Graph createGraphFromPoints(std::vector<ml::vec3f> points, size_t number_of_nodes, int k)
//{
//	Graph graph;
//	std::vector<int> node_point_indices = uniform_node_indices(points.size(), number_of_nodes);
//
//	for (int index : node_point_indices)
//	{
//		vertex_index v = boost::add_vertex(graph);
//		Node n(points[index]);
//		boost::put(boost::get(node_t(), graph), v, n);
//	}
//
//	DeformationGraphKNN knn(graph, k);
//	auto& nodes = boost::get(node_t(), graph);
//
//	for (auto & p : points) {
//		std::vector<vertex_index> nodes = knn.k_nearest_indices(p, k);
//
//		for (size_t i = 0; i < nodes.size() - 2; ++i) {
//			auto n1 = nodes[i];
//			auto n2 = nodes[i + 1];
//			if (boost::edge(n1, n2, graph).second == false)
//				boost::add_edge(n1, n2, graph);
//		}
//	}
//	return graph;
//}


double DeformationGraph::weight(const ml::vec3f & point, Node & node, double dmax)
{
	double normed_distance = ml::dist(point, node._g);
	double weight = std::pow(1. - (normed_distance / dmax), 2);
	return weight;
}

std::vector<double> DeformationGraph::weights(const ml::vec3f & point, std::vector<Node>& k_plus_1_nearest_nodes)
{
	auto last_node = k_plus_1_nearest_nodes[k_plus_1_nearest_nodes.size() - 1];
	double d_max = ml::dist(point, last_node._g);

	std::vector<double> weights;
	for (size_t i = 0; i < k_plus_1_nearest_nodes.size() - 1; ++i)
	{
		weights.push_back(weight(point, k_plus_1_nearest_nodes[i], d_max));
	}

	double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
	std::for_each(weights.begin(), weights.end(), [sum](double & w) { w = w / sum; });
	return weights;
}

ml::vec3f DeformationGraph::deformPoint(const ml::vec3f & point, std::vector<Node> & k_plus_1_nearest_nodes)
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


Mesh DeformationGraph::deformPoints(const Mesh & points)
{
	Mesh deformed_points = points;
	for (auto & p : deformed_points.getVertices())
	{
		std::vector<Node> knn_nodes = _knn->k_nearest(p.position, _k + 1);
		p.position = deformPoint(p.position, knn_nodes);
	}
	return deformed_points;
}

std::vector<ml::vec3f> DeformationGraph::getDeformationGraph()
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

DeformationGraph::DeformationGraph(const Mesh & points, size_t number_of_nodes)
	: _graph(0)
{
	auto & vertices = points.getVertices();
	std::vector<int> node_point_indices = uniform_node_indices(vertices.size(), number_of_nodes);

	for (int index : node_point_indices)
	{
		vertex_index v = boost::add_vertex(_graph);
		if (vertices.size() > index) {
			Node n(vertices[index].position, vertices[index].normal);
			boost::put(boost::get(node_t(), _graph), v, n);
		}
	}

	_knn = std::make_unique<DeformationGraphKNN>(_graph, _k + 1);
	for (auto & p : vertices) {
		std::vector<vertex_index> node_indices = _knn->k_nearest_indices(p.position, _k);

		for (size_t i = 0; i < node_indices.size() - 1; ++i) {
			auto n1 = node_indices[i];
			auto n2 = node_indices[i + 1];
			if (boost::edge(n1, n2, _graph).second == false)
				boost::add_edge(n1, n2, _graph);
		}
	}

	auto& nodes = boost::get(node_t(), _graph);
	int count = 0;
	for (auto vp = boost::vertices(_graph); vp.first != vp.second; ++vp.first) {
		Node& node = nodes[*vp.first];
		_global_rigid_deformation._g += node._g;
		count++;
	}
	_global_rigid_deformation._g /= count;
}
