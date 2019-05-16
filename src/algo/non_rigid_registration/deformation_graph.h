#pragma once

#include "mLibInclude.h"
#include "deformation_graph_knn.h"
#include "node_weighting.h"

#include <vector>


typedef ml::TriMeshf Mesh;



class NearestNodes
{
public:
	ml::vec3f point;
	std::vector<vertex_index> nodes;
	std::vector<double> weights;
public:
	NearestNodes() {}
	NearestNodes(ml::vec3f p, const std::vector<vertex_index> & n, const std::vector<double> & w)
		: point(p)
		, nodes(n)
		, weights(w)
	{}
};

template<typename Graph, typename Node>
class DeformationGraph
{
public:
	const int _k = 4;
	Node _global_rigid_deformation;
	Graph _graph;
	std::unique_ptr<GraphKNN<Graph, Node>> _deformation_graph_knn;
private:
	std::vector<int> uniform_node_indices(size_t number_of_points, size_t number_of_nodes);
public:
	std::vector<double> weights(const ml::vec3f & point, std::vector<vertex_index>& nearest_nodes_indices) const;
	ml::vec3f deformPoint(const ml::vec3f & point, const NearestNodes & nearest_nodes) const;
public:
	std::vector<ml::vec3f> getDeformationGraph();
	std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> getDeformationGraphEdges();
public:
	DeformationGraph() = default;
	// all mesh vertices will be deformation nodes
	DeformationGraph(const Mesh & nodes);
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
std::vector<double> DeformationGraph<Graph, Node>::weights(const ml::vec3f & point, std::vector<vertex_index>& nearest_nodes_indices) const
{
	auto & nodes = boost::get(node_t(), _graph);
	std::vector<ml::vec3f> knn_nodes;
	for (auto & i : nearest_nodes_indices) {
		knn_nodes.push_back(nodes[i].position());
	}
	return nodeDistanceWeighting(point, knn_nodes);	
}

template<typename Graph, typename Node>
ml::vec3f DeformationGraph<Graph, Node>::deformPoint(const ml::vec3f & point, const NearestNodes & nearest_nodes) const
{
	ml::vec3f deformed_point = ml::vec3f::origin;

	auto & nodes = boost::get(node_t(), _graph);

	for (size_t i = 0; i < nearest_nodes.nodes.size() - 1; ++i)
	{
		auto & node = nodes[nearest_nodes.nodes[i]];
		double w = nearest_nodes.weights[i];
		ml::vec3f transformed_point = node.deformPosition(point) * w;
		deformed_point += transformed_point;
	}

	ml::vec3f global_deformed_point = _global_rigid_deformation.deformPosition(deformed_point);
	return global_deformed_point;
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
std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> DeformationGraph<Graph, Node>::getDeformationGraphEdges()
{
	std::vector<ml::vec3f> source_points;
	std::vector<ml::vec3f> target_points;
	auto & nodes = boost::get(node_t(), _graph);
	boost::graph_traits<Graph>::edge_iterator ei, ei_end;
	for (boost::tie(ei, ei_end) = boost::edges(_graph); ei != ei_end; ++ei) 
	{
		Node & n_i = nodes[boost::source(*ei, _graph)];
		ml::vec3f pos_i = n_i.deformedPosition();
		pos_i = _global_rigid_deformation.deformPosition(pos_i);
		source_points.push_back(pos_i);

		Node & n_j = nodes[boost::target(*ei, _graph)];
		ml::vec3f pos_j = n_j.deformedPosition();
		pos_j = _global_rigid_deformation.deformPosition(pos_j);
		target_points.push_back(pos_j);
	}
	return std::make_pair(source_points, target_points);
}


template<typename Graph, typename Node>
DeformationGraph<Graph, Node>::DeformationGraph(const Mesh & mesh)
	: _graph(0)
{
	auto & vertices = mesh.getVertices();

	std::map<int, vertex_index> index_to_vertex_index;
	for (int index = 0; index < vertices.size(); ++index)
	{
		vertex_index v = boost::add_vertex(_graph);
		Node n(index, vertices[index].position, vertices[index].normal.getNormalized());
		boost::put(boost::get(node_t(), _graph), v, n);
		index_to_vertex_index[index] = v;
	}

	auto & indices = mesh.getIndices();
	for (int index = 0; index < indices.size(); ++index)
	{
		auto n1 = index_to_vertex_index[indices[index][0]];
		auto n2 = index_to_vertex_index[indices[index][1]];
		auto n3 = index_to_vertex_index[indices[index][2]];
		if (boost::edge(n1, n2, _graph).second == false)
			boost::add_edge(n1, n2, _graph);
		if (boost::edge(n2, n3, _graph).second == false)
			boost::add_edge(n2, n3, _graph);
		if (boost::edge(n3, n1, _graph).second == false)
			boost::add_edge(n3, n1, _graph);
	}
	_deformation_graph_knn = std::make_unique<GraphKNN<Graph, Node>>(_graph, _k + 1);

	auto& nodes = boost::get(node_t(), _graph);
	int count = 0;
	ml::vec3f global_position = ml::vec3f::origin;
	for (auto vp = boost::vertices(_graph); vp.first != vp.second; ++vp.first) {
		Node& node = nodes[*vp.first];
		global_position += node.position();
		count++;
	}
	global_position /= static_cast<float>(count);
	_global_rigid_deformation = Node(-1, global_position, ml::vec3d::eZ);
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
			Node n(index, vertices[index].position, vertices[index].normal.getNormalized());
			boost::put(boost::get(node_t(), _graph), v, n);
		}
	}

	auto& nodes = boost::get(node_t(), _graph);
	_deformation_graph_knn = std::make_unique<GraphKNN<Graph, Node>>(_graph, _k + 1);
	for (auto vp = boost::vertices(_graph); vp.first != vp.second; ++vp.first) {
		auto node_index = *vp.first;
		Node& node = nodes[node_index];
		std::vector<vertex_index> node_indices = _deformation_graph_knn->k_nearest_indices(node.g(), _k);

		for (size_t i = 0; i < node_indices.size(); ++i) {
			auto n1 = node_index;
			auto n2 = node_indices[i];
			if (boost::edge(n1, n2, _graph).second == false)
				boost::add_edge(n1, n2, _graph);
		}
	}


	int count = 0;
	ml::vec3f global_position = ml::vec3f::origin;
	for (auto vp = boost::vertices(_graph); vp.first != vp.second; ++vp.first) {
		Node& node = nodes[*vp.first];
		global_position += node.position();
		count++;
	}
	global_position /= static_cast<float>(count);
	_global_rigid_deformation = Node(-1, global_position, ml::vec3d::eZ);
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
	Node inverse_deformation_node(node, true);
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
