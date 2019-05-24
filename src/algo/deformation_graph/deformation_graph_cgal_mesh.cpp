#include "deformation_graph_cgal_mesh.h"
#include "algo/registration/node_weighting.h"
#include <cassert>

namespace DeformationGraph {


Point deformNodePosition(Point point, Vector translation)
{
	Vector v = point - CGAL::ORIGIN;
	return CGAL::ORIGIN + v + translation;
}

Direction deformNodeNormal(Direction normal, Matrix rotation)
{
	return rotation(normal);
}

Point deformPositionAtNode(Point point, Point node_position, Matrix node_rotation, Vector node_translation)
{
	Vector rotated_point = node_rotation(point - node_position);
	Vector moved_position = (node_position - CGAL::ORIGIN) + node_translation;
	return CGAL::ORIGIN + moved_position + rotated_point;
}

Direction deformNormalAtNode(Direction normal, Matrix node_rotation)
{
	return node_rotation(normal);
}

Point deformNodePosition(NodeAndPoint point)
{
	Vector v = point._point - CGAL::ORIGIN;
	return CGAL::ORIGIN + v + point._deformation->translation();
}
Direction deformNodeNormal(NodeAndPoint point)
{
	return deformNodeNormal(point._normal, point._deformation->rotation());
}
Point deformPositionAtNode(Point point, NodeAndPoint node)
{
	Vector rotated_point = node._deformation->rotation()(point - node._point);
	Vector moved_position = (node._point - CGAL::ORIGIN) + node._deformation->translation();
	return CGAL::ORIGIN + moved_position + rotated_point;
}
Direction deformNormalAtNode(Direction normal, NodeAndPoint node)
{
	return node._deformation->rotation()(normal);
}



std::vector<double> nodeDistanceWeighting(const ml::vec3f & point, const std::vector<ml::vec3f>& node_positions)
{
	auto last_node = node_positions[node_positions.size() - 1];
	double d_max = ml::dist(point, last_node);

	std::vector<double> weights;
	for (size_t i = 0; i < node_positions.size() - 1; ++i)
	{
		double normed_distance = ml::dist(point, node_positions[i]);
		double weight = std::pow(1. - (normed_distance / d_max), 2);
		weights.push_back(weight);
	}

	double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
	std::for_each(weights.begin(), weights.end(), [sum](double & w) { w = w / sum; });
	return weights;
}



std::vector<double> DeformationGraphCgalMesh::weights(const Point & point, std::vector<vertex_descriptor>& nearest_nodes) const
{
	vertex_descriptor last_node_descriptor = nearest_nodes[nearest_nodes.size() - 1];
	Point last_node = _mesh.point(last_node_descriptor);
	double d_max = sqrt(CGAL::squared_distance(point, last_node));

	std::vector<double> weights;
	for (size_t i = 0; i < nearest_nodes.size() - 1; ++i)
	{
		Point node_point = _mesh.point(nearest_nodes[i]);
		double normed_distance = sqrt(CGAL::squared_distance(point, node_point));
		double weight = std::pow(1. - (normed_distance / d_max), 2);
		weights.push_back(weight);
	}
	double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
	std::for_each(weights.begin(), weights.end(), [sum](double & w) { w = w / sum; });
	return weights;
}


std::vector<vertex_descriptor> DeformationGraphCgalMesh::nearestNodes(const Point & point) const
{
	Neighbor_search search = _knn_search->search(point);
	vertex_descriptor nearest_node_index = search.begin()->first;

	auto property_map_nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<INode>>("node");
	assert(property_map_nodes.second);
	auto & nodes = property_map_nodes.first;

	auto getNodeDistances = [&](vertex_descriptor vertex_index) {
		std::map<vertex_descriptor, double> node_distance;
		for (auto & v : _mesh.vertices_around_target(_mesh.halfedge(vertex_index)))
		{
			auto neighbor_point = _mesh.point(v);
			auto point = _mesh.point(vertex_index);
			node_distance[v] = CGAL::squared_distance(point, neighbor_point);
		}
		return node_distance;
	};

	std::map<vertex_descriptor, double> node_distance = getNodeDistances(nearest_node_index);
	if (node_distance.size() < _k) {
		for (auto & n : node_distance) {
			auto more_distance = getNodeDistances(n.first);
			node_distance.insert(more_distance.begin(), more_distance.end());
		}
	}

	std::vector<std::pair<vertex_descriptor, double>> sorted_node_distance;
	for (auto & d : node_distance) {
		sorted_node_distance.push_back(d);
	}

	std::sort(sorted_node_distance.begin(), sorted_node_distance.end(),
			  [](const std::pair<vertex_descriptor, double> & rhs, const std::pair<vertex_descriptor, double> & lhs)
	{
		return rhs.second < lhs.second;
	});

	std::vector<vertex_descriptor> indices;
	indices.push_back(nearest_node_index);
	for (int i = 0; i < _k && i < sorted_node_distance.size(); ++i)
		indices.push_back(sorted_node_distance[i].first);
	return indices;
}


Point DeformationGraphCgalMesh::deformPoint(const Point & point, const NearestNodes & nearest_nodes) const
{
	Vector deformed_point(0.,0.,0.);

	auto & property_map_nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<INode>>("node");
	assert(property_map_nodes.second);
	auto & nodes = property_map_nodes.first;

	DeformationGraphMesh::Property_map<vertex_descriptor, std::shared_ptr<INode>> PropertyMapNodes;
	for (size_t i = 0; i < nearest_nodes.nodes.size() - 1; ++i)
	{
		auto vertex_index = nearest_nodes.nodes[i];
		double w = nearest_nodes.weights[i];

		Vector transformed_point = deformedPositionAtNode(vertex_index, point) - CGAL::ORIGIN;
		transformed_point *= w;
		deformed_point += transformed_point;
	}

	Point global_deformed_point = deformPositionAtNode(CGAL::ORIGIN + deformed_point,
													   _global_center, 
													   _global_deformation->rotation(),
													   _global_deformation->translation());
	return global_deformed_point;
}

Point DeformationGraphCgalMesh::deformedPosition(vertex_descriptor vertex_index) const
{
	auto nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<INode>>("v:node").first;
	return deformNodePosition(_mesh.point(vertex_index), nodes[vertex_index]->translation());
}

Direction DeformationGraphCgalMesh::deformedNormal(vertex_descriptor vertex_index) const
{
	auto nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<INode>>("v:node").first;
	auto vertex_normals = _mesh.property_map<vertex_descriptor, Direction>("v:normals").first;

	return deformNodeNormal(vertex_normals[vertex_index], nodes[vertex_index]->rotation());
}

Point DeformationGraphCgalMesh::deformedPositionAtNode(vertex_descriptor vertex_index, const Point & pos) const
{
	std::shared_ptr<INode> node = (_mesh.property_map<vertex_descriptor, std::shared_ptr<INode>>("v:node").first)[vertex_index];
	return deformPositionAtNode(pos, _mesh.point(vertex_index), node->rotation(), node->translation());
}

Direction DeformationGraphCgalMesh::deformedNormalAtNode(vertex_descriptor vertex_index, const Direction & normal) const
{
	std::shared_ptr<INode> node = (_mesh.property_map<vertex_descriptor, std::shared_ptr<INode>>("v:node").first)[vertex_index];
	
	return deformNormalAtNode(normal, node->rotation());
}

NodeAndPoint DeformationGraphCgalMesh::deformNode(vertex_descriptor node_index)
{
	auto & nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<INode>>("node").first;

	Point pos = deformedPosition(node_index);
	Direction normal = deformedNormal(node_index);
	Point global_pos = deformPositionAtNode(pos, _global_center, _global_deformation->rotation(), _global_deformation->translation());
	Direction global_normal = deformNormalAtNode(normal, _global_deformation->rotation());

	NodeAndPoint n;
	n._point = global_pos;
	n._normal = global_normal;
	return n;
	//Mesh::Vertex v;
	//v.position = global_pos;
	//v.normal = global_normal;
	//return v;
}

NodeAndPoint DeformationGraphCgalMesh::getNode(vertex_descriptor node_index)
{
	NodeAndPoint node;
	std::shared_ptr<INode> n = _mesh.property_map<vertex_descriptor, std::shared_ptr<INode>>("v:node").first[node_index];
	node._deformation = n;
	node._point = _mesh.point(node_index);
	node._normal = _mesh.property_map<vertex_descriptor, Direction>("v:normal").first[node_index];
	return node;
}


DeformationGraphCgalMesh::DeformationGraphCgalMesh(const DeformationGraphMesh & mesh, std::function<std::shared_ptr<INode>()> create_node)
	: _mesh(mesh)
{
	_knn_search = std::make_unique<NearestNeighborSearch>(_mesh);

	SurfaceMesh::Property_map<vertex_descriptor, std::shared_ptr<INode>> nodes;
	bool created;
	boost::tie(nodes, created) = _mesh.add_property_map<vertex_descriptor, std::shared_ptr<INode>>("v:nodes", create_node());
	assert(created);
	auto normals = _mesh.property_map<vertex_descriptor, Direction>("v:normals").first;

	Vector global_position(0., 0., 0.);
	for (auto & v : _mesh.vertices()) {
		auto point = _mesh.point(v);
		auto normal = normals[v];// _mesh.normal[v];
		nodes[v] = create_node();

		global_position += point - CGAL::ORIGIN;
	}
	global_position /= _mesh.number_of_vertices();

	_global_center = CGAL::ORIGIN + global_position;
	_global_deformation = create_node();
}


DeformationGraphCgalMesh::DeformationGraphCgalMesh(const SurfaceMesh & graph, const std::shared_ptr<INode> & global_deformation)
	: _mesh(graph)
	, _global_deformation(global_deformation)
{

	//_nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<INode>>("node");
	//_deformation_graph_knn = std::make_unique<GraphKNN<Graph, Node>>(_graph, _k + 1);
}


DeformationGraphCgalMesh::DeformationGraphCgalMesh(const DeformationGraphCgalMesh & deformation_graph)
	: _global_deformation(deformation_graph._global_deformation)
	, _global_center(deformation_graph._global_center)
	, _mesh(deformation_graph._mesh)
{
	//_deformation_graph_knn = std::make_unique<GraphKNN<Graph, Node>>(_graph, _k + 1);
}

DeformationGraphCgalMesh & DeformationGraphCgalMesh::operator=(DeformationGraphCgalMesh other)
{
	if (&other == this)
		return *this;

	_global_deformation = other._global_deformation;
	_global_center = other._global_center;
	_mesh = other._mesh;
	//_deformation_graph_knn = std::make_unique<GraphKNN<Graph, Node>>(_graph, _k + 1);
	return *this;
}











SurfaceMesh DeformedMesh::deformPoints()
{
	SurfaceMesh::Property_map<vertex_descriptor, NearestNodes> nearest_nodes;
	bool found;
	boost::tie(nearest_nodes, found) = _mesh.property_map<vertex_descriptor, NearestNodes>("v:nearest_nodes");

	SurfaceMesh deformed_points = _mesh;
	for (auto & v : _mesh.vertices()) {
		_mesh.point(v) = _deformation_graph.deformPoint(_mesh.point(v), nearest_nodes[v]);
	}
	return deformed_points;
}


DeformedMesh::DeformedMesh(const SurfaceMesh & mesh, const DeformationGraphCgalMesh & deformation_graph)
	: _mesh(mesh)
	, _deformation_graph(deformation_graph)
{
	SurfaceMesh::Property_map<vertex_descriptor, NearestNodes> nearest_nodes;
	bool created;
	boost::tie(nearest_nodes, created) = _mesh.add_property_map<vertex_descriptor, NearestNodes>("v:nearest_nodes", NearestNodes());
	assert(created);

	for (auto & v : _mesh.vertices()) {
		auto point = _mesh.point(v);
		std::vector<vertex_descriptor> knn_nodes_indices = _deformation_graph.nearestNodes(point);
		std::vector<double> weights = _deformation_graph.weights(point, knn_nodes_indices);
		nearest_nodes[v] = NearestNodes(point, knn_nodes_indices, weights);
	}
}


}