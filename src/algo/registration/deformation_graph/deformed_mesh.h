#pragma once

#include "mLibInclude.h"
#include "deformation_graph.h"

namespace Registration
{

template <typename DeformationGraph>
SurfaceMesh deformationGraphToSurfaceMesh(const DeformationGraph & deformation_graph, bool color_based_on_cost, bool smooth_cost = true, bool fit_cost = false)
{
	SurfaceMesh mesh = deformation_graph._mesh;
	for (auto & v : mesh.vertices()) {
		auto deformed = deformation_graph.deformNode(v);
		mesh.point(v) = deformed.position();
	}
	return mesh;
}


template <typename DeformationGraph>
class DeformedMesh
{
private:
	const DeformationGraph & _deformation_graph;
	SurfaceMesh _mesh;
	unsigned int _k; // number of interpolated deformation graph nodes per vertex
private:
	NearestNodes createNearestNodes(vertex_descriptor v) const;
public:
	CGAL::Iterator_range<SurfaceMesh::Vertex_iterator> vertices() const;
	uint32_t number_of_vertices() const;
	Point point(SurfaceMesh::Vertex_index v) const;
	Vector normal(SurfaceMesh::Vertex_index v) const;
	Point deformed_point(SurfaceMesh::Vertex_index v) const;
	Vector deformed_normal(SurfaceMesh::Vertex_index v) const;
	NearestNodes & nearestNodes(SurfaceMesh::Vertex_index v) const;
	SurfaceMesh deformPoints();
public:
	DeformedMesh(const SurfaceMesh & mesh, const DeformationGraph & deformation_graph, unsigned int number_of_interpolation_neighbors);
};


template <typename DeformationGraph>
CGAL::Iterator_range<SurfaceMesh::Vertex_iterator> DeformedMesh<DeformationGraph>::vertices() const
{
	return _mesh.vertices();
}

template <typename DeformationGraph>
uint32_t DeformedMesh<DeformationGraph>::number_of_vertices() const
{
	return _mesh.number_of_vertices();
}

template <typename DeformationGraph>
Point DeformedMesh<DeformationGraph>::point(SurfaceMesh::Vertex_index v) const
{
	return _mesh.point(v);
}

template <typename DeformationGraph>
Vector DeformedMesh<DeformationGraph>::normal(SurfaceMesh::Vertex_index v) const
{
	auto normals = _mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
	return normals[v];
}

template <typename DeformationGraph>
Point DeformedMesh<DeformationGraph>::deformed_point(SurfaceMesh::Vertex_index v) const
{
	Point p = _deformation_graph.deformPoint(_mesh.point(v), nearestNodes(v));
	return p;
}

template <typename DeformationGraph>
Vector DeformedMesh<DeformationGraph>::deformed_normal(SurfaceMesh::Vertex_index v) const
{
	auto normals = _mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
	Vector n = _deformation_graph.deformNormal(normals[v], nearestNodes(v));
	return n;
}

template <typename DeformationGraph>
NearestNodes & DeformedMesh<DeformationGraph>::nearestNodes(SurfaceMesh::Vertex_index v) const
{
	auto nearest_nodes = _mesh.property_map<vertex_descriptor, NearestNodes>("v:nearest_nodes");
	assert(nearest_nodes.second);
	return nearest_nodes.first[v];
}

template <typename DeformationGraph>
SurfaceMesh DeformedMesh<DeformationGraph>::deformPoints()
{
	SurfaceMesh::Property_map<vertex_descriptor, NearestNodes> nearest_nodes;
	bool found;
	boost::tie(nearest_nodes, found) = _mesh.property_map<vertex_descriptor, NearestNodes>("v:nearest_nodes");

	SurfaceMesh deformed_points = _mesh;
	for (auto & v : _mesh.vertices()) {
		deformed_points.point(v) = _deformation_graph.deformPoint(_mesh.point(v), nearest_nodes[v]);
	}
	return deformed_points;
}

template <typename DeformationGraph>
NearestNodes DeformedMesh<DeformationGraph>::createNearestNodes(vertex_descriptor v) const
{
	auto point = _mesh.point(v);

	std::vector<vertex_descriptor> nearest_deformation_nodes = _deformation_graph.getKNearestNodes(point, _k + 1);

	// calculate weights

	// max distance
	vertex_descriptor last_node_descriptor = nearest_deformation_nodes[nearest_deformation_nodes.size() - 1];
	Point last_node = _deformation_graph.getNodePosition(last_node_descriptor);
	double d_max = std::sqrt(CGAL::squared_distance(point, last_node));
	if (nearest_deformation_nodes.size() < 2) {
		d_max = 1.;
		std::cout << "found only one nearest node" << std::endl;
	}

	// calculate weight per deformation node
	std::vector<std::pair<vertex_descriptor, double>> vertex_weight_vector;
	double sum = 0.;
	for (size_t i = 0; i < nearest_deformation_nodes.size() - 1; ++i)
	{
		vertex_descriptor v = nearest_deformation_nodes[i];
		Point node_point = _deformation_graph.getNodePosition(v);
		double distance = std::sqrt(CGAL::squared_distance(point, node_point));
		double weight = std::pow(1. - (distance / d_max), 2);
		vertex_weight_vector.push_back(std::make_pair(v, weight));
		sum += weight;
	}
	// divide by sum
	std::for_each(vertex_weight_vector.begin(), vertex_weight_vector.end(), [sum](std::pair<vertex_descriptor, double> & v_w) { v_w.second = v_w.second / sum; });

	if (vertex_weight_vector.size() < _k) {
		std::cout << "not enought nodes found" << std::endl;
	}
	return NearestNodes(point, vertex_weight_vector);
}



template <typename DeformationGraph>
DeformedMesh<DeformationGraph>::DeformedMesh(const SurfaceMesh & mesh, const DeformationGraph & deformation_graph, unsigned int number_of_interpolation_neighbors)
	: _mesh(mesh)
	, _deformation_graph(deformation_graph)
	, _k(number_of_interpolation_neighbors)
{
	SurfaceMesh::Property_map<vertex_descriptor, NearestNodes> nearest_nodes;
	bool created;
	boost::tie(nearest_nodes, created) = _mesh.add_property_map<vertex_descriptor, NearestNodes>("v:nearest_nodes", NearestNodes());
	//assert(created);

	for (auto & v : _mesh.vertices()) {
		nearest_nodes[v] = createNearestNodes(v);
	}
}


}