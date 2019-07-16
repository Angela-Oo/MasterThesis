#include "stdafx.h"
#include "deformed_mesh.h"
#include "algo/registration/hsv_to_rgb.h"

namespace DG {


SurfaceMesh deformationGraphToSurfaceMesh(const DeformationGraph & deformation_graph, bool color_based_on_cost, bool smooth_cost, bool fit_cost)
{
	SurfaceMesh mesh = deformation_graph._mesh;
	auto normals = mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
	for (auto & v : mesh.vertices()) {
		auto deformed = deformation_graph.deformNode(v);
		mesh.point(v) = deformed._point;
		normals[v] = deformed._normal;
	}
	return mesh;
}

CGAL::Iterator_range<SurfaceMesh::Vertex_iterator> DeformedMesh::vertices() const
{
	return _mesh.vertices();
}

uint32_t DeformedMesh::number_of_vertices() const
{
	return _mesh.number_of_vertices();
}

Point DeformedMesh::point(SurfaceMesh::Vertex_index v) const
{
	return _mesh.point(v);
}

Vector DeformedMesh::normal(SurfaceMesh::Vertex_index v) const
{
	auto normals = _mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
	return normals[v];
}

Point DeformedMesh::deformed_point(SurfaceMesh::Vertex_index v) const
{
	Point p = _deformation_graph.deformPoint(_mesh.point(v), nearestNodes(v));
	return p;
}

Vector DeformedMesh::deformed_normal(SurfaceMesh::Vertex_index v) const
{
	auto normals = _mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
	Vector n = _deformation_graph.deformNormal(normals[v], nearestNodes(v));
	return n;
}

NearestNodes & DeformedMesh::nearestNodes(SurfaceMesh::Vertex_index v) const
{
	auto nearest_nodes = _mesh.property_map<vertex_descriptor, NearestNodes>("v:nearest_nodes");
	assert(nearest_nodes.second);
	return nearest_nodes.first[v];
}

std::vector<DG::PositionAndDeformation> DeformedMesh::deformations(SurfaceMesh::Vertex_index v) const
{
	std::vector<DG::PositionAndDeformation> d;
	auto & nodes = nearestNodes(v);
	for (auto n : nodes.nodes) {
		d.emplace_back(_deformation_graph.getNode(n));
	}
	return d;
}

SurfaceMesh DeformedMesh::deformPoints()
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


DeformedMesh::DeformedMesh(const SurfaceMesh & mesh, const DeformationGraph & deformation_graph)
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