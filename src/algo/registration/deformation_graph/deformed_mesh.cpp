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