#pragma once

#include "deformation_graph.h"
#include "mesh/mesh_definition.h"

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
NearestNodes createNearestNodes(const DeformationGraph & deformation_graph, Point point, unsigned int k)
{
	std::vector<vertex_descriptor> nearest_deformation_nodes = deformation_graph.getKNearestNodes(point, k + 1);

	// calculate weights

	std::vector<std::pair<vertex_descriptor, double>> vertex_weight_vector;
	double sum = 0.;
	double d_max = 1.;

	auto radius_map = deformation_graph._mesh.property_map<vertex_descriptor, double>("v:radius");
	if (false) { //radius_map.second) {
		// calculate weight per deformation node
		// wj(point, vi, ri) = max(0., (1 -  d(vi, point)^2 / ri)^3)
		
		for (size_t i = 0; i < nearest_deformation_nodes.size() - 1; ++i)
		{
			vertex_descriptor v = nearest_deformation_nodes[i];
			Point node_point = deformation_graph.getDeformation(v).position();

			double distance = CGAL::squared_distance(point, node_point);
			//double radius = pow((radius_map.first[v] * 2.0), 2);
			double radius = pow((radius_map.first[v]), 2);
			double weight = 1. - (distance / radius);
			weight = std::pow(weight, 3);
			weight = std::max(0., weight);
			vertex_weight_vector.push_back(std::make_pair(v, weight));
			sum += weight;
		}
	}
	else if (false) { //radius_map.second) {
		// calculate weight per deformation node
		// wj(point, vi, ri) = max(0., (1 -  d(vi, point)^2 / ri)^3)
		std::vector<double> distances;
		std::vector<Point> points;
		for (size_t i = 0; i < nearest_deformation_nodes.size() - 1; ++i)
		{
			vertex_descriptor v = nearest_deformation_nodes[i];
			Point node_point = deformation_graph.getDeformation(v).position();
			double distance = CGAL::squared_distance(point, node_point);
			distances.push_back(distance);
			points.push_back(node_point);
		}
		double radius = *std::max_element(distances.begin(), distances.end());
		
		for (size_t i = 0; i < points.size() - 1; ++i)
		{
			double d = CGAL::squared_distance(points[i], points[i+1]);
			if (d > radius)
				radius = d;
		}

		//radius = *std::max_element(distances.begin(), distances.end());

		for (size_t i = 0; i < distances.size(); ++i)
		{
			auto v = nearest_deformation_nodes[i];
			double weight = 1. - (distances[i] / radius);
			weight = std::pow(weight, 3);
			weight = std::max(0., weight);
			vertex_weight_vector.push_back(std::make_pair(v, weight));
			sum += weight;
		}
	}
	else {
		// max distance d_max distance to the k+1 node
		vertex_descriptor last_node_descriptor = nearest_deformation_nodes.back();
		Point last_node = deformation_graph.getDeformation(last_node_descriptor).position();
		d_max = std::sqrt(CGAL::squared_distance(point, last_node));
		if (nearest_deformation_nodes.size() < 2) {
			d_max = 1.;
			std::cout << "found only one nearest node" << std::endl;
		}

		// calculate weight per deformation node
		// wj(vi) = (1. - || vi - gj || / d_max)^2
		for (size_t i = 0; i < nearest_deformation_nodes.size() - 1; ++i)
		{
			vertex_descriptor v = nearest_deformation_nodes[i];
			Point node_point = deformation_graph.getDeformation(v).position();

			double distance = std::sqrt(CGAL::squared_distance(point, node_point));
			double weight = 1. - (distance / d_max);
			weight = std::pow(weight, 2);
			vertex_weight_vector.push_back(std::make_pair(v, weight));
			sum += weight;
		}
	}


	// normalize weights by dividing through the sum of all weights
	std::for_each(vertex_weight_vector.begin(), vertex_weight_vector.end(), [sum](std::pair<vertex_descriptor, double> & v_w) { v_w.second = v_w.second / sum; });

	if (vertex_weight_vector.size() < k) {
		std::cout << "not enought nodes found" << std::endl;
	}
	if (vertex_weight_vector.size() != k)
	{
		std::cout << "more vertices than expected " << k << std::endl;
	}
	return NearestNodes(point, vertex_weight_vector);
}



template <typename DeformationGraph>
class DeformedMesh
{
private:
	const DeformationGraph & _deformation_graph;
	SurfaceMesh _mesh;
	unsigned int _k; // number of interpolated deformation graph nodes per vertex
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
		auto point = _mesh.point(v);
		nearest_nodes[v] = createNearestNodes(_deformation_graph, point, _k);
	}
}


}