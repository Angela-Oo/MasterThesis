#include "stdafx.h"
#include "deformed_mesh.h"
#include "algo/registration/hsv_to_rgb.h"

namespace DG {


double getMeanFitCost(const SurfaceMesh & mesh)
{
	auto property_map_fit_costs = mesh.property_map<vertex_descriptor, double>("v:fit_cost");
	if (property_map_fit_costs.second) {
		auto fit_costs = property_map_fit_costs.first;
		double mean_fit_cost = 0.;
		for (auto v : mesh.vertices()) {
			mean_fit_cost += fit_costs[v];
		}
		mean_fit_cost /= mesh.number_of_vertices();
		return mean_fit_cost;
	}
	std::cout << " no fit property " << std::endl;
	return 0.;
}

double getReferenceCost(const SurfaceMesh & mesh)
{
	auto fit_costs = mesh.property_map<vertex_descriptor, double>("v:fit_cost").first;
	auto smooth_costs = mesh.property_map<edge_descriptor, double>("e:smooth_cost").first;
	auto conf_costs = mesh.property_map<vertex_descriptor, double>("v:conf_cost").first;

	double mean_fit_cost = 0.;
	double max_fit_cost = 0.0;
	double max_conf_cost = 0.0;

	for (auto v : mesh.vertices()) {
		mean_fit_cost += fit_costs[v];
		if (fit_costs[v] > max_fit_cost)
			max_fit_cost = fit_costs[v];
		if (conf_costs[v] > max_conf_cost)
			max_conf_cost = conf_costs[v];
	}
	mean_fit_cost /= mesh.number_of_vertices();

	double mean_smooth_cost = 0.;
	double max_smooth_cost = 0.0;
	for (auto e : mesh.edges()) {
		mean_smooth_cost += smooth_costs[e];
		if (smooth_costs[e] > max_smooth_cost)
			max_smooth_cost = smooth_costs[e];
	}
	mean_smooth_cost /= mesh.number_of_edges();

	auto k_mean_cost = std::max(mean_fit_cost, mean_smooth_cost);
	k_mean_cost *= 10.;

	std::cout << "max costs: fit " << max_fit_cost
		<< " conf " << max_conf_cost
		<< " smooth " << max_smooth_cost
		<< " k mean visualize cost " << k_mean_cost << " ";

	return k_mean_cost;
}

void setVertexColorBasedOnFitCost(SurfaceMesh & mesh, double reference_cost)
{
	auto property_fit = mesh.property_map<vertex_descriptor, double>("v:fit_cost");
	if (property_fit.second)
	{
		auto fit_costs = property_fit.first;

		// add color property if not already exists
		auto colors = mesh.add_property_map<vertex_descriptor, ml::vec4f>("v:color").first;

		// if vertex was used for optimization flag is set use it
		auto vertex_used = mesh.property_map<vertex_descriptor, bool>("v:vertex_used");

		for (auto & v : mesh.vertices())
		{
			double error = (reference_cost > 0.) ? (fit_costs[v] / reference_cost) : fit_costs[v];
			error = std::min(1., error);
			colors[v] = errorToRGB(error);

			if (vertex_used.second) {
				if (!vertex_used.first[v]) {
					colors[v] = ml::RGBColor::Black.toVec4f();
				}
			}
			//if (node.weight() < 0.7)
			//	vertex.color = ml::RGBColor::White.toVec4f();
		}
	}
	else {
		std::cout << "no fit property" << std::endl;
	}
}


SurfaceMesh deformationGraphToSurfaceMesh(const DeformationGraph & deformation_graph)
{
	SurfaceMesh mesh = deformation_graph._mesh;
	auto normals = mesh.property_map<vertex_descriptor, Direction>("v:normal").first;
	for (auto & v : mesh.vertices()) {
		auto deformed = deformation_graph.deformNode(v);
		mesh.point(v) = deformed._point;
		normals[v] = deformed._normal;
	}

	// color
	auto reference_cost = getReferenceCost(mesh);
	setVertexColorBasedOnFitCost(mesh, reference_cost);
	//auto fit_costs = mesh.property_map<vertex_descriptor, double>("v:fit_cost").first;
	//auto colors = mesh.property_map<vertex_descriptor, ml::vec4f>("v:color").first;
	//for (auto & v : mesh.vertices()) 
	//{
	//	double error = (reference_cost > 0.) ? (fit_costs[v] / reference_cost) : fit_costs[v];
	//	error = std::min(1., error);
	//	colors[v] = errorToRGB(error);

	//	//if (node.weight() < 0.7)
	//	//	vertex.color = ml::RGBColor::White.toVec4f();
	//	//else if (!node._found_nearest_point)
	//	//	vertex.color = ml::RGBColor::Black.toVec4f();
	//	//mesh.m_vertices.push_back(vertex);
	//}

	auto smooth_costs = mesh.property_map<edge_descriptor, double>("e:smooth_cost").first;
	auto edge_colors = mesh.property_map<edge_descriptor, ml::vec4f>("e:color").first;
	for (auto & e : mesh.edges())
	{
		double error = (reference_cost > 0.) ? (smooth_costs[e] / reference_cost) : smooth_costs[e];
		error = std::min(1., error);
		edge_colors[e] = errorToRGB(error);
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