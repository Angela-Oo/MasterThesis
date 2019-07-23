#include "render_deformation_graph.h"
#include "algo/registration/hsv_to_rgb.h"

using namespace Registration;
namespace Visualize {


std::tuple<double, double> getMeanAndMaxVertexCost(const SurfaceMesh & mesh, std::string cost_name)
{
	if (mesh.number_of_vertices() > 0 && mesh.property_map<vertex_descriptor, double>(cost_name).second) {
		auto costs = mesh.property_map<vertex_descriptor, double>(cost_name).first;
		double mean_cost = 0.;
		double max_cost = 0.0;
		for (auto v : mesh.vertices()) {
			mean_cost += costs[v];
			if (costs[v] > max_cost)
				max_cost = costs[v];
		}
		mean_cost /= mesh.number_of_vertices();
		return std::make_tuple(mean_cost, max_cost);
	}
	return std::make_tuple(0.1, 0.1);
}

std::tuple<double, double> getMeanAndMaxConfCost(const SurfaceMesh & mesh)
{
	return getMeanAndMaxVertexCost(mesh, "v:conf_cost");
}

std::tuple<double, double> getMeanAndMaxFitCost(const SurfaceMesh & mesh)
{
	return getMeanAndMaxVertexCost(mesh, "v:fit_cost");
}

std::tuple<bool, double, double> getMeanAndMaxSmoothCost(const SurfaceMesh & mesh)
{
	auto smooth_costs = mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	if (mesh.number_of_vertices() > 0 && smooth_costs.second) {	
		double mean_smooth_cost = 0.;
		double max_smooth_cost = 0.0;
		for (auto e : mesh.edges()) {
			mean_smooth_cost += smooth_costs.first[e];
			if (smooth_costs.first[e] > max_smooth_cost)
				max_smooth_cost = smooth_costs.first[e];
		}
		mean_smooth_cost /= mesh.number_of_edges();
		return std::make_tuple(true, mean_smooth_cost, max_smooth_cost);
	}
	return std::make_tuple(false, 0.1, 0.1);
}

double getReferenceCost(const SurfaceMesh & mesh, bool use_only_smooth_cost)
{
	if (mesh.property_map<edge_descriptor, double>("e:smooth_cost").second) {
		bool valid;
		double mean_smooth_cost;
		double max_smooth_cost;
		std::tie(valid, mean_smooth_cost, max_smooth_cost) = getMeanAndMaxSmoothCost(mesh);

		if (valid) {
			if (use_only_smooth_cost) {
				double reference_cost = mean_smooth_cost * 10.;
				//std::cout << "max smooth costs: " << max_smooth_cost << " reference smooth cost " << reference_cost << " ";
				return reference_cost;
			}
			else {
				double mean_fit_cost;
				double max_fit_cost;
				std::tie(mean_fit_cost, max_fit_cost) = getMeanAndMaxFitCost(mesh);
				auto reference_cost = std::max(mean_fit_cost, mean_smooth_cost) * 10.;
				std::cout << "max costs: fit " << max_fit_cost << " smooth " << max_smooth_cost << " reference cost " << reference_cost << " ";
				return reference_cost;
			}
		}
	}
	return 0.;
}

//-----------------------------------------------------------------------------

void setVertexColorOfUnusedVerticesToBlack(SurfaceMesh & mesh)
{	
	// if vertex was used for optimization flag is set use it
	auto vertex_used = mesh.property_map<vertex_descriptor, bool>("v:vertex_used");
	if (vertex_used.second) {
		auto colors = mesh.property_map<vertex_descriptor, ml::vec4f>("v:color").first;
		for (auto & v : mesh.vertices()) {
			if (!vertex_used.first[v]) {
				colors[v] = ml::RGBColor::Black.toVec4f();
			}
		}
	}
}

void setVertexColorWithSmallWeightsToWhite(SurfaceMesh & mesh)
{	
	auto deformations = mesh.property_map<vertex_descriptor, std::shared_ptr<IPositionDeformation>>("v:node");
	if (deformations.second) {
		auto colors = mesh.property_map<vertex_descriptor, ml::vec4f>("v:color").first;

		for (auto & v : mesh.vertices()) {
			if (deformations.first[v]->weight() < 0.5)
				colors[v] = ml::RGBColor::White.toVec4f();
		}
	}
}

void setVertexColorBasedOnFitCost(SurfaceMesh & mesh, double reference_cost)
{
	auto property_fit = mesh.property_map<vertex_descriptor, double>("v:fit_cost");
	if (property_fit.second)
	{
		auto fit_costs = property_fit.first;
		auto colors = mesh.property_map<vertex_descriptor, ml::vec4f>("v:color").first;

		for (auto & v : mesh.vertices())
		{
			double error = (reference_cost > 0.) ? (fit_costs[v] / reference_cost) : fit_costs[v];
			error = std::min(1., error);
			colors[v] = errorToRGB(error);
		}
	}
}


void setVertexColor(SurfaceMesh & mesh, double reference_cost, bool use_only_smooth_cost)
{
	// add color property if not already exists
	auto vertex_color = errorToRGB(0.);
	auto colors = mesh.add_property_map<vertex_descriptor, ml::vec4f>("v:color", vertex_color).first;
	if (!use_only_smooth_cost) {
		setVertexColorBasedOnFitCost(mesh, reference_cost);
	}
	//else {		
	//	for (auto & v : mesh.vertices())
	//	{
	//		colors[v] = errorToRGB(0.);
	//	}
	//}

	setVertexColorOfUnusedVerticesToBlack(mesh);
	setVertexColorWithSmallWeightsToWhite(mesh);
}


void setEdgeColor(SurfaceMesh & mesh, double reference_cost)
{
	if (mesh.property_map<edge_descriptor, double>("e:smooth_cost").second) {
		auto smooth_costs = mesh.property_map<edge_descriptor, double>("e:smooth_cost").first;
		auto edge_colors = mesh.property_map<edge_descriptor, ml::vec4f>("e:color").first;
		for (auto & e : mesh.edges())
		{
			double error = (reference_cost > 0.) ? (smooth_costs[e] / reference_cost) : smooth_costs[e];
			error = std::min(1., error);
			edge_colors[e] = errorToRGB(error);
		}
	}
}

//-----------------------------------------------------------------------------

void setDeformationGraphColor(SurfaceMesh & mesh, bool use_only_smooth_cost)
{
	double reference_cost = getReferenceCost(mesh, use_only_smooth_cost);
	setVertexColor(mesh, reference_cost, use_only_smooth_cost);
	setEdgeColor(mesh, reference_cost);
}

}