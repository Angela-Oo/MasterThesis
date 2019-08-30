#include "render_deformation_graph.h"
#include "algo/registration/util/hsv_to_rgb.h"

using namespace Registration;
namespace Visualize {


std::tuple<bool, double, double> getMeanAndMaxVertexCost(const SurfaceMesh & mesh, std::string cost_name)
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
		return std::make_tuple(true, mean_cost, max_cost);
	}
	return std::make_tuple(false, 0.1, 0.1);
}

std::tuple<bool, double, double> getMeanAndMaxEdgeValue(const SurfaceMesh & mesh, std::string property_name)
{
	auto values = mesh.property_map<edge_descriptor, double>(property_name);

	if (mesh.number_of_vertices() > 0 && values.second) {
		double mean = 0.;
		double max = 0.0;
		for (auto e : mesh.edges()) {
			mean += values.first[e];
			if (values.first[e] > max)
				max = values.first[e];
		}
		mean /= mesh.number_of_edges();
		return std::make_tuple(true, mean, max);
	}
	return std::make_tuple(false, 0.1, 0.1);
}

std::tuple<bool, double, double> getMeanAndMaxConfCost(const SurfaceMesh & mesh)
{
	return getMeanAndMaxVertexCost(mesh, "v:conf_cost");
}

std::tuple<bool, double, double> getMeanAndMaxFitCost(const SurfaceMesh & mesh)
{
	return getMeanAndMaxVertexCost(mesh, "v:fit_cost");
}

std::tuple<bool, double, double> getMeanAndMaxSmoothCost(const SurfaceMesh & mesh)
{
	return getMeanAndMaxEdgeValue(mesh, "e:smooth_cost");
}

std::tuple<bool, double, double> getMeanAndMaxRigidityWeight(const SurfaceMesh & mesh)
{
	return getMeanAndMaxEdgeValue(mesh, "e:rigidity");
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

//void setVertexColorWithSmallWeightsToWhite(SurfaceMesh & mesh)
//{	
//	auto deformations = mesh.property_map<vertex_descriptor, PositionDeformation>("v:node_deformation");
//	if (deformations.second) {
//		auto colors = mesh.property_map<vertex_descriptor, ml::vec4f>("v:color").first;
//
//		for (auto & v : mesh.vertices()) {
//			if (deformations.first[v]->weight() < 0.5)
//				colors[v] = ml::RGBColor::White.toVec4f();
//		}
//	}
//}

void setVertexColorBasedOnFitCost(SurfaceMesh & mesh)
{
	bool valid;
	double mean_cost;
	double max_cost;
	std::tie(valid, mean_cost, max_cost) = getMeanAndMaxVertexCost(mesh, "v:fit_cost");

	auto fit_costs = mesh.property_map<vertex_descriptor, double>("v:fit_cost").first;
	if (valid)
	{
		auto colors = mesh.property_map<vertex_descriptor, ml::vec4f>("v:color").first;
		for (auto & v : mesh.vertices())
		{
			double error = (max_cost > 0.) ? (fit_costs[v] / max_cost) : fit_costs[v];
			error = std::min(1., error);
			colors[v] = errorToRGB(error);
		}
	}
}


void setVertexColor(SurfaceMesh & mesh, VertexColor dg_vertex_color)
{
	// add color property if not already exists
	auto vertex_color = errorToRGB(0.);
	auto colors = mesh.add_property_map<vertex_descriptor, ml::vec4f>("v:color", vertex_color).first;
	if (dg_vertex_color == VertexColor::FitCost) {
		setVertexColorBasedOnFitCost(mesh);
	}
	setVertexColorOfUnusedVerticesToBlack(mesh);
}


void setEdgeColorToSmoothCost(SurfaceMesh & mesh)
{
	auto edge_colors = mesh.property_map<edge_descriptor, ml::vec4f>("e:color");
	auto smooth_costs = mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	bool valid;
	double mean_smooth_cost;
	double max_smooth_cost;
	std::tie(valid, mean_smooth_cost, max_smooth_cost) = getMeanAndMaxEdgeValue(mesh, "e:smooth_cost");

	if (valid && edge_colors.second && smooth_costs.second) {
		double reference_cost = mean_smooth_cost * 5.;
		for (auto & e : mesh.edges())
		{
			double error = (reference_cost > 0.) ? (smooth_costs.first[e] / reference_cost) : smooth_costs.first[e];
			error = std::min(1., error);
			edge_colors.first[e] = errorToRGB(error);
		}
	}
}



void setEdgeColorToRigidityCost(SurfaceMesh & mesh)
{
	auto edge_colors = mesh.property_map<edge_descriptor, ml::vec4f>("e:color");
	auto smooth_costs = mesh.property_map<edge_descriptor, double>("e:rigidity_cost");
	bool valid;
	double mean_smooth_cost;
	double max_smooth_cost;
	std::tie(valid, mean_smooth_cost, max_smooth_cost) = getMeanAndMaxEdgeValue(mesh, "e:rigidity_cost");

	if (valid && edge_colors.second && smooth_costs.second) {
		double reference_cost = max_smooth_cost;// mean_smooth_cost * 2.;
		for (auto & e : mesh.edges())
		{
			double error = (reference_cost > 0.) ? (smooth_costs.first[e] / reference_cost) : smooth_costs.first[e];
			error = std::min(1., error);
			edge_colors.first[e] = errorToRGB(error);
		}
	}
}


void setEdgeColorToRigidityValue(SurfaceMesh & mesh)
{
	auto edge_colors = mesh.property_map<edge_descriptor, ml::vec4f>("e:color");
	auto rigidity = mesh.property_map<edge_descriptor, double>("e:rigidity");
	bool valid;
	double mean, max;
	std::tie(valid, mean, max) = getMeanAndMaxEdgeValue(mesh, "e:rigidity");

	if (valid && edge_colors.second && rigidity.second) {
		double min = *std::min_element(rigidity.first.begin(), rigidity.first.end());
		std::cout << std::endl << "max rigidity: " << max << " mean rigidity " << mean << " min rigidity " << min;

		double reference_cost = mean * 2.;
		for (auto & e : mesh.edges())
		{
			double error = (max - rigidity.first[e]) / max;
			error = std::min(1., error);
			//double error = std::max(0., (10. - rigidity.first[e]));
			//error = error * 0.1;
			//error = pow(error, 3);
			//error = std::min(1., error);
			//double error = exp(-0.1 * rigidity.first[e]);
			edge_colors.first[e] = errorToRGB(error);
		}
	}
}

//-----------------------------------------------------------------------------

void setDeformationGraphColor(SurfaceMesh & mesh, VertexColor vertex_color, EdgeColor edge_color)
{
	setVertexColor(mesh, vertex_color);

	if(edge_color == EdgeColor::SmoothCost)
		setEdgeColorToSmoothCost(mesh);
	else if(edge_color == EdgeColor::RigidityCost)
		setEdgeColorToRigidityCost(mesh);
	else
		setEdgeColorToRigidityValue(mesh);	
}

}