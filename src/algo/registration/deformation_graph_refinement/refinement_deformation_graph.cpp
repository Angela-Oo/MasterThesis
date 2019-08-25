#pragma once

#include "refinement_deformation_graph.h"

namespace Registration {


std::vector<edge_descriptor> getEdgesToRefine(SurfaceMesh & refined_mesh, double min_smooth_cost, double percentage_max_smooth_cost)
{
	auto smooth_cost_property_map = refined_mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	assert(smooth_cost_property_map.second);
	auto smooth_cost = smooth_cost_property_map.first;

	double max_smooth_cost = *(std::max_element(smooth_cost.begin(), smooth_cost.end()));
	double refine_max_smooth_cost = std::max(max_smooth_cost * percentage_max_smooth_cost, min_smooth_cost);

	std::vector<edge_descriptor> refine_edges;
	for (auto e : refined_mesh.edges())
	{
		if (smooth_cost[e] > refine_max_smooth_cost) {
			refine_edges.push_back(e);
		}
	}

	std::cout << std::endl << "max smooth " << max_smooth_cost << " refine " << refine_max_smooth_cost << " number of edges " << refine_edges.size() << std::endl;
	return refine_edges;
}


std::vector<vertex_descriptor> getVerticesToRefine(SurfaceMesh & refined_mesh, double min_smooth_cost, double percentage_max_smooth_cost)
{
	auto smooth_cost_property_map = refined_mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	assert(smooth_cost_property_map.second);
	auto smooth_cost = smooth_cost_property_map.first;


	std::map<vertex_descriptor, double> vertex_smooth_cost;
	double max_smooth_cost = 0.;
	for (auto v : refined_mesh.vertices()) 
	{
		double v_smooth_cost = 0.;
		unsigned int i = 0;
		for (auto he : refined_mesh.halfedges_around_target(refined_mesh.halfedge(v)))
		{
			edge_descriptor e = refined_mesh.edge(he);
			v_smooth_cost += smooth_cost[e];
			++i;
		}
		if (i > 0) {
			v_smooth_cost = v_smooth_cost / i;
			vertex_smooth_cost[v] = v_smooth_cost;
			if (v_smooth_cost > max_smooth_cost)
				max_smooth_cost = v_smooth_cost;
		}
	}

	double refine_max_smooth_cost = std::max(max_smooth_cost * percentage_max_smooth_cost, min_smooth_cost);
	
	std::vector<vertex_descriptor> refine_vertices;
	for (auto v : vertex_smooth_cost) 
	{		
		if (v.second > refine_max_smooth_cost) {
			refine_vertices.push_back(v.first);
		}		
	}

	std::cout << std::endl << "max smooth " << max_smooth_cost << " refine " << refine_max_smooth_cost << " number of vertices " << refine_vertices.size() << std::endl;
	return refine_vertices;
}


}