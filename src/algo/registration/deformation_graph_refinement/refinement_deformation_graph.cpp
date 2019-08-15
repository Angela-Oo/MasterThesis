#pragma once

#include "refinement_deformation_graph.h"

namespace Registration {


std::vector<edge_descriptor> getEdgesToRefine(SurfaceMesh & refined_mesh)
{
	auto smooth_cost_property_map = refined_mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	assert(smooth_cost_property_map.second);
	auto smooth_cost = smooth_cost_property_map.first;

	double max_smooth_cost = *(std::max_element(smooth_cost.begin(), smooth_cost.end()));
	double refine_max_smooth_cost = std::max(max_smooth_cost * 0.9, 0.01);	

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


}