#pragma once

#include "refine_deformation_graph.h"
#include <algorithm>
#include <CGAL/boost/graph/Euler_operations.h>

namespace Registration
{

std::vector<edge_descriptor> getEdgesToRefine(SurfaceMesh & refined_mesh)
{
	auto smooth_cost_property_map = refined_mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	assert(smooth_cost_property_map.second);
	auto smooth_cost = smooth_cost_property_map.first;

	double max_smooth_cost = *(std::max_element(smooth_cost.begin(), smooth_cost.end()));
	double refine_max_smooth_cost = std::max(max_smooth_cost * 0.5, 0.01);

	std::cout << "max smoot " << max_smooth_cost << " refine " << refine_max_smooth_cost;
	auto refine_property_map = refined_mesh.add_property_map<edge_descriptor, bool>("e:refine", false);
	auto refine = refine_property_map.first;

	std::vector<edge_descriptor> refine_edges;
	for (auto e : refined_mesh.edges())
	{
		if (smooth_cost[e] > refine_max_smooth_cost) {
			refine_edges.push_back(e);
			refine[e] = true;
		}
	}
	return refine_edges;
}

SurfaceMesh refineDeformationGraph(const SurfaceMesh & deformation_graph_mesh)
{
	SurfaceMesh refined_mesh = deformation_graph_mesh;

	std::vector<edge_descriptor> refine_edges = getEdgesToRefine(refined_mesh);

	auto refine_property_map = refined_mesh.property_map<edge_descriptor, bool>("e:refine");
	auto refine = refine_property_map.first;
	for (auto e : refined_mesh.edges())
	{
		//refined_mesh.add_vertex()
		//CGAL::Euler::add_center_vertex(refined_mesh.halfedge(e), refined_mesh);
	}

	return refined_mesh;
}


}