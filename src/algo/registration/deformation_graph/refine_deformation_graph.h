#pragma once

#include "deformation_graph.h"

namespace Registration
{

SurfaceMesh refineDeformationGraph(const SurfaceMesh & deformation_graph_mesh)
{
	//auto smooth_cost_property_map = deformation_graph_mesh.property_map<edge_descriptor, double>("e:smooth_cost", 0.);
	//assert(smooth_cost_property_map.second);
	//auto smooth_cost = smooth_cost_property_map.first;

	//double max = std::max(smooth_cost.begin(), smooth_cost.end());
	//for (auto e : deformation_graph_mesh.edges())
	//{
	//	smooth_cost[e];
	//}
	return deformation_graph_mesh;
}


template <typename PositionDeformation>
DeformationGraph<PositionDeformation> refineDeformationGraph(const DeformationGraph<PositionDeformation> & deformation_graph)
{
	refineDeformationGraph(deformation_graph._mesh);
	return deformation_graph;
}


}