#pragma once

#include "refine_deformation_graph.h"
#include "algo/surface_mesh/mesh_operations.h"
#include <algorithm>
#include <CGAL/boost/graph/Euler_operations.h>
#include <optional>

namespace Registration
{

std::vector<edge_descriptor> getEdgesToRefine(SurfaceMesh & refined_mesh)
{
	auto smooth_cost_property_map = refined_mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	assert(smooth_cost_property_map.second);
	auto smooth_cost = smooth_cost_property_map.first;

	double max_smooth_cost = *(std::max_element(smooth_cost.begin(), smooth_cost.end()));
	double refine_max_smooth_cost = std::max(max_smooth_cost * 0.6, 0.01);

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


void refineMesh(std::vector<edge_descriptor> refine_edges, SurfaceMesh & mesh)
{
	//auto refine_property_map = mesh.property_map<edge_descriptor, bool>("e:refine");

	auto level_property_map = mesh.add_property_map<vertex_descriptor, int>("v:level", 0);	
	auto level = level_property_map.first;

	std::map<face_descriptor, std::vector<vertex_descriptor>> fe;
	for (auto e : refine_edges)
	{
		auto new_he = splitEdgeAtCenter(e, mesh);
		auto new_v = mesh.target(new_he);
		level[new_v] = 1;

		for (auto f_around_v : CGAL::faces_around_target(new_he, mesh)) {
			fe[f_around_v].push_back(new_v);
		}
	}
	for (auto f : fe) {
		if (f.second.size() == 2) {
			splitFace(f.first, f.second[0], f.second[1], mesh);
		}
		else if (f.second.size() == 3) {
			splitFace(f.first, f.second[0], f.second[1], f.second[2], mesh);
		}
	}
}

SurfaceMesh refineDeformationGraph(const SurfaceMesh & deformation_graph_mesh)
{
	SurfaceMesh refined_mesh = deformation_graph_mesh;

	std::vector<edge_descriptor> refine_edges = getEdgesToRefine(refined_mesh);
	refineMesh(refine_edges, refined_mesh);

	return refined_mesh;
}


}