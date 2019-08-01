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
	double refine_max_smooth_cost = std::max(max_smooth_cost * 0.9, 0.01);

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

void splitEdge(edge_descriptor e, SurfaceMesh & mesh)
{
	auto he = mesh.halfedge(e);

	Point source = mesh.point(mesh.source(he));
	Point target = mesh.point(mesh.target(he));
	Vector dir = target - source;
	Point center_point = source + (dir * 0.5);

	auto new_he_f0 = CGAL::Euler::split_edge(he, mesh);
	mesh.point(mesh.target(new_he_f0)) = center_point;
		
	auto split_f0_he0 = new_he_f0;
	auto split_f0_he1 = mesh.prev(mesh.prev(new_he_f0));

	auto new_he_f1 = mesh.opposite(new_he_f0);
	auto split_f1_he0 = mesh.prev(new_he_f1);
	auto split_f1_he1 = mesh.next(new_he_f1);

	CGAL::Euler::split_face(split_f0_he0, split_f0_he1, mesh);
	CGAL::Euler::split_face(split_f1_he0, split_f1_he1, mesh);
}

SurfaceMesh refineDeformationGraph(const SurfaceMesh & deformation_graph_mesh)
{
	SurfaceMesh refined_mesh = deformation_graph_mesh;

	std::vector<edge_descriptor> refine_edges = getEdgesToRefine(refined_mesh);

	auto refine_property_map = refined_mesh.property_map<edge_descriptor, bool>("e:refine");
	auto refine = refine_property_map.first;
	for (auto e : refine_edges)
	{
		splitEdge(e, refined_mesh);
	}

	return refined_mesh;
}


}