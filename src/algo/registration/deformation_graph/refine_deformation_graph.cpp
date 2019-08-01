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

halfedge_descriptor splitEdgeAtCenter(edge_descriptor e, SurfaceMesh & mesh)
{
	auto he = mesh.halfedge(e);
	Point source = mesh.point(mesh.source(he));
	Point target = mesh.point(mesh.target(he));
	Vector dir = target - source;
	Point center_point = source + (dir * 0.5);

	auto new_he = CGAL::Euler::split_edge(he, mesh);
	mesh.point(mesh.target(new_he)) = center_point;
	return new_he;
}

void splitEdge(edge_descriptor e, SurfaceMesh & mesh)
{
	auto new_he_f0 = splitEdgeAtCenter(e, mesh);
		
	auto split_f0_he0 = new_he_f0;
	auto split_f0_he1 = mesh.prev(mesh.prev(new_he_f0));

	auto new_he_f1 = mesh.opposite(new_he_f0);
	auto split_f1_he0 = mesh.prev(new_he_f1);
	auto split_f1_he1 = mesh.next(new_he_f1);

	CGAL::Euler::split_face(split_f0_he0, split_f0_he1, mesh);
	CGAL::Euler::split_face(split_f1_he0, split_f1_he1, mesh);
}



void splitFace(face_descriptor f, SurfaceMesh & mesh)
{
	auto he_0 = mesh.halfedge(f);
	auto he_1 = mesh.next(he_0);
	auto he_2 = mesh.next(he_1);
	
	auto new_he_0 = splitEdgeAtCenter(mesh.edge(he_0), mesh);
	auto new_he_1 = splitEdgeAtCenter(mesh.edge(he_1), mesh);
	auto new_he_2 = splitEdgeAtCenter(mesh.edge(he_2), mesh);

	CGAL::Euler::split_face(new_he_0, new_he_1, mesh);
	CGAL::Euler::split_face(new_he_1, new_he_2, mesh);
	CGAL::Euler::split_face(new_he_2, new_he_0, mesh);

	//if (!mesh.is_border(mesh.edge(he_0))) {
	//	auto ohe_0 = mesh.opposite(new_he_0);
	//	auto split_0_he0 = mesh.next(ohe_0);
	//	auto split_0_he1 = mesh.prev(ohe_0);
	//	CGAL::Euler::split_face(split_0_he0, split_0_he1, mesh);
	//}
	//if (!mesh.is_border(mesh.edge(he_1))) {
	//	auto ohe_1 = mesh.opposite(new_he_1);
	//	auto split_1_he0 = mesh.next(ohe_1);
	//	auto split_1_he1 = mesh.prev(ohe_1);
	//	CGAL::Euler::split_face(split_1_he0, split_1_he1, mesh);
	//}
	//if (!mesh.is_border(mesh.edge(he_2))) {
	//	auto ohe_2 = mesh.opposite(new_he_2);
	//	auto split_2_he0 = mesh.next(ohe_2);
	//	auto split_2_he1 = mesh.prev(ohe_2);
	//	CGAL::Euler::split_face(split_2_he0, split_2_he1, mesh);
	//}
}

SurfaceMesh refineDeformationGraph(const SurfaceMesh & deformation_graph_mesh)
{
	SurfaceMesh refined_mesh = deformation_graph_mesh;

	std::vector<edge_descriptor> refine_edges = getEdgesToRefine(refined_mesh);

	auto refine_property_map = refined_mesh.property_map<edge_descriptor, bool>("e:refine");
	auto refine = refine_property_map.first;
	std::map<face_descriptor, std::vector<halfedge_descriptor>> fe;
	for (auto e : refine_edges)
	{
		auto he = refined_mesh.halfedge(e);
		auto ohe = refined_mesh.opposite(he);
		auto f0 = refined_mesh.face(he);
		auto f1 = refined_mesh.face(ohe);
		fe[f0].push_back(he);
		fe[f1].push_back(ohe);
		//splitEdge(e, refined_mesh);
	}
	for (auto f : fe) {
		//if (f.second.size() >= 2) {
		splitFace(f.first, refined_mesh);
		//}
		//else {
		//	splitEdge(f.second[0], refined_mesh);
		//}
	}

	return refined_mesh;
}


}