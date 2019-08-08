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

std::vector<edge_descriptor> getEdgesToSplit(SurfaceMesh & mesh)
{
	auto edges = getEdgesToRefine(mesh);
	std::map<face_descriptor, std::vector<edge_descriptor>> fe;
	for (auto e : edges)
	{
		auto f0 = mesh.face(mesh.halfedge(e));
		auto f1 = mesh.face(mesh.opposite(mesh.halfedge(e)));

		fe[f0].push_back(e);
		fe[f1].push_back(e);		
	}

	for (auto f : fe)
	{
		if (f.second.size() == 2) {
			for (auto e : CGAL::edges_around_face(mesh.halfedge(f.first), mesh))
			{
				if (e != f.second[0] && e != f.second[1])
					edges.push_back(e);
			}
		}
	}
	return edges;
}


std::map<face_descriptor, std::vector<edge_descriptor>> getFacesToSplit(std::vector<edge_descriptor>& edges, SurfaceMesh & mesh)
{
	auto level_property_map = mesh.property_map<vertex_descriptor, int>("v:level");
	auto level = level_property_map.first;

	std::map<face_descriptor, std::vector<edge_descriptor>> faces;
	for (auto e : edges)
	{
		auto he = mesh.halfedge(e);
		auto ohe = mesh.opposite(he);

		auto v_he = mesh.target(mesh.next(he));
		auto v_ohe = mesh.target(mesh.next(ohe));

		if (level[v_he] < level[v_ohe]) {
			faces[mesh.face(he)].push_back(e);
		}
		else if (level[v_he] > level[v_ohe]) {
			faces[mesh.face(ohe)].push_back(e);
		}
		else {
			faces[mesh.face(he)].push_back(e);
			faces[mesh.face(ohe)].push_back(e);
		}
	}
	return faces;
}

void refineMeshBySplittingEdges(std::vector<edge_descriptor> refine_edges, SurfaceMesh & mesh)
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
			splitFaceAtEdge(f.first, f.second[0], f.second[1], mesh);
		}
		else if (f.second.size() == 3) {
			splitFaceAtEdge(f.first, f.second[0], f.second[1], f.second[2], mesh);
		}
	}
}

SurfaceMesh refineDeformationGraph(const SurfaceMesh & deformation_graph_mesh)
{
	SurfaceMesh refined_mesh = deformation_graph_mesh;

	std::vector<edge_descriptor> refine_edges = getEdgesToRefine(refined_mesh);
	refineMeshBySplittingEdges(refine_edges, refined_mesh);

	return refined_mesh;
}


}