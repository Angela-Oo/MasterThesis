#pragma once

#include "deformation_graph.h"
#include "algo/surface_mesh/mesh_definition.h"
#include "algo/surface_mesh/mesh_operations.h"
#include <algorithm>

namespace Registration
{
std::vector<edge_descriptor> getEdgesToRefine(SurfaceMesh & refined_mesh);

// gets all edges to refine and also edges that need to be additional split as e.g. the third edge in a triangle
std::vector<edge_descriptor> getEdgesToSplit(SurfaceMesh & mesh);


std::map<face_descriptor, std::vector<edge_descriptor>> getFacesToSplit(std::vector<edge_descriptor>& edges, SurfaceMesh & mesh);

SurfaceMesh refineDeformationGraph(const SurfaceMesh & deformation_graph_mesh);






template <typename PositionDeformation>
halfedge_descriptor splitDeformationGraphEdge(edge_descriptor e, SurfaceMesh & mesh)
{
	auto deformation_property_map = mesh.property_map<vertex_descriptor, PositionDeformation>("v:node_deformation");
	auto deformation = deformation_property_map.first;

	auto he = mesh.halfedge(e);
	auto d0 = deformation[mesh.source(he)];
	auto d1 = deformation[mesh.target(he)];
	auto d = linearInterpolation(d0, d1);

	auto new_he = CGAL::Euler::split_edge(he, mesh);
	auto new_v = mesh.target(new_he);

	mesh.point(new_v) = d.position();
	deformation[new_v] = d;
	return new_he;
}

template <typename PositionDeformation>
std::map<face_descriptor, std::vector<vertex_descriptor>> splitDeformationGraphEdges(const std::vector<edge_descriptor> & refine_edges, SurfaceMesh & mesh)
{
	auto level_property_map = mesh.add_property_map<vertex_descriptor, int>("v:level", 0);
	auto level = level_property_map.first;

	std::map<face_descriptor, std::vector<vertex_descriptor>> fe;
	for (auto e : refine_edges)
	{
		auto new_he = splitDeformationGraphEdge<PositionDeformation>(e, mesh);
		auto new_v = mesh.target(new_he);
		level[new_v] = 1;

		for (auto f_around_v : CGAL::faces_around_target(new_he, mesh)) {
			fe[f_around_v].push_back(new_v);
		}
	}
	return fe;
}



template <typename PositionDeformation>
SurfaceMesh refineDeformationGraphMesh(SurfaceMesh mesh)
{
	std::vector<edge_descriptor> refine_edges = getEdgesToSplit(mesh);

	std::map<face_descriptor, std::vector<vertex_descriptor>> fe = splitDeformationGraphEdges<PositionDeformation>(refine_edges, mesh);
	splitFaces(fe, mesh);
	
	mesh.collect_garbage();
	return mesh;
}






template <typename PositionDeformation>
vertex_descriptor splitDeformationGraphFace(face_descriptor f, SurfaceMesh & mesh)
{
	auto deformation_property_map = mesh.property_map<vertex_descriptor, PositionDeformation>("v:node_deformation");
	auto deformation = deformation_property_map.first;

	auto he = mesh.halfedge(f);
	auto d0 = deformation[mesh.target(he)];
	auto d1 = deformation[mesh.target(mesh.next(he))];
	auto d2 = deformation[mesh.target(mesh.prev(he))];
	PositionDeformation d = mean(d0, d1, d2);

	auto new_v = splitFace(f, mesh);

	mesh.point(new_v) = d.position();
	deformation[new_v] = d;
	return new_v;
}


template <typename PositionDeformation>
std::vector<vertex_descriptor> splitDeformationGraphFaces(const std::map<face_descriptor, std::vector<edge_descriptor>> & refine_faces, SurfaceMesh & mesh)
{
	auto level_property_map = mesh.add_property_map<vertex_descriptor, int>("v:level", 0);
	auto level = level_property_map.first;

	std::vector<vertex_descriptor> v;
	for (auto f : refine_faces)
	{
		auto new_v = splitDeformationGraphFace<PositionDeformation>(f.first, mesh);
		level[new_v] = 1;
		v.push_back(new_v);
	}
	return v;
}


template <typename PositionDeformation>
SurfaceMesh refineDeformationGraphMeshTest(SurfaceMesh mesh)
{
	auto edges = getEdgesToRefine(mesh);
	auto refine_faces = getFacesToSplit(edges, mesh);
	std::vector<vertex_descriptor> new_vertices = splitDeformationGraphFaces<PositionDeformation>(refine_faces, mesh);
	flipEdges(edges, mesh);
	mesh.collect_garbage();
	return mesh;
}


template <typename PositionDeformation>
DeformationGraph<PositionDeformation> refineDeformationGraph(const DeformationGraph<PositionDeformation> & deformation_graph)
{
	//auto refined_mesh = refineDeformationGraphMesh<PositionDeformation>(deformation_graph._mesh);
	auto refined_mesh = refineDeformationGraphMeshTest<PositionDeformation>(deformation_graph._mesh);

	return DeformationGraph<PositionDeformation>(refined_mesh, deformation_graph._global);
}


}