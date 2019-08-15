#pragma once

#include "deformation_graph.h"
#include "mesh/mesh_definition.h"
#include "algo/surface_mesh/mesh_operations.h"
#include "algo/registration/deformation_graph/deformed_mesh.h"
#include <algorithm>

namespace Registration
{
std::vector<edge_descriptor> getEdgesToRefine(SurfaceMesh & refined_mesh);

// gets all edges to refine and also edges that need to be additional split as e.g. the third edge in a triangle
std::vector<edge_descriptor> getEdgesToSplit(SurfaceMesh & mesh);


std::map<face_descriptor, std::vector<edge_descriptor>> getFacesToSplit(std::vector<edge_descriptor>& edges, SurfaceMesh & mesh);

SurfaceMesh refineDeformationGraph(const SurfaceMesh & deformation_graph_mesh);





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
	auto level_property_map = mesh.property_map<vertex_descriptor, int>("v:level");
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
	auto level_property_map = mesh.add_property_map<vertex_descriptor, int>("v:level", 0);
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
	//auto refined_mesh = refineDeformationGraphMeshTest<PositionDeformation>(deformation_graph._mesh);

	auto h_mesh = deformation_graph._hierarchical_mesh;
	h_mesh._mesh = deformation_graph._mesh;
	auto edges = getEdgesToRefine(h_mesh._mesh);

	std::vector<vertex_descriptor> new_vertices;
	for (auto & e : edges) {
		auto vs = h_mesh.refineEdge(e);
		new_vertices.insert(new_vertices.end(), vs.begin(), vs.end());
	}
	h_mesh.triangulate();

	auto & deformation_property_map = h_mesh._mesh.property_map<vertex_descriptor, PositionDeformation>("v:node_deformation").first; // TODO
	for (auto v : new_vertices) {
		int k = 4; // todo
		NearestNodes kNN = createNearestNodes<DeformationGraph<PositionDeformation>>(deformation_graph, h_mesh._mesh.point(v), k);

		std::vector<std::pair<PositionDeformation, double>> deformation_weights_vector;
		for (auto n_w : kNN.node_weight_vector)
		{
			auto node = deformation_graph.getDeformation(n_w.first);
			deformation_weights_vector.push_back(std::make_pair(node, n_w.second));
		}
		PositionDeformation deformation = interpolateDeformations(h_mesh._mesh.point(v), deformation_weights_vector);
		deformation_property_map[v] = deformation;
	}

	return DeformationGraph<PositionDeformation>(h_mesh, deformation_graph._global);
}


}