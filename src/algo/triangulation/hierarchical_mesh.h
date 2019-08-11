#pragma once

#include "mesh/mesh_definition.h"
#include "algo/nearest_neighbor_search/nearest_neighbor_search.h"

std::map<vertex_descriptor, std::vector<vertex_descriptor>> cluster(const SurfaceMesh & mesh, const SurfaceMesh & child_mesh);

Point add_vertex(const SurfaceMesh & original_mesh, vertex_descriptor v_original_mesh, SurfaceMesh & new_mesh);

class HierarchicalMesh
{
public:
	SurfaceMesh _mesh;
	std::vector<SurfaceMesh> _meshes;
	std::vector<std::map<vertex_descriptor, std::vector<vertex_descriptor>>> _vertex_cluster_map;
public:
	void refineVertex(vertex_descriptor vertex);
	void refineEdge(edge_descriptor edge);
	void triangulate();
public:
	HierarchicalMesh() = default;
	HierarchicalMesh(const std::vector<SurfaceMesh> & meshes);

};

