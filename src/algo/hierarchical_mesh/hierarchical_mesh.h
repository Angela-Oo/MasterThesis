#pragma once

#include "mesh/mesh_definition.h"
#include "algo/nearest_neighbor_search/nearest_neighbor_search.h"

std::map<vertex_descriptor, std::vector<vertex_descriptor>> cluster(const SurfaceMesh & mesh, const SurfaceMesh & child_mesh);

Point add_vertex(const SurfaceMesh & original_mesh, vertex_descriptor v_original_mesh, SurfaceMesh & new_mesh);

class HierarchicalMesh
{
public:
	std::vector<SurfaceMesh> _meshes;
	std::vector<std::map<vertex_descriptor, std::vector<vertex_descriptor>>> _vertex_cluster_map;
public:
	SurfaceMesh getInitMesh();
public:
	HierarchicalMesh() = default;
	HierarchicalMesh(const std::vector<SurfaceMesh> & meshes);
	HierarchicalMesh(const HierarchicalMesh & deformation_graph);
	HierarchicalMesh & operator=(HierarchicalMesh other);
};


class HierarchicalMeshRefinement
{
	const HierarchicalMesh & _hierarchical_mesh;
private:
	std::vector<vertex_descriptor> refineVertex(vertex_descriptor vertex, SurfaceMesh & mesh);
	std::vector<vertex_descriptor> refineEdge(edge_descriptor edge, SurfaceMesh & mesh);
	void triangulate(SurfaceMesh & mesh);
public:
	std::vector<vertex_descriptor> refine(std::vector<edge_descriptor> edges, SurfaceMesh & mesh);
public:
	HierarchicalMeshRefinement(const HierarchicalMesh & hierarchical_mesh);
};
