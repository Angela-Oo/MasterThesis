#pragma once

#include "mesh/mesh_definition.h"

std::map<vertex_descriptor, std::vector<vertex_descriptor>> cluster(const SurfaceMesh & mesh, const SurfaceMesh & child_mesh);


struct MeshLevel
{
	unsigned int level;
	vertex_descriptor cluster_v;
	vertex_descriptor cluster_v_finer_level;
	MeshLevel()
		: level(0)
	{}
	MeshLevel(unsigned int l)
		: level(l)
	{}
};

MeshLevel getMeshLevel(const SurfaceMesh & mesh, const vertex_descriptor & v);


class HierarchicalMesh
{
private:
	std::vector<SurfaceMesh> _meshes;
	std::vector<std::map<vertex_descriptor, std::vector<vertex_descriptor>>> _vertex_cluster_map;
public:
	SurfaceMesh getInitMesh();
	bool validLevel(unsigned int level) const;
	size_t size() const;
	const SurfaceMesh & getMesh(unsigned int level) const;
	const std::map<vertex_descriptor, std::vector<vertex_descriptor>> & getClusters(unsigned int level) const;
	const std::vector<vertex_descriptor> & getCluster(const MeshLevel & v) const;
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
	std::vector<vertex_descriptor> refine(std::vector<vertex_descriptor> vertices, SurfaceMesh & mesh);
	std::vector<vertex_descriptor> refine(std::vector<edge_descriptor> edges, SurfaceMesh & mesh);	
public:
	HierarchicalMeshRefinement(const HierarchicalMesh & hierarchical_mesh);
};
