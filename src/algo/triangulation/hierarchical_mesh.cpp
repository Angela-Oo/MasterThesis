#include "hierarchical_mesh.h"

void HierarchicalMesh::refineEdge(edge_descriptor edge)
{
	auto finer_level_v = _mesh.property_map<vertex_descriptor, vertex_descriptor>("v:finer_level_v").first;


}

HierarchicalMesh::HierarchicalMesh(const std::vector<SurfaceMesh> & meshes)
	: _meshes(meshes)
{
	assert(!_meshes.empty());
	_mesh = _meshes.back();
}

