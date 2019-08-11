#include "hierarchical_mesh.h"


HierarchicalMesh::HierarchicalMesh(const std::vector<SurfaceMesh> & meshes)
	: _meshes(meshes)
{
	assert(!_meshes.empty());
	_mesh = _meshes[0];
}

