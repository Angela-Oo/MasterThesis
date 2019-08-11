#pragma once

#include "mesh/mesh_definition.h"
#include "algo/nearest_neighbor_search/nearest_neighbor_search.h"

class HierarchicalMesh
{
public:
	SurfaceMesh _mesh;
	std::vector<SurfaceMesh> _meshes;
public:
	HierarchicalMesh() = default;
	HierarchicalMesh(const std::vector<SurfaceMesh> & meshes);

};

