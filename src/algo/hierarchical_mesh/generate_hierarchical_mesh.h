#pragma once

#include "mesh/mesh_definition.h"
#include "algo/hierarchical_mesh/hierarchical_mesh.h"

class HierarchicalMeshLevelCreator
{
	unsigned int _level;
	double _radius;
public:
	SurfaceMesh create_mesh();
	Point add_vertex(const SurfaceMesh & original_mesh, vertex_descriptor v_original_mesh, SurfaceMesh & new_mesh);
public:
	HierarchicalMeshLevelCreator(unsigned int level, double radius)
		: _level(level)
		, _radius(radius)
	{}
};


SurfaceMesh generateHierarchicalMeshLevel(const SurfaceMesh & mesh, double radius, unsigned int level);

HierarchicalMesh generateHierarchicalMesh(const SurfaceMesh & mesh, double min_radius, unsigned int levels);

