#pragma once

#include "mesh/mesh_definition.h"
#include "algo/triangulation/hierarchical_mesh.h"

SurfaceMesh generateHierarchicalMeshLevel(const SurfaceMesh & mesh, double radius);

HierarchicalMesh generateHierarchicalMesh(const SurfaceMesh & mesh, double min_radius, unsigned int levels);

