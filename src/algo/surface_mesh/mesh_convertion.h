#pragma once

#include "mLibInclude.h"
#include "mesh/mesh_definition.h"

typedef ml::TriMeshf Mesh;

SurfaceMesh convertToCGALMesh(const Mesh& triMesh, bool calculate_normals = false);

Mesh convertToTriMesh(const SurfaceMesh& mesh);