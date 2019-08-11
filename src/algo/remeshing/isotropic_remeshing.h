#pragma once

#include "mesh/mesh_definition.h"

SurfaceMesh isotropicRemeshing(const SurfaceMesh & mesh, double target_edge_length);