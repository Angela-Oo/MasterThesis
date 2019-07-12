#pragma once

#include "algo/surface_mesh/mesh_definition.h"

SurfaceMesh isotropicRemeshing(const SurfaceMesh & mesh, double target_edge_length);