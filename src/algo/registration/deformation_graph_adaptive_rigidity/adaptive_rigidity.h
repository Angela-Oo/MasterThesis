#pragma once

#include "algo/registration/deformation_graph_refinement/refinement_deformation_graph.h"
#include "mesh/mesh_definition.h"

namespace Registration {

size_t adaptRigidity(SurfaceMesh & deformation, double rigidity_smooth_cost_threshold, double minimal_rigidity);

}

