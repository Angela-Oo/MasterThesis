#pragma once

#include "mesh/mesh_definition.h"
#include "algo/registration/deformation_graph/deformation_graph.h"

namespace Visualize {

void setDeformationGraphColor(SurfaceMesh & mesh, bool use_only_smooth_cost);

}