#pragma once

#include "algo/surface_mesh/mesh_definition.h"
#include "deformation_graph.h"
#include <algorithm>

namespace Registration
{

std::vector<edge_descriptor> getEdgesToRefine(SurfaceMesh & refined_mesh);

void splitEdge(edge_descriptor e, SurfaceMesh & mesh);

SurfaceMesh refineDeformationGraph(const SurfaceMesh & deformation_graph_mesh);


template <typename PositionDeformation>
DeformationGraph<PositionDeformation> refineDeformationGraph(const DeformationGraph<PositionDeformation> & deformation_graph)
{
	auto refined_mesh = refineDeformationGraph(deformation_graph._mesh);
	return DeformationGraph<PositionDeformation>(refined_mesh, deformation_graph._global);
}


}