#pragma once

#include "mesh/mesh_definition.h"
#include "algo/registration/deformation_graph/deformation_graph.h"

namespace Visualize {

enum class VertexColor
{
	Default,
	FitCost
};

enum class EdgeColor
{
	SmoothCost,
	RigidityCost,
	RigidityValue
};

void setDeformationGraphColor(SurfaceMesh & mesh, VertexColor vertex_color = VertexColor::Default, EdgeColor edge_color = EdgeColor::SmoothCost);

}