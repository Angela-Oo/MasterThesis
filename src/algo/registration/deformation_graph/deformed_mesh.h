#pragma once

#include "mLibInclude.h"
#include "deformation_graph.h"

namespace DG
{

SurfaceMesh deformationGraphToSurfaceMesh(const DeformationGraph & deformation_graph, bool color_based_on_cost, bool smooth_cost = true, bool fit_cost = false);

class DeformedMesh
{
private:
	const DeformationGraph & _deformation_graph;
	SurfaceMesh _mesh;
public:
	SurfaceMesh deformPoints();
public:
	DeformedMesh(const SurfaceMesh & mesh, const DeformationGraph & deformation_graph);
};


}