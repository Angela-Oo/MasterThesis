#pragma once

#include "mLibInclude.h"
#include "deformation_graph.h"

namespace DG
{

double getMeanFitCost(const SurfaceMesh & mesh);

void setVertexColorBasedOnFitCost(SurfaceMesh & mesh, double reference_cost);

SurfaceMesh deformationGraphToSurfaceMesh(const DeformationGraph & deformation_graph, bool color_based_on_cost);

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