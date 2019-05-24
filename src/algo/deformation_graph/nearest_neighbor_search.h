#pragma once

#include <stdafx.h>
#include "algo/mesh_simplification/deformation_graph_mesh.h"

class FindNearestNeighbor
{
	const SurfaceMesh & _mesh;
public:
	void find();
	FindNearestNeighbor(const SurfaceMesh & mesh);
};