#pragma once
#include "rigid_deformation.h"
#include "algo/surface_mesh/mesh_definition.h"


class RigidDeformedMesh
{
private:
	const RigidDeformation & _deformation;
	SurfaceMesh _mesh;
public:
	SurfaceMesh deformPoints();
public:
	RigidDeformedMesh(const SurfaceMesh & mesh, const RigidDeformation & deformation_graph);
};

