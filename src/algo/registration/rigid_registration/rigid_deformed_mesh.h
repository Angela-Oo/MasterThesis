#pragma once
#include "rigid_deformation.h"
#include "algo/surface_mesh/mesh_definition.h"

namespace Registration {

class RigidDeformedMesh
{
private:
	const RigidDeformation & _deformation;
	SurfaceMesh _mesh;
public:
	SurfaceMesh deformPoints(const SurfaceMesh & mesh);
	SurfaceMesh deformationGraphMesh();
	SurfaceMesh deformedMesh(const SurfaceMesh & mesh);
public:
	RigidDeformedMesh(const RigidDeformation & deformation_graph);
};

}