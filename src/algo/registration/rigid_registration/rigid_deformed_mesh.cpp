#include "rigid_deformed_mesh.h"

namespace Registration {

SurfaceMesh RigidDeformedMesh::deformPoints(const SurfaceMesh & mesh)
{
	SurfaceMesh deformed_points = mesh;
	for (auto & v : mesh.vertices()) {
		deformed_points.point(v) = _deformation.deformPoint(mesh.point(v));
	}
	return deformed_points;
}

SurfaceMesh RigidDeformedMesh::deformationGraphMesh()
{
	return SurfaceMesh();
}

SurfaceMesh RigidDeformedMesh::deformedMesh(const SurfaceMesh & mesh)
{
	return deformPoints(mesh);
}

RigidDeformedMesh::RigidDeformedMesh(const RigidDeformation & deformation)
	: _deformation(deformation)
{}

}