#include "stdafx.h"
#include "rigid_deformed_mesh.h"

SurfaceMesh RigidDeformedMesh::deformPoints()
{
	SurfaceMesh deformed_points = _mesh;
	for (auto & v : _mesh.vertices()) {
		deformed_points.point(v) = _deformation.deformPoint(_mesh.point(v));
	}
	return deformed_points;
}


RigidDeformedMesh::RigidDeformedMesh(const SurfaceMesh & mesh, const RigidDeformation & deformation)
	: _mesh(mesh)
	, _deformation(deformation)
{

}

