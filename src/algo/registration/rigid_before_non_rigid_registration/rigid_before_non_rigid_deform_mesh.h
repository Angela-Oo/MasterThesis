#pragma once

#include "rigid_before_non_rigid_deformation.h"
#include "mesh/mesh_definition.h"
#include "algo/registration/deformation_graph/deformation_graph_deform_mesh.h"
#include "algo/registration/rigid_registration/rigid_deformed_mesh.h"

namespace Registration
{

template <typename NonRigidDeformation, typename DeformMesh>
class RigidBeforeNonRigidDeformMesh
{
private:
	const RigidBeforeNonRigidDeformation<NonRigidDeformation> & _deformation;
	unsigned int _k; // number of interpolated deformation graph nodes per vertex
public:
	SurfaceMesh deformationGraphMesh();
	SurfaceMesh deformedMesh(const SurfaceMesh & mesh);
public:
	RigidBeforeNonRigidDeformMesh(const RigidBeforeNonRigidDeformation<NonRigidDeformation> & deformation);
};

template <typename NonRigidDeformation, typename DeformMesh>
SurfaceMesh RigidBeforeNonRigidDeformMesh<NonRigidDeformation, DeformMesh>::deformationGraphMesh()
{
	if (_deformation.is_rigid_deformation) {
		RigidDeformedMesh deform_mesh(_deformation.rigid_deformation);
		return deform_mesh.deformationGraphMesh();
	}
	else {
		DeformMesh deform_mesh(_deformation.non_rigid_deformation);
		return deform_mesh.deformationGraphMesh();
	}
}

template <typename NonRigidDeformation, typename DeformMesh>
SurfaceMesh RigidBeforeNonRigidDeformMesh<NonRigidDeformation, DeformMesh>::deformedMesh(const SurfaceMesh & mesh)
{
	if (_deformation.is_rigid_deformation) {
		RigidDeformedMesh deform_mesh(_deformation.rigid_deformation);
		return deform_mesh.deformedMesh(mesh);
	}
	else {
		DeformMesh deform_mesh(_deformation.non_rigid_deformation);
		return deform_mesh.deformedMesh(mesh);
	}
}

template <typename NonRigidDeformation, typename DeformMesh>
RigidBeforeNonRigidDeformMesh<NonRigidDeformation, DeformMesh>::RigidBeforeNonRigidDeformMesh(const RigidBeforeNonRigidDeformation<NonRigidDeformation> & deformation)
	: _deformation(deformation)
{}


}
