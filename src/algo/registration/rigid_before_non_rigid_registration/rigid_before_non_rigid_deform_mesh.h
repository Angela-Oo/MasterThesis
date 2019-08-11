#pragma once

#include "rigid_before_non_rigid_deformation.h"
#include "mesh/mesh_definition.h"
#include "algo/registration/deformation_graph/deformation_graph_deform_mesh.h"
#include "algo/registration/rigid_registration/rigid_deformed_mesh.h"

namespace Registration
{

template <typename NonRigidDeformation>
class RigidBeforeNonRigidDeformMesh
{
private:
	const RigidBeforeNonRigidDeformation<NonRigidDeformation> & _deformation;
	unsigned int _k; // number of interpolated deformation graph nodes per vertex
public:
	SurfaceMesh deformationGraphMesh();
	SurfaceMesh deformedMesh(const SurfaceMesh & mesh);
public:
	RigidBeforeNonRigidDeformMesh(const RigidBeforeNonRigidDeformation<NonRigidDeformation> & deformation, unsigned int number_of_interpolation_neighbors = 4);
};

template <typename NonRigidDeformation>
SurfaceMesh RigidBeforeNonRigidDeformMesh<NonRigidDeformation>::deformationGraphMesh()
{
	if (_deformation.is_rigid_deformation) {
		RigidDeformedMesh deform_mesh(_deformation.rigid_deformation);
		return deform_mesh.deformationGraphMesh();
	}
	else {
		DeformationGraphDeformMesh<NonRigidDeformation> deform_mesh(_deformation.non_rigid_deformation, _k);
		return deform_mesh.deformationGraphMesh();
	}
}

template <typename NonRigidDeformation>
SurfaceMesh RigidBeforeNonRigidDeformMesh<NonRigidDeformation>::deformedMesh(const SurfaceMesh & mesh)
{
	if (_deformation.is_rigid_deformation) {
		RigidDeformedMesh deform_mesh(_deformation.rigid_deformation);
		return deform_mesh.deformedMesh(mesh);
	}
	else {
		DeformationGraphDeformMesh<NonRigidDeformation> deform_mesh(_deformation.non_rigid_deformation, _k);
		return deform_mesh.deformedMesh(mesh);
	}
}

template <typename NonRigidDeformation>
RigidBeforeNonRigidDeformMesh<NonRigidDeformation>::RigidBeforeNonRigidDeformMesh(const RigidBeforeNonRigidDeformation<NonRigidDeformation> & deformation, unsigned int number_of_interpolation_neighbors)
	: _deformation(deformation)
	, _k(number_of_interpolation_neighbors)
{}


}
