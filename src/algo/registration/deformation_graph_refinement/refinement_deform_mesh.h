#pragma once

#include "refinement_deformation.h"
#include "mesh/mesh_definition.h"

namespace Registration
{

template <typename Deformation, typename DeformMesh>
class RefinementDeformMesh
{
private:
	const RefineDeformationGraphDeformation<Deformation> & _deformation;
public:
	SurfaceMesh deformationGraphMesh();
	SurfaceMesh deformedMesh(const SurfaceMesh & mesh);
public:
	RefinementDeformMesh(const RefineDeformationGraphDeformation<Deformation> & deformation);
};

template <typename Deformation, typename DeformMesh>
SurfaceMesh RefinementDeformMesh<Deformation, DeformMesh>::deformationGraphMesh()
{
	DeformMesh deform_mesh(_deformation.non_rigid_deformation);
	return deform_mesh.deformationGraphMesh();
}

template <typename Deformation, typename DeformMesh>
SurfaceMesh RefinementDeformMesh<Deformation, DeformMesh>::deformedMesh(const SurfaceMesh & mesh)
{
	DeformMesh deform_mesh(_deformation.non_rigid_deformation);
	return deform_mesh.deformedMesh(mesh);
}

template <typename Deformation, typename DeformMesh>
RefinementDeformMesh<Deformation, DeformMesh>::RefinementDeformMesh(const RefineDeformationGraphDeformation<Deformation> & deformation)
	: _deformation(deformation)
{}


}
