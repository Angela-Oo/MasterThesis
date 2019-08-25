#pragma once

#include "deformation_graph.h"
#include "deformed_mesh.h"

namespace Registration
{

template <typename DeformationGraph>
class DeformationGraphDeformMesh
{
private:
	const DeformationGraph & _deformation;
public:
	SurfaceMesh deformationGraphMesh();
	SurfaceMesh deformedMesh(const SurfaceMesh & mesh);
public:
	DeformationGraphDeformMesh(const DeformationGraph & deformation_graph); // todo
};

template <typename DeformationGraph>
SurfaceMesh DeformationGraphDeformMesh<DeformationGraph>::deformationGraphMesh()
{
	return deformationGraphToSurfaceMesh(_deformation, true);
}

template <typename DeformationGraph>
SurfaceMesh DeformationGraphDeformMesh<DeformationGraph>::deformedMesh(const SurfaceMesh & mesh)
{
	DeformedMesh<DeformationGraph> deformed(mesh, _deformation);
	return deformed.deformPoints();
}

template <typename DeformationGraph>
DeformationGraphDeformMesh<DeformationGraph>::DeformationGraphDeformMesh(const DeformationGraph & deformation)
	: _deformation(deformation)
{}


}