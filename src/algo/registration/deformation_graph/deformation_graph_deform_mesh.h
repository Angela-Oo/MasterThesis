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
	unsigned int _k; // number of interpolated deformation graph nodes per vertex
public:
	SurfaceMesh deformationGraphMesh();
	SurfaceMesh deformedMesh(const SurfaceMesh & mesh);
public:
	DeformationGraphDeformMesh(const DeformationGraph & deformation_graph, unsigned int number_of_interpolation_neighbors);
};

template <typename DeformationGraph>
SurfaceMesh DeformationGraphDeformMesh<DeformationGraph>::deformationGraphMesh()
{
	return deformationGraphToSurfaceMesh(_deformation, true);
}

template <typename DeformationGraph>
SurfaceMesh DeformationGraphDeformMesh<DeformationGraph>::deformedMesh(const SurfaceMesh & mesh)
{
	DeformedMesh<DeformationGraph> deformed(mesh, _deformation, _k);
	return deformed.deformPoints();
}

template <typename DeformationGraph>
DeformationGraphDeformMesh<DeformationGraph>::DeformationGraphDeformMesh(const DeformationGraph & deformation, unsigned int number_of_interpolation_neighbors)
	: _deformation(deformation)
	, _k(number_of_interpolation_neighbors)
{}


}