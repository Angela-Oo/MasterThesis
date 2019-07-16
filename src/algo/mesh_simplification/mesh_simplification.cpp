#include "stdafx.h"

#include "mesh_simplification.h"
#include "algo/surface_mesh/mesh_convertion.h"

#include "poison_surface_remeshing.h"
#include "make_mesh_3_remeshing.h"
#include "isotropic_remeshing.h"


Mesh createReducedMesh(const Mesh & mesh, double target_edge_length)
{
	auto surface_mesh = convertToCGALMesh(mesh);
	surface_mesh = createReducedMesh(surface_mesh, target_edge_length);
	return convertToTriMesh(surface_mesh);
}

enum class ReduceMeshStrategy{
	NONE,
	ISOTROPIC,
	POISON,
	MAKEMESH3
};

SurfaceMesh createReducedMesh(const SurfaceMesh & mesh, double target_edge_length)
{
	ReduceMeshStrategy strategy = ReduceMeshStrategy::ISOTROPIC;
	if(strategy == ReduceMeshStrategy::ISOTROPIC)
		return isotropicRemeshing(mesh, target_edge_length);
	else if (strategy == ReduceMeshStrategy::POISON)
		return poisonSurfaceRemeshing(mesh, target_edge_length);
	else if (strategy == ReduceMeshStrategy::MAKEMESH3)
		return makeMesh3Remeshing(mesh, target_edge_length);
	else if (strategy == ReduceMeshStrategy::NONE)
		return mesh;
	return mesh;
}
