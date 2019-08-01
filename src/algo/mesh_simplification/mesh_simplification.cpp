#include "mesh_simplification.h"
#include "poison_surface_remeshing.h"
#include "make_mesh_3_remeshing.h"
#include "isotropic_remeshing.h"
#include "algo/surface_mesh/mesh_convertion.h"
#include <CGAL/Polygon_mesh_processing/repair.h>

Mesh createReducedMesh(const Mesh & mesh, double target_edge_length, ReduceMeshStrategy strategy)
{
	auto surface_mesh = convertToCGALMesh(mesh);
	surface_mesh = createReducedMesh(surface_mesh, target_edge_length, strategy);
	return convertToTriMesh(surface_mesh);
}



SurfaceMesh createReducedMesh(const SurfaceMesh & mesh, double target_edge_length, ReduceMeshStrategy strategy)
{
	SurfaceMesh reduced_mesh;
	if(strategy == ReduceMeshStrategy::ISOTROPIC)
		reduced_mesh = isotropicRemeshing(mesh, target_edge_length);
	else if (strategy == ReduceMeshStrategy::POISON)
		reduced_mesh = poisonSurfaceRemeshing(mesh, target_edge_length);
	else if (strategy == ReduceMeshStrategy::MAKEMESH3)
		reduced_mesh = makeMesh3Remeshing(mesh, target_edge_length);
	else // if (strategy == ReduceMeshStrategy::NONE)
		reduced_mesh = mesh;
	
	CGAL::Polygon_mesh_processing::remove_isolated_vertices(reduced_mesh);
	return reduced_mesh;
}
