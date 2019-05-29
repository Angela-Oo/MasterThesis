#include "stdafx.h"

#include "mesh_simplification.h"
#include "algo/surface_mesh/mesh_convertion.h"
//
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>

//// Stop-condition policy
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>


Mesh createReducedMesh(const Mesh & mesh, int number_of_vertices)
{
	auto surface_mesh = convertToCGALMesh(mesh);

	//CGAL::Surface_mesh_simplification::Count_stop_predicate_vertices<SurfaceMesh> stop(number_of_vertices);

	// This is a stop predicate (defines when the algorithm terminates).
	// In this example, the simplification stops when the number of undirected edges
	// left in the surface mesh drops below the specified number (1000)
	CGAL::Surface_mesh_simplification::Count_stop_predicate<SurfaceMesh> stop(number_of_vertices);

	int r = CGAL::Surface_mesh_simplification::edge_collapse(surface_mesh, stop);
	return convertToTriMesh(surface_mesh);
}


SurfaceMesh createReducedMesh(const SurfaceMesh & mesh, int number_of_vertices)
{
	SurfaceMesh surface_mesh = mesh;
	CGAL::Surface_mesh_simplification::Count_stop_predicate<SurfaceMesh> stop(number_of_vertices);
	int r = CGAL::Surface_mesh_simplification::edge_collapse(surface_mesh, stop);
	return surface_mesh;
}
