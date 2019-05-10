#include "mesh_simplification.h"
#include "mLibCGAL.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
// Stop-condition policy
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>

Mesh createReducedMesh(const Mesh & mesh)
{
	auto surface_mesh = mesh;

	CGAL::Surface_mesh_simplification::Count_stop_predicate_vertices<Mesh> stop(1000);


	// This is a stop predicate (defines when the algorithm terminates).
	// In this example, the simplification stops when the number of undirected edges
	// left in the surface mesh drops below the specified number (1000)
	CGAL::Surface_mesh_simplification::Count_stop_predicate<Mesh> stop(1000);

	//CGAL::Surface_mesh_simplification::
	int r = CGAL::Surface_mesh_simplification::edge_collapse(surface_mesh, stop);
	return surface_mesh;	
}
