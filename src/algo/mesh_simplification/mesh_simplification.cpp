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


#include <CGAL/Polygon_mesh_processing/refine.h>
#include <CGAL/Polygon_mesh_processing/fair.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>

Mesh createReducedMesh(const Mesh & mesh, int number_of_vertices)
{
	auto surface_mesh = convertToCGALMesh(mesh);
	surface_mesh = createReducedMesh(surface_mesh, number_of_vertices);
	return convertToTriMesh(surface_mesh);
}


struct halfedge2edge
{
	halfedge2edge(const SurfaceMesh& m, std::vector<edge_descriptor>& edges)
		: m_mesh(m), m_edges(edges)
	{}
	void operator()(const halfedge_descriptor& h) const
	{
		m_edges.push_back(edge(h, m_mesh));
	}
	const SurfaceMesh& m_mesh;
	std::vector<edge_descriptor>& m_edges;
};


SurfaceMesh createReducedMesh(const SurfaceMesh & mesh, int number_of_vertices)
{
	SurfaceMesh surface_mesh = mesh;

	//CGAL::Surface_mesh_simplification::Count_stop_predicate<SurfaceMesh> stop(number_of_vertices);
	//int r = CGAL::Surface_mesh_simplification::edge_collapse(surface_mesh, stop);


	double target_edge_length = 0.05;

	std::vector<edge_descriptor> border;
	CGAL::Polygon_mesh_processing::border_halfedges(faces(surface_mesh), surface_mesh, boost::make_function_output_iterator(halfedge2edge(surface_mesh, border)));
	CGAL::Polygon_mesh_processing::split_long_edges(border, target_edge_length, surface_mesh);

	//CGAL::Polygon_mesh_processing::split_long_edges();
	CGAL::Polygon_mesh_processing::isotropic_remeshing(faces(surface_mesh),
													   target_edge_length,
													   surface_mesh,
													   CGAL::Polygon_mesh_processing::parameters::number_of_iterations(10));// .protect_constraints(true));
	return surface_mesh;
}
