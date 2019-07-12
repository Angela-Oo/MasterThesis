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


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/boost/graph/graph_traits_PolyMesh_ArrayKernelT.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/IO/read_xyz_points.h>
//#include <CGAL/Mesh_criteria_3.h>
//#include <CGAL/make_mesh_3.h>

Mesh createReducedMesh(const Mesh & mesh, double target_edge_length)
{
	auto surface_mesh = convertToCGALMesh(mesh);
	surface_mesh = createReducedMesh(surface_mesh, target_edge_length);
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





SurfaceMesh createReducedMeshPoisonSurfaceReconstruction(const SurfaceMesh & mesh, double target_edge_length)
{
	assert(surface_mesh.property_map<vertex_descriptor, Vector>("v:normal").second);
	auto normals = mesh.property_map<vertex_descriptor, Vector>("v:normal").first;

	typedef CGAL::Exact_predicates_inexact_constructions_kernel RemeshKernel;
	//typedef CGAL::Simple_cartesian<double> RemeshKernel;
	typedef RemeshKernel::Point_3 RemeshPoint;
	typedef RemeshKernel::Vector_3 RemeshVector;
	typedef std::pair<RemeshPoint, RemeshVector> PointWithNormal;
	std::vector<PointWithNormal> points;
	for (auto & v : mesh.vertices())
	{
		auto p = mesh.point(v);
		auto n = normals[v];
		points.emplace_back(std::make_pair(RemeshPoint(p.x(), p.y(), p.z()), RemeshVector(n.x(), n.y(), n.z())));
	}

	CGAL::Polyhedron_3<RemeshKernel> remeshed_surface;
	double average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>
		(points, 6, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointWithNormal>()));

	double 	sm_angle = 20.0;
	double 	sm_radius = 30;
	double 	sm_distance = 0.375;
	bool success = CGAL::poisson_surface_reconstruction_delaunay(points.begin(), points.end(),
																	CGAL::First_of_pair_property_map<PointWithNormal>(),
																	CGAL::Second_of_pair_property_map<PointWithNormal>(),
																	remeshed_surface,
																	average_spacing,
																	sm_angle,
																	sm_radius,
																	sm_distance);

	//bool success = CGAL::poisson_surface_reconstruction_delaunay(surface_mesh.vertices().begin(), surface_mesh.vertices().end(),
	//															 surface_mesh.points().begin(),
	//															 normals.begin(),
	//															 remeshed_surface, target_edge_length);

	std::cout << "success " << success << std::endl;
	SurfaceMesh surface_mesh;
	CGAL::copy_face_graph(remeshed_surface, surface_mesh);
	auto add_normals = surface_mesh.add_property_map<vertex_descriptor, Vector>("v:normal", Vector(0., 0., 0.)).first;
	CGAL::Polygon_mesh_processing::compute_vertex_normals(surface_mesh, add_normals);
	surface_mesh.add_property_map<vertex_descriptor, ml::vec4f>("v:color", ml::vec4f(1., 1., 1., 1.)).first;
	surface_mesh.add_property_map<edge_descriptor, ml::vec4f>("e:color", ml::vec4f(1., 1., 1., 1.)).first;

	std::cout << "count of surface mesh vertices " << surface_mesh.number_of_vertices() << std::endl;
	return surface_mesh;
}


SurfaceMesh createReducedMeshRemeshing(const SurfaceMesh & mesh, double target_edge_length)
{	

	// Mesh criteria

	//CGAL::Mesh_criteria_3<SurfaceMesh> criteria(0.025, //edge_size
	//											25,//facet_angle = 
	//											0.1,//facet_size
	//											0.001);//facet_distance

	//// Mesh generation
	//auto refined_mesh = CGAL::make_mesh_3<SurfaceMesh>(surface_mesh, criteria, CGAL::parameters::no_perturb(), CGAL::parameters::no_exude());

	return mesh;
}


SurfaceMesh createReducedMeshIsotropicRemeshing(const SurfaceMesh & mesh, double target_edge_length)
{
	SurfaceMesh surface_mesh = mesh;

	std::vector<edge_descriptor> border;
	CGAL::Polygon_mesh_processing::border_halfedges(faces(surface_mesh), surface_mesh, boost::make_function_output_iterator(halfedge2edge(surface_mesh, border)));
	CGAL::Polygon_mesh_processing::split_long_edges(border, target_edge_length, surface_mesh);

	CGAL::Polygon_mesh_processing::isotropic_remeshing(faces(surface_mesh),
													   target_edge_length,
													   surface_mesh,
													   CGAL::Polygon_mesh_processing::parameters::number_of_iterations(10).relax_constraints(true));// .protect_constraints(true));

	auto normals = surface_mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
	CGAL::Polygon_mesh_processing::compute_vertex_normals(surface_mesh, normals);

	std::cout << "count of surface mesh vertices " << surface_mesh.number_of_vertices() << std::endl;
	return surface_mesh;
}



SurfaceMesh createReducedMesh(const SurfaceMesh & mesh, double target_edge_length)
{
	SurfaceMesh surface_mesh = mesh;
	try {
		//return createReducedMeshIsotropicRemeshing(mesh, target_edge_length);
		return createReducedMeshPoisonSurfaceReconstruction(mesh, target_edge_length);
		//return createReducedMeshRemeshing(mesh, target_edge_length);
	}
	catch (...) {
		return mesh;
	}
	return surface_mesh;
}
