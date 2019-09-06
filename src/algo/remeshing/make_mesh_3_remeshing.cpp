#include "make_mesh_3_remeshing.h"

#include "mLibCore.h"
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedral_mesh_domain_with_features_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/make_mesh_3.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel ExactInexactKernel;


SurfaceMesh makeMesh3Remeshing(const SurfaceMesh & mesh, double target_edge_length)
{
	// Polyhedron type
	typedef CGAL::Mesh_polyhedron_3<ExactInexactKernel>::type Polyhedron;
	typedef CGAL::Polyhedral_mesh_domain_with_features_3<ExactInexactKernel> MeshDomain;
	// Triangulation
	typedef CGAL::Mesh_triangulation_3<MeshDomain>::type Tr;
	typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr, MeshDomain::Corner_index, MeshDomain::Curve_index> C3t3;
	// Criteria
	typedef CGAL::Mesh_criteria_3<Tr> MeshCriteria;

	Polyhedron poly;
	CGAL::copy_face_graph(mesh, poly);
	// Create a vector with only one element: the pointer to the polyhedron.
	std::vector<Polyhedron*> poly_ptrs_vector(1, &poly);
	MeshDomain domain(poly_ptrs_vector.begin(), poly_ptrs_vector.end());

	MeshCriteria criteria(CGAL::parameters::edge_size = target_edge_length,
						  CGAL::parameters::facet_angle = 25,
						  CGAL::parameters::facet_size = 1.5,
						  CGAL::parameters::facet_distance = 0.01,
						  CGAL::parameters::cell_radius_edge_ratio = 2,
						  CGAL::parameters::cell_size = 1.5);

	// Mesh generation
	C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(domain, criteria, CGAL::parameters::no_perturb(), CGAL::parameters::no_exude());

	SurfaceMesh surface_mesh;
	CGAL::facets_in_complex_3_to_triangle_mesh(c3t3, surface_mesh);

	auto add_normals = surface_mesh.add_property_map<vertex_descriptor, Vector>("v:normal", Vector(0., 0., 0.)).first;
	CGAL::Polygon_mesh_processing::compute_vertex_normals(surface_mesh, add_normals);
	surface_mesh.add_property_map<vertex_descriptor, ml::vec4f>("v:color", ml::vec4f(1., 1., 1., 1.)).first;
	surface_mesh.add_property_map<edge_descriptor, ml::vec4f>("e:color", ml::vec4f(1., 1., 1., 1.)).first;

	std::cout << " count of surface mesh vertices " << surface_mesh.number_of_vertices() << std::endl;
	return surface_mesh;

	//// Mesh generation
	//auto refined_mesh = CGAL::make_mesh_3<SurfaceMesh>(surface_mesh, criteria, CGAL::parameters::no_perturb(), CGAL::parameters::no_exude());
}




SurfaceMesh makeMesh3RemeshingTest(const SurfaceMesh & mesh, double target_edge_length)
{
	// Polyhedron type
	typedef CGAL::Mesh_polyhedron_3<ExactInexactKernel>::type Polyhedron;
	typedef CGAL::Polyhedral_mesh_domain_with_features_3<ExactInexactKernel> MeshDomain;
	// Triangulation
	typedef CGAL::Mesh_triangulation_3<MeshDomain>::type Tr;
	typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr, MeshDomain::Corner_index, MeshDomain::Curve_index> C3t3;
	// Criteria
	typedef CGAL::Mesh_criteria_3<Tr> MeshCriteria;

	Polyhedron poly;
	CGAL::copy_face_graph(mesh, poly);
	// Create a vector with only one element: the pointer to the polyhedron.
	std::vector<Polyhedron*> poly_ptrs_vector(1, &poly);
	MeshDomain domain(poly_ptrs_vector.begin(), poly_ptrs_vector.end());

	MeshCriteria criteria(CGAL::parameters::edge_size = 0.1,
						  CGAL::parameters::facet_angle = 20,
						  CGAL::parameters::facet_size = 0.7,
						  CGAL::parameters::facet_distance = 0.01,
						  CGAL::parameters::cell_radius_edge_ratio = 2,
						  CGAL::parameters::cell_size = 1.5);

	// Mesh generation
	C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(domain, criteria, CGAL::parameters::no_perturb(), CGAL::parameters::no_exude());

	SurfaceMesh surface_mesh;
	CGAL::facets_in_complex_3_to_triangle_mesh(c3t3, surface_mesh);

	auto add_normals = surface_mesh.add_property_map<vertex_descriptor, Vector>("v:normal", Vector(0., 0., 0.)).first;
	CGAL::Polygon_mesh_processing::compute_vertex_normals(surface_mesh, add_normals);
	surface_mesh.add_property_map<vertex_descriptor, ml::vec4f>("v:color", ml::vec4f(1., 1., 1., 1.)).first;
	surface_mesh.add_property_map<edge_descriptor, ml::vec4f>("e:color", ml::vec4f(1., 1., 1., 1.)).first;

	std::cout << " count of surface mesh vertices " << surface_mesh.number_of_vertices() << std::endl;
	return surface_mesh;

	//// Mesh generation
	//auto refined_mesh = CGAL::make_mesh_3<SurfaceMesh>(surface_mesh, criteria, CGAL::parameters::no_perturb(), CGAL::parameters::no_exude());
}

