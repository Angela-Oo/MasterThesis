#include "poison_surface_remeshing.h"
#include "mLibCore.h"
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel ExactInexactKernel;

SurfaceMesh poisonSurfaceRemeshing(const SurfaceMesh & mesh, double target_edge_length)
{
	bool normals_exists = mesh.property_map<vertex_descriptor, Vector>("v:normal").second;
	assert(normals_exists);
	auto normals = mesh.property_map<vertex_descriptor, Vector>("v:normal").first;

	typedef CGAL::Exact_predicates_inexact_constructions_kernel PoisonRemeshKernel;
	typedef PoisonRemeshKernel::Point_3 RemeshPoint;
	typedef PoisonRemeshKernel::Vector_3 RemeshVector;
	typedef std::pair<RemeshPoint, RemeshVector> PointWithNormal;
	std::vector<PointWithNormal> points;
	for (auto & v : mesh.vertices())
	{
		auto p = mesh.point(v);
		auto n = normals[v];
		points.emplace_back(std::make_pair(RemeshPoint(p.x(), p.y(), p.z()), RemeshVector(n.x(), n.y(), n.z())));
	}

	CGAL::Polyhedron_3<PoisonRemeshKernel> remeshed_surface;
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


	std::cout << "success " << success << std::endl;

	SurfaceMesh surface_mesh;
	CGAL::copy_face_graph(remeshed_surface, surface_mesh);
	auto add_normals = surface_mesh.add_property_map<vertex_descriptor, Vector>("v:normal", Vector(0., 0., 0.)).first;
	CGAL::Polygon_mesh_processing::compute_vertex_normals(surface_mesh, add_normals);
	surface_mesh.add_property_map<vertex_descriptor, ml::vec4f>("v:color", ml::vec4f(1., 1., 1., 1.)).first;
	surface_mesh.add_property_map<edge_descriptor, ml::vec4f>("e:color", ml::vec4f(1., 1., 1., 1.)).first;

	std::cout << " count of surface mesh vertices " << surface_mesh.number_of_vertices() << std::endl;
	return surface_mesh;
}



