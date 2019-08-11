#include "generate_hierarchical_mesh.h"
#include "surface_mesh_poisson_disk_sampling.h"
#include "triangulation.h"
#include "algo/remeshing/mesh_simplification.h"
#include <CGAL/Advancing_front_surface_reconstruction.h>

Point add_vertex(const SurfaceMesh & original_mesh, vertex_descriptor v_original_mesh, SurfaceMesh & new_mesh)
{
	auto original_mesh_normals = original_mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
	auto original_mesh_level = original_mesh.property_map<vertex_descriptor, unsigned int>("v:level").first;

	auto normals = new_mesh.add_property_map<vertex_descriptor, Vector>("v:normal", Vector(0., 0., 0.)).first;
	auto finer_level_v = new_mesh.add_property_map<vertex_descriptor, vertex_descriptor>("v:finer_level_v", vertex_descriptor()).first;
	auto level = new_mesh.add_property_map<vertex_descriptor, unsigned int>("v:level", 0).first;
	auto finer_level_point = new_mesh.add_property_map<vertex_descriptor, Point>("v:finer_level_point", Point(0., 0., 0.)).first;

	auto point = original_mesh.point(v_original_mesh);
	auto v = new_mesh.add_vertex(point);

	finer_level_v[v] = v_original_mesh;
	normals[v] = original_mesh_normals[v_original_mesh];
	level[v] = original_mesh_level[v_original_mesh] - 1;
	finer_level_point[v] = original_mesh.point(original_mesh.source(original_mesh.halfedge(v_original_mesh)));

	return point;
}

SurfaceMesh generateHierarchicalMeshLevel(const SurfaceMesh & mesh, double radius)
{
	SurfaceMeshPoissonDiskSampling poisson_disk_sampling(mesh, radius);
	SurfaceMesh hierarchical_mesh = poisson_disk_sampling.create(add_vertex);

	Construct construct(hierarchical_mesh);
	CGAL::advancing_front_surface_reconstruction(hierarchical_mesh.points().begin(),
												 hierarchical_mesh.points().end(),
												 construct);

	hierarchical_mesh.add_property_map<edge_descriptor, ml::vec4f>("e:color", ml::vec4f(1., 1., 1., 1.));
	hierarchical_mesh.add_property_map<vertex_descriptor, ml::vec4f>("v:color", ml::vec4f(1., 1., 1., 1.));
	return hierarchical_mesh;
}

HierarchicalMesh generateHierarchicalMesh(const SurfaceMesh & mesh, double min_radius, unsigned int levels)
{
	std::vector<SurfaceMesh> meshes;

	auto reduced_mesh = createReducedMesh(mesh, min_radius);
	reduced_mesh.add_property_map<vertex_descriptor, unsigned int>("v:level", levels - 1);
	meshes.push_back(reduced_mesh);

	for (auto i = 1; i < levels; ++i) {
		double radius = min_radius * pow(2, meshes.size());
		meshes.push_back(generateHierarchicalMeshLevel(meshes.back(), radius));
	}
	std::reverse(meshes.begin(), meshes.end());
	HierarchicalMesh hierarchical_mesh(meshes);
	return hierarchical_mesh;
}
