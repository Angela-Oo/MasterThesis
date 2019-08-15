#include "generate_hierarchical_mesh.h"
#include "surface_mesh_poisson_disk_sampling.h"
#include "triangulation.h"
#include "algo/remeshing/mesh_simplification.h"
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Polygon_mesh_processing/repair.h>

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
	hierarchical_mesh.add_property_map<vertex_descriptor, bool>("v:refined", false);
	CGAL::Polygon_mesh_processing::remove_isolated_vertices(hierarchical_mesh);
	return hierarchical_mesh;
}

HierarchicalMesh generateHierarchicalMesh(const SurfaceMesh & mesh, double min_radius, unsigned int levels)
{
	std::vector<SurfaceMesh> meshes;

	auto reduced_mesh = createReducedMesh(mesh, min_radius);
	reduced_mesh.add_property_map<vertex_descriptor, unsigned int>("v:level", levels - 1);
	meshes.push_back(reduced_mesh);

	for (unsigned int i = 1; i < levels; ++i) {
		double radius = min_radius * pow(2, meshes.size());
		meshes.push_back(generateHierarchicalMeshLevel(meshes.back(), radius));
	}
	std::reverse(meshes.begin(), meshes.end());
	HierarchicalMesh hierarchical_mesh(meshes);
	return hierarchical_mesh;
}
