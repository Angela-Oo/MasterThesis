#include "generate_hierarchical_mesh.h"
#include "surface_mesh_poisson_disk_sampling.h"
#include "triangulation.h"
#include "algo/remeshing/mesh_simplification.h"
#include <CGAL/Advancing_front_surface_reconstruction.h>


SurfaceMesh generateHierarchicalMeshLevel(const SurfaceMesh & mesh, double radius)
{
	SurfaceMeshPoissonDiskSampling poisson_disk_sampling(mesh, radius);
	SurfaceMesh hierarchical_mesh = poisson_disk_sampling.create();

	Construct construct(hierarchical_mesh);
	CGAL::advancing_front_surface_reconstruction(hierarchical_mesh.points().begin(),
												 hierarchical_mesh.points().end(),
												 construct);
	return hierarchical_mesh;
}

HierarchicalMesh generateHierarchicalMesh(const SurfaceMesh & mesh, double min_radius, unsigned int levels)
{
	std::vector<SurfaceMesh> meshes;

	auto reduced_mesh = createReducedMesh(mesh, min_radius);
	meshes.push_back(reduced_mesh);

	for (auto i = 1; i < levels; ++i) {
		double radius = min_radius * pow(2, meshes.size());
		meshes.push_back(generateHierarchicalMeshLevel(meshes.back(), radius));
	}

	HierarchicalMesh hierarchical_mesh(meshes);
	return hierarchical_mesh;
}
