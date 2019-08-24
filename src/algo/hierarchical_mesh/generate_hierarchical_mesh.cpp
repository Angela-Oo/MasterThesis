#include "generate_hierarchical_mesh.h"
#include "surface_mesh_poisson_disk_sampling.h"
#include "triangulation.h"
#include "algo/remeshing/mesh_simplification.h"
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Polygon_mesh_processing/repair.h>


SurfaceMesh HierarchicalMeshLevelCreator::create_mesh()
{
	double radius = 1.5 * _radius; // radius used for calculating interpolation weights
	SurfaceMesh mesh;
	mesh.add_property_map<vertex_descriptor, Vector>("v:normal", Vector(0., 0., 0.)).first;
	mesh.add_property_map<vertex_descriptor, MeshLevel>("v:level", MeshLevel(_level)).first;
	mesh.add_property_map<vertex_descriptor, double>("v:radius", radius).first;
	return mesh;
}

Point HierarchicalMeshLevelCreator::add_vertex(const SurfaceMesh & original_mesh, vertex_descriptor v_original_mesh, SurfaceMesh & new_mesh)
{
	auto original_mesh_normals = original_mesh.property_map<vertex_descriptor, Vector>("v:normal").first;

	auto normals = new_mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
	auto level = new_mesh.property_map<vertex_descriptor, MeshLevel>("v:level").first;

	auto point = original_mesh.point(v_original_mesh);
	auto v = new_mesh.add_vertex(point);

	level[v].cluster_v = v;
	level[v].cluster_v_finer_level = v_original_mesh;
	normals[v] = original_mesh_normals[v_original_mesh];
	//cluster[v] = v;
	return point;
}


SurfaceMesh generateHierarchicalMeshLevel(const SurfaceMesh & mesh, double radius, unsigned int level)
{
	SurfaceMeshPoissonDiskSampling poisson_disk_sampling(mesh, radius);
	SurfaceMesh hierarchical_mesh = poisson_disk_sampling.create(HierarchicalMeshLevelCreator(level, radius));

	

	Construct construct(hierarchical_mesh);
	CGAL::advancing_front_surface_reconstruction(hierarchical_mesh.points().begin(),
												 hierarchical_mesh.points().end(),
												 construct);

	CGAL::Polygon_mesh_processing::remove_isolated_vertices(hierarchical_mesh);

	/*double d = 0;
	int i = 0;
	for (auto & e : hierarchical_mesh.edges())
	{
		auto source = hierarchical_mesh.point(hierarchical_mesh.source(e.halfedge()));
		auto target = hierarchical_mesh.point(hierarchical_mesh.target(e.halfedge()));
		auto distance = std::sqrt(CGAL::squared_distance(source, target));
		d += distance;
		++i;
	}
	if(i > 0)
		d /= i;

	auto radius_map = hierarchical_mesh.property_map<vertex_descriptor, double>("v:radius").first;
	for (auto & v : hierarchical_mesh.vertices())
	{
		radius_map[v] = d;
	}*/
	return hierarchical_mesh;
}

HierarchicalMesh generateHierarchicalMesh(const SurfaceMesh & mesh, double min_radius, unsigned int levels)
{
	std::vector<SurfaceMesh> meshes;

	auto reduced_mesh = createReducedMesh(mesh, min_radius);
	reduced_mesh.add_property_map<vertex_descriptor, MeshLevel>("v:level", MeshLevel(levels - 1));
	reduced_mesh.add_property_map<vertex_descriptor, double>("v:radius", min_radius);
	meshes.push_back(reduced_mesh);

	for (unsigned int i = 1; i < levels; ++i) {
		double radius = min_radius * pow(2, meshes.size());
		auto level = levels - i - 1;
		meshes.push_back(generateHierarchicalMeshLevel(meshes.back(), radius, level));
	}
	std::reverse(meshes.begin(), meshes.end());
	HierarchicalMesh hierarchical_mesh(meshes);
	return hierarchical_mesh;
}
