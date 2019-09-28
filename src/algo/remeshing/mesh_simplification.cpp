#include "mesh_simplification.h"
#include "poison_surface_remeshing.h"
#include "make_mesh_3_remeshing.h"
#include "isotropic_remeshing.h"
#include "algo/surface_mesh/mesh_convertion.h"
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include "algo/nearest_neighbor_search/nearest_neighbor_search.h"

Mesh createReducedMesh(const Mesh & mesh, double target_edge_length, ReduceMeshStrategy strategy)
{
	auto surface_mesh = convertToCGALMesh(mesh);
	surface_mesh = createReducedMesh(surface_mesh, target_edge_length, strategy);
	return convertToTriMesh(surface_mesh);
}


double getMaximalPointDistance(const SurfaceMesh & mesh)
{
	FurthestNeighborSearch search(mesh);
	double max_distance = 0.;
	for (auto vertex : mesh.vertices()) {
		Neighbor_search s = search.search(mesh.point(vertex), 1);
		for (Neighbor_search::iterator it = s.begin(); it != s.end(); ++it) {
			auto distance = std::sqrt(it->second);
			if (distance > max_distance)
				max_distance = distance;
		}
	}
	return max_distance;
}

double getMeshArea(const SurfaceMesh & mesh)
{
	return CGAL::Polygon_mesh_processing::area(mesh);
}

double deformationGraphEdgeLength(const SurfaceMesh & mesh, double edge_length_percentage_of_surface_area)
{
	double max_area = getMeshArea(mesh);
	double edge_length = std::sqrt(max_area) * edge_length_percentage_of_surface_area;
	std::cout << " max area " << max_area << " used edge length " << edge_length << std::endl;
	return edge_length;
}

SurfaceMesh createReducedMesh(const SurfaceMesh & mesh, double edge_length, ReduceMeshStrategy strategy)
{
	SurfaceMesh reduced_mesh;
	if(strategy == ReduceMeshStrategy::ISOTROPIC)
		reduced_mesh = isotropicRemeshing(mesh, edge_length);
	else if (strategy == ReduceMeshStrategy::POISON)
		reduced_mesh = poisonSurfaceRemeshing(mesh, edge_length);
	else if (strategy == ReduceMeshStrategy::MAKEMESH3)
		reduced_mesh = makeMesh3Remeshing(mesh, edge_length);
	else // if (strategy == ReduceMeshStrategy::NONE)
		reduced_mesh = mesh;
	
	CGAL::Polygon_mesh_processing::remove_isolated_vertices(reduced_mesh);
	return reduced_mesh;
}
