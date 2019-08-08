#include "triangulation.h"
#include "algo/surface_mesh/nearest_neighbor_search.h"

SurfaceMesh triangulate(SurfaceMesh mesh, double radius)
{
	RadiusNearestNeighborSearch search(mesh);

	for (auto v : mesh.vertices()) {
		auto vertices = search.search(mesh.point(v), radius);
		for (auto n : vertices)
		{
			if (n.first != v) {
				mesh.add_edge(v, n.first);
			}
		}			
	}

	return mesh;
}