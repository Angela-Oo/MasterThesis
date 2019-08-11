#include "triangulation.h"
#include "algo/surface_mesh/nearest_neighbor_search.h"

#include <CGAL/Advancing_front_surface_reconstruction.h>



void surfaceMeshFrontTriangulation(SurfaceMesh & mesh)
{
	Construct construct(mesh);
	CGAL::advancing_front_surface_reconstruction(mesh.points().begin(),
												 mesh.points().end(),
												 construct);
}


SurfaceMesh triangulateKNN(SurfaceMesh mesh, double radius)
{
	RadiusNearestNeighborSearch search(mesh);

	for (auto v : mesh.vertices()) {
		Radius_search s = search.search(mesh.point(v));
		int i = 0;
		for (Radius_search::iterator it = s.begin(); it != s.end(); ++it) {
			if (it->first != v) {
				mesh.add_edge(it->first, v);
				++i;
				if (i > 4)
					break;
			}
		}
		//auto vertices = search.search(mesh.point(v), radius);
		//for (auto n : vertices)
		//{
		//	if (n.first != v) {
		//		mesh.add_edge(v, n.first);
		//	}
		//}			
	}

	return mesh;
}



SurfaceMesh triangulate(SurfaceMesh mesh, double radius)
{
	surfaceMeshFrontTriangulation(mesh);
	return mesh;
}