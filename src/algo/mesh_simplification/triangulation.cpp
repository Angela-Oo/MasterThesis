#include "triangulation.h"
#include "algo/surface_mesh/nearest_neighbor_search.h"

#include <CGAL/Advancing_front_surface_reconstruction.h>

typedef CGAL::cpp11::array<std::size_t, 3> Facet;

struct Construct {
	SurfaceMesh& mesh;
	template < typename PointIterator>
	Construct(SurfaceMesh& mesh, PointIterator b, PointIterator e)
		: mesh(mesh)
	{
		for (; b != e; ++b) {
			boost::graph_traits<SurfaceMesh>::vertex_descriptor v;
			v = add_vertex(mesh);
			mesh.point(v) = *b;
		}
	}
	Construct& operator=(const Facet f)
	{
		typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor vertex_descriptor;
		typedef boost::graph_traits<SurfaceMesh>::vertices_size_type size_type;
		mesh.add_face(vertex_descriptor(static_cast<size_type>(f[0])),
					  vertex_descriptor(static_cast<size_type>(f[1])),
					  vertex_descriptor(static_cast<size_type>(f[2])));
		return *this;
	}
	Construct&
		operator*() { return *this; }
	Construct&
		operator++() { return *this; }
	Construct
		operator++(int) { return *this; }
};



SurfaceMesh simple_triangulate(SurfaceMesh mesh)
{
	SurfaceMesh triangulation;
	Construct construct(triangulation, mesh.points().begin(), mesh.points().end());
	CGAL::advancing_front_surface_reconstruction(mesh.points().begin(),
												 mesh.points().end(),
												 construct);
	return triangulation;
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
	return simple_triangulate(mesh);
}