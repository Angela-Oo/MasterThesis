#pragma once

#include "algo/surface_mesh/mesh_definition.h"

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
	Construct(SurfaceMesh& mesh)
		: mesh(mesh)
	{ }
	Construct& operator=(const Facet f)
	{
		typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor vertex_descriptor;
		typedef boost::graph_traits<SurfaceMesh>::vertices_size_type size_type;
		mesh.add_face(vertex_descriptor(static_cast<size_type>(f[0])),
					  vertex_descriptor(static_cast<size_type>(f[1])),
					  vertex_descriptor(static_cast<size_type>(f[2])));
		return *this;
	}
	Construct& operator*() { return *this; }
	Construct& operator++() { return *this; }
	Construct  operator++(int) { return *this; }
};


void surfaceMeshFrontTriangulation(SurfaceMesh & mesh);

SurfaceMesh triangulate(SurfaceMesh mesh, double radius);