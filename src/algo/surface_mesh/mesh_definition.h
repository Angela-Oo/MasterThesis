#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

//#include <CGAL/Search_traits_3.h>
//#include <CGAL/Search_traits_adapter.h>
//#include <CGAL/Orthogonal_k_neighbor_search.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Direction_3 Direction;
typedef Kernel::Aff_transformation_3 Matrix;
typedef CGAL::Surface_mesh<Kernel::Point_3> SurfaceMesh;

typedef SurfaceMesh::Vertex_index vertex_descriptor;
typedef SurfaceMesh::edge_index edge_descriptor;
typedef SurfaceMesh::halfedge_index halfedge_descriptor;
typedef SurfaceMesh::Face_index face_descriptor;
typedef SurfaceMesh::Point Point;
