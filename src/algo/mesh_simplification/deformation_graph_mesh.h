#pragma once

#include "algo/surface_mesh/mesh_definition.h"
#include "mLibInclude.h"
//#include "boost/graph/adjacency_list.hpp"
//#include <CGAL/Simple_cartesian.h>
//#include <CGAL/Surface_mesh.h>

#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>

typedef CGAL::Surface_mesh<Kernel::Point_3> DeformationGraphMesh;

//typedef boost::property_map<vertex_descriptor, std::shared_ptr<INode>>::type NodeProperty;

// search
//typedef boost::property_map<SurfaceMesh, CGAL::vertex_point_t>::type Vertex_point_pmap;
//typedef boost::graph_traits<SurfaceMesh>::vertices_size_type size_type;
//
//typedef CGAL::Search_traits_3<Kernel> TreeTraits;
//typedef CGAL::Search_traits_adapter<boost::graph_traits<SurfaceMesh>::vertex_descriptor, Vertex_point_pmap, TreeTraits> Traits;
//
//typedef CGAL::Orthogonal_k_neighbor_search<Traits> Neighbor_search;
//typedef Neighbor_search::Tree Tree;

typedef ml::TriMeshf Mesh;

DeformationGraphMesh convertToDeformationGraphMesh(const Mesh& triMesh);
//
//Mesh convertToTriMesh(DeformationGraphMesh& mesh);
