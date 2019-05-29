#pragma once

#include <stdafx.h>
#include "algo/surface_mesh/mesh_definition.h"


#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>

// search
typedef boost::property_map<SurfaceMesh, CGAL::vertex_point_t>::type Vertex_point_pmap;
typedef boost::graph_traits<SurfaceMesh>::vertices_size_type size_type;

typedef CGAL::Search_traits_3<Kernel> TreeTraits;
typedef CGAL::Search_traits_adapter<boost::graph_traits<SurfaceMesh>::vertex_descriptor, Vertex_point_pmap, TreeTraits> Traits;
typedef CGAL::Orthogonal_k_neighbor_search<Traits> Neighbor_search;
typedef Neighbor_search::Tree Tree;


/// NearestNeighborSearch search(mesh);
/// Neighbor_search s = search.neighbor_search(Point(0., 0., 0.), 5);
/// for (Neighbor_search::iterator it = s.begin(); it != s.end(); ++it) {
/// 	auto distance = std::sqrt(it->second);
/// 	auto vertex_handle = it->first;
/// }
class NearestNeighborSearch
{
	std::unique_ptr<Tree> _tree;
	Vertex_point_pmap _vertex_point_property_map;
public:
	Neighbor_search search(Point point, int K = 1);
	NearestNeighborSearch(const SurfaceMesh & mesh);
};