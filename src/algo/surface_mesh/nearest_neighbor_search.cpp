#include "nearest_neighbor_search.h"


/// NearestNeighborSearch search(mesh);
/// Neighbor_search s = search.neighbor_search(Point(0., 0., 0.), 5);
/// for (Neighbor_search::iterator it = s.begin(); it != s.end(); ++it) {
/// 	auto distance = std::sqrt(it->second);
/// 	auto vertex_handle = it->first;
/// }
Neighbor_search NearestNeighborSearch::search(Point point, int K)
{
	Neighbor_search::Distance distance(_vertex_point_property_map);
	Neighbor_search search(*_tree, point, K, 0, true, distance);
	return search;
}

NearestNeighborSearch::NearestNeighborSearch(const SurfaceMesh & mesh)
{
	_vertex_point_property_map = get(CGAL::vertex_point, mesh);
	_tree = std::make_unique<Tree>(vertices(mesh).begin(), vertices(mesh).end(), Tree::Splitter(), Traits(_vertex_point_property_map));
}

NearestNeighborSearch::NearestNeighborSearch(const SurfaceMesh & mesh, 
											 const std::vector<vertex_descriptor>::iterator vertices_begin,
											 const std::vector<vertex_descriptor>::iterator vertices_end)
{
	_vertex_point_property_map = get(CGAL::vertex_point, mesh);
	_tree = std::make_unique<Tree>(vertices_begin, vertices_end, Tree::Splitter(), Traits(_vertex_point_property_map));
}
