#include "nearest_neighbor_search.h"
//
//#include <CGAL/Search_traits_3.h>
//#include <CGAL/Search_traits_adapter.h>
//#include <CGAL/Orthogonal_k_neighbor_search.h>
//#include <CGAL/Surface_mesh.h>
//
//typedef CGAL::Search_traits_3<Kernel>                                    Traits_base;
//typedef CGAL::Search_traits_adapter<vertex_descriptor, NodeProperty, Traits_base> Traits;
//typedef CGAL::Orthogonal_k_neighbor_search<Traits>                      K_neighbor_search;
//typedef K_neighbor_search::Tree                                         Tree;
//typedef Tree::Splitter                                                  Splitter;
//typedef K_neighbor_search::Distance    Distance;

void test(const SurfaceMesh &mesh, Point query, int N) {

	//Tree tree(mesh.points().begin(), mesh.points().end());

	//// Initialize the search structure, and search all N points
	//Neighbor_search search(tree, query, N);
	//// report the N nearest neighbors and their distance
 //  // This should sort all N points by increasing distance from origin
	//for (Neighbor_search::iterator it = search.begin(); it != search.end(); ++it) {
	//	std::cout << it->first << " " << std::sqrt(it->second) << std::endl;
	//}
}
void FindNearestNeighbor::find()
{

}

FindNearestNeighbor::FindNearestNeighbor(const SurfaceMesh & mesh)
	: _mesh(mesh)
{
	
}
