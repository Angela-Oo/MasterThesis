#pragma once
#include "mLibInclude.h"
#include "boost/graph/adjacency_list.hpp"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> SurfaceMesh;
typedef SurfaceMesh::Vertex_index vertex_descriptor;
typedef SurfaceMesh::Edge_index edge_descriptor;
typedef SurfaceMesh::Face_index face_descriptor;

namespace CGAL
{
namespace Surface_mesh_simplification
{

// 
// Stops when the number of vertices left falls below a given number.
//
template<class TM_>
class Count_stop_predicate_vertices
{
public:
	typedef TM_ TM;
	typedef typename boost::graph_traits<TM>::vertices_size_type size_type;
public:
	Count_stop_predicate_vertices(std::size_t aThres) : mThres(aThres) {}

	template <typename F, typename Profile>
	bool operator()(F const&         // aCurrentCost
					, Profile const& // aEdgeProfile
					, std::size_t    // aInitialCount
					, std::size_t       aCurrentCount
					) const
	{
		return aCurrentCount < mThres;
	}
private:
	std::size_t mThres;
};
}
}


typedef ml::TriMeshf Mesh;

Mesh convertToTriMesh(SurfaceMesh& mesh);

Mesh createReducedMesh(const Mesh & mesh, int number_of_vertices);