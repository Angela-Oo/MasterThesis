#pragma once
#include "mLibInclude.h"
#include "boost/graph/adjacency_list.hpp"


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


Mesh createReducedMesh(const Mesh & mesh);