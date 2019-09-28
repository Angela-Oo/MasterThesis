#include "chamfer_distance.h"
#include "algo/nearest_neighbor_search/nearest_neighbor_search.h"
#include <numeric>

namespace Registration {

double ChamferDistance::sumDistances()
{
	double sum_source = std::accumulate(source_vertices_distances.second.begin(),
										source_vertices_distances.second.end(),
										0.);
	double sum_target = std::accumulate(target_vertices_distances.second.begin(),
										target_vertices_distances.second.end(),
										0.);
	return sum_source + sum_target;
}

std::vector<double> ChamferDistance::distances() const
{
	std::vector<double> distances;
	distances.insert(distances.end(), source_vertices_distances.second.begin(), source_vertices_distances.second.end());
	distances.insert(distances.end(), target_vertices_distances.second.begin(), target_vertices_distances.second.end());
	return distances;
}

double ChamferDistance::meanDistances()
{
	double sum = sumDistances();
	size_t number_points = source_vertices_distances.second.size() + target_vertices_distances.second.size();
	return sum / number_points;
}


	
ChamferDistance::VerticesDistances calculateSquaredDistances(const SurfaceMesh& source, const NearestNeighborSearch & nn_search_target)
{
	std::vector<vertex_descriptor> v_ids;
	std::vector<double> distances;
	for (auto vertex : source.vertices()) {
		const auto point = source.point(vertex);
		auto s = nn_search_target.search(point);
		for (Neighbor_search::iterator it = s.begin(); it != s.end(); ++it) {
			auto distance = it->second;
			v_ids.push_back(vertex);
			distances.push_back(distance);
		}
	}
	return std::make_pair(v_ids, distances);
}


ChamferDistance chamferDistance(const SurfaceMesh & source, const SurfaceMesh & target)
{
	const auto nn_search_source = std::make_unique<NearestNeighborSearch>(source);
	const auto nn_search_target = std::make_unique<NearestNeighborSearch>(target);

	ChamferDistance distances;
	distances.source_vertices_distances = calculateSquaredDistances(source, *nn_search_target.get());
	distances.target_vertices_distances = calculateSquaredDistances(target, *nn_search_source.get());
	return distances;	
}


	

void CalculateChamferDistance::setSource(const SurfaceMesh & source)
{
	_source = &source;
	_nn_search_source = std::make_unique<NearestNeighborSearch>(*_source);
}
	
void CalculateChamferDistance::setTarget(const SurfaceMesh & target)
{
	_target = &target;
	_nn_search_target = std::make_unique<NearestNeighborSearch>(*_target);
}
	
ChamferDistance CalculateChamferDistance::calculateChamferDistance() const
{
	ChamferDistance distances;
	distances.source_vertices_distances = calculateSquaredDistances(*_source, *_nn_search_target.get());
	distances.target_vertices_distances = calculateSquaredDistances(*_target, *_nn_search_source.get());
	return distances;
}
	
CalculateChamferDistance::CalculateChamferDistance(const SurfaceMesh & source, const SurfaceMesh & target)
	: _source(&source)
	, _target(&target)
{
	_nn_search_source = std::make_unique<NearestNeighborSearch>(*_source);
	_nn_search_target = std::make_unique<NearestNeighborSearch>(*_target);
}
	

}
