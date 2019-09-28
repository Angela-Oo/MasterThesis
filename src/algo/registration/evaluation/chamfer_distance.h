#pragma once

#include "mesh/mesh_definition.h"
#include "registration_error.h"
#include "algo/nearest_neighbor_search/nearest_neighbor_search.h"

namespace Registration {

	
class ChamferDistance
{
public:
	using VerticesDistances = std::pair<std::vector<vertex_descriptor>, std::vector<double>>;

	VerticesDistances source_vertices_distances;
	VerticesDistances target_vertices_distances;
	double sumDistances();
	std::vector<double> distances() const;
	double meanDistances();
};

ChamferDistance chamferDistance(const SurfaceMesh & source, const SurfaceMesh & target);



class CalculateChamferDistance
{
	const SurfaceMesh * _source;
	const SurfaceMesh * _target;
	std::unique_ptr<NearestNeighborSearch> _nn_search_source;
	std::unique_ptr<NearestNeighborSearch> _nn_search_target;
public:
	void setSource(const SurfaceMesh & source);
	void setTarget(const SurfaceMesh & target);
	ChamferDistance calculateChamferDistance() const;
	CalculateChamferDistance(const SurfaceMesh & source, const SurfaceMesh & target);
};
	
}
