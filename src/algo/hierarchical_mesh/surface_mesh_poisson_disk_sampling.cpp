#include "surface_mesh_poisson_disk_sampling.h"
#include "triangulation.h"

#include <CGAL/Advancing_front_surface_reconstruction.h>

vertex_descriptor SurfaceMeshPoissonDiskSampling::nextVertex()
{
	if (_candidates.empty()) {
		return *_unsupported_vertices.begin();
	}
	else {
		auto v_it = std::min_element(_candidates.begin(),
									 _candidates.end(),
									 [](const auto& l, const auto& r) -> bool { return l.second < r.second; });
		return v_it->first;
	}
}

void SurfaceMeshPoissonDiskSampling::supportVertex(vertex_descriptor v)
{
	_unsupported_vertices.erase(v);
	_candidates.erase(v);
}

void SurfaceMeshPoissonDiskSampling::addCandidate(std::pair<vertex_descriptor, double> n)
{
	auto unsupported = _unsupported_vertices.find(n.first);
	if (unsupported != _unsupported_vertices.end()) {
		auto found = _candidates.find(n.first);
		if (found != _candidates.end()) { // update distance if nessesary
			if (found->second > n.second) {
				_candidates[n.first] = n.second;
			}
		}
		else { // add new candidate
			_candidates[n.first] = n.second;
		}
	}
}

bool SurfaceMeshPoissonDiskSampling::finished()
{
	return _unsupported_vertices.empty();
}

void SurfaceMeshPoissonDiskSampling::updateUnsupportedVertices(Point point)
{
	std::vector<std::pair<vertex_descriptor, double>> neighbors_in_radius = _search.search(point, _max_radius);
	for (auto & n : neighbors_in_radius)
	{
		if (n.second < _radius) {
			supportVertex(n.first);
		}
		else {
			addCandidate(n);
		}
	}
}



SurfaceMeshPoissonDiskSampling::SurfaceMeshPoissonDiskSampling(const SurfaceMesh & mesh, double radius)
	: _mesh(mesh)
	, _radius(radius)
	, _max_radius(2. * radius)
	, _search(mesh)
{
	for (auto v : _mesh.vertices())
		_unsupported_vertices.insert(v);
}

