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

Point add_vertex(const SurfaceMesh & original_mesh, vertex_descriptor v_original_mesh, SurfaceMesh & new_mesh)
{
	auto original_mesh_normals = original_mesh.property_map<vertex_descriptor, Vector>("v:normal").first;

	auto normals = new_mesh.add_property_map<vertex_descriptor, Vector>("v:normal", Vector(0., 0., 0.)).first;
	auto finer_level_v = new_mesh.add_property_map<vertex_descriptor, vertex_descriptor>("v:finer_level_v", vertex_descriptor()).first;
	auto level = new_mesh.add_property_map<vertex_descriptor, unsigned int>("v:level", 0).first;

	auto point = original_mesh.point(v_original_mesh);
	auto v = new_mesh.add_vertex(point);

	finer_level_v[v] = v_original_mesh;
	normals[v] = original_mesh_normals[v_original_mesh];

	return point;
}
//
//Point add_vertex(const SurfaceMesh & original_mesh, vertex_descriptor v_original_mesh, SurfaceMesh & new_mesh)
//{
//	auto original_mesh_normals = original_mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
//
//	auto normals = new_mesh.add_property_map<vertex_descriptor, Vector>("v:normal", Vector(0., 0., 0.)).first;
//
//	auto point = original_mesh.point(v_original_mesh);
//	auto v = new_mesh.add_vertex(point);
//	normals[v] = original_mesh_normals[v_original_mesh];
//	return point;
//}

SurfaceMesh SurfaceMeshPoissonDiskSampling::create()
{
	SurfaceMesh hierarchical_mesh;	
	_candidates[*_unsupported_vertices.begin()] = 0.;

	while (!finished()) {
		auto v = nextVertex();
		auto point = add_vertex(_mesh, v, hierarchical_mesh);
		supportVertex(v);
		updateUnsupportedVertices(point);
	}
	return hierarchical_mesh;
}

SurfaceMeshPoissonDiskSampling::SurfaceMeshPoissonDiskSampling(const SurfaceMesh & mesh, double radius)
	: _mesh(mesh)
	, _radius(radius)
	, _max_radius(2. * radius)
	, _search(mesh)
{
	for (auto v : mesh.vertices())
		_unsupported_vertices.insert(v);
}

