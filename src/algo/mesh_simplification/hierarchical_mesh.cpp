#include "hierarchical_mesh.h"


vertex_descriptor GenerateHierarchicalMesh::nextVertex()
{
	auto v_it = std::min_element(_candidates.begin(),
								 _candidates.end(),
								 [](const auto& l, const auto& r) -> bool { return l.second < r.second; });
	return v_it->first;
}

void GenerateHierarchicalMesh::updateUnsupportedVertices(Point point)
{
	std::map<vertex_descriptor, double> neighbors_in_radius = _search.search(point, _max_radius);
	for (auto & n : neighbors_in_radius) {
		if (n.second < _radius) {
			_unsupported_vertices.erase(n.first);
			_candidates.erase(n.first);
		}
		else {
			auto found = _candidates.find(n.first);
			if (found != _candidates.end()) {
				if (found->second > n.second) {
					_candidates[n.first] = n.second;
				}
			}
			else {
				_candidates[n.first] = n.second;
			}
		}
	}
}


SurfaceMesh GenerateHierarchicalMesh::create()
{
	SurfaceMesh hierarchical_mesh;
	_candidates[*_unsupported_vertices.begin()] = 0.;

	vertex_descriptor v = _candidates.begin()->first;

	for (int i = 0; i < 5; ++i) {
		hierarchical_mesh.add_vertex(_mesh.point(v));
		_unsupported_vertices.erase(v);
		_candidates.erase(v);
		updateUnsupportedVertices(_mesh.point(v));
		v = nextVertex();
	}
	return hierarchical_mesh;
}


GenerateHierarchicalMesh::GenerateHierarchicalMesh(const SurfaceMesh & mesh, double radius)
	: _mesh(mesh)
	, _radius(radius)
	, _max_radius(2. * radius)
	, _search(mesh)
{
	for (auto v : mesh.vertices())
		_unsupported_vertices.insert(v);
}


//void updateUnsupportedVertices(RadiusNearestNeighborSearch &search, 
//							   Point point, 
//							   double radius,
//							   double max_radius,
//							   std::map<vertex_descriptor, double> &unsupported_vertices)
//{
//	
//	std::map<vertex_descriptor, double> neighbors_in_radius = search.search(point, max_radius);
//	for (auto & n : neighbors_in_radius) {
//		if (n.second < radius) {
//			unsupported_vertices.erase(n.first);
//		}
//		else {
//			unsupported_vertices[n.first] = n.second;
//		}
//	}
//}


SurfaceMesh createHierarchicalMesh(const SurfaceMesh & mesh, double radius)
{
	GenerateHierarchicalMesh generator(mesh, radius);
	return generator.create();
	//std::map<vertex_descriptor, double> unsupported_vertices;
	//for (auto v : mesh.vertices())
	//	unsupported_vertices[v] = INFINITY;

	//RadiusNearestNeighborSearch search(mesh);
	//SurfaceMesh hierarchical_mesh;

	//double max_radius = 2. * radius;
	//vertex_descriptor v = (*unsupported_vertices.begin()).first;

	//for (int i = 0; i < 5; ++i) {
	//	hierarchical_mesh.add_vertex(mesh.point(v));
	//	unsupported_vertices.erase(v);
	//	updateUnsupportedVertices(search, mesh.point(v), radius, max_radius, unsupported_vertices);
	//	auto v_it = std::min_element(unsupported_vertices.begin(), 
	//								 unsupported_vertices.end(),
	//								 [](const auto& l, const auto& r) -> bool{ return l.second < r.second; });
	//	v = v_it->first;
	//}
	//return hierarchical_mesh;
	//return createReducedMesh(mesh, radius);
}
