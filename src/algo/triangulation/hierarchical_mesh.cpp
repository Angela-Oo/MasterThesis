#include "hierarchical_mesh.h"
#include "triangulation.h"

#include <CGAL/Advancing_front_surface_reconstruction.h>

vertex_descriptor GenerateHierarchicalMesh::nextVertex()
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

void GenerateHierarchicalMesh::supportVertex(vertex_descriptor v) 
{
	_unsupported_vertices.erase(v);
	_candidates.erase(v);
}

void GenerateHierarchicalMesh::addCandidate(std::pair<vertex_descriptor, double> n) 
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

bool GenerateHierarchicalMesh::finished()
{
	return _unsupported_vertices.empty();
}

void GenerateHierarchicalMesh::updateUnsupportedVertices(Point point)
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

void GenerateHierarchicalMesh::insertNodes(SurfaceMesh &hierarchical_mesh)
{
	auto normals = hierarchical_mesh.add_property_map<vertex_descriptor, Vector>("v:normal", Vector(0., 0., 0.)).first;
	auto mesh_normals = _mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
	_candidates[*_unsupported_vertices.begin()] = 0.;

	while (!finished()) {
		auto v = nextVertex();
		auto point = _mesh.point(v);
		auto h_v = hierarchical_mesh.add_vertex(point);
		normals[h_v] = mesh_normals[v];
		supportVertex(v);
		updateUnsupportedVertices(point);
	}
}

void GenerateHierarchicalMesh::triangulateNodes(SurfaceMesh &hierarchical_mesh)
{
	Construct construct(hierarchical_mesh);
	CGAL::advancing_front_surface_reconstruction(hierarchical_mesh.points().begin(),
												 hierarchical_mesh.points().end(),
												 construct);
}

SurfaceMesh GenerateHierarchicalMesh::create()
{
	SurfaceMesh hierarchical_mesh;
	insertNodes(hierarchical_mesh);	
	triangulateNodes(hierarchical_mesh);
	return hierarchical_mesh;
}


GenerateHierarchicalMesh::GenerateHierarchicalMesh(const SurfaceMesh & mesh, double radius)
	: _mesh(mesh)
	, _radius(radius)
	, _max_radius(1.5 * radius)
	, _search(mesh)
{
	for (auto v : mesh.vertices())
		_unsupported_vertices.insert(v);
}





SurfaceMesh createHierarchicalMesh(const SurfaceMesh & mesh, double radius)
{
	GenerateHierarchicalMesh generator(mesh, radius);
	return generator.create();
}
