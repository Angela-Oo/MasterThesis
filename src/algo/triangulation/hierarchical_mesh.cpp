#include "hierarchical_mesh.h"

void HierarchicalMesh::refineEdge(edge_descriptor edge)
{
	auto v0 = _mesh.target(_mesh.halfedge(edge));
	auto finer_level_v = _mesh.property_map<vertex_descriptor, vertex_descriptor>("v:finer_level_v").first[v0];
	auto level = _mesh.property_map<vertex_descriptor, unsigned int>("v:level").first[v0];
	auto finer_level_point = _mesh.property_map<vertex_descriptor, Point>("v:finer_level_point").first;
		
	//if (level > 0) {
		auto & m = _meshes[level + 1];

		auto new_v = _mesh.add_vertex(finer_level_point[v0]);
		_mesh.add_edge(new_v, v0);
		
		auto t = m.point(finer_level_v);
		auto v_t = _mesh.add_vertex(t);
		_mesh.add_edge(v_t, v0);
		for (auto v : m.vertices_around_target(m.halfedge(finer_level_v)))
		{
			auto p = m.point(v);
			auto new_v = _mesh.add_vertex(p);
			_mesh.add_edge(new_v, v0);
		}
	//}
}

HierarchicalMesh::HierarchicalMesh(const std::vector<SurfaceMesh> & meshes)
	: _meshes(meshes)
{
	assert(!_meshes.empty());
	_mesh = _meshes[0];
}

