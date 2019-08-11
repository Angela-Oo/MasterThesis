#include "hierarchical_mesh.h"
#include "algo/nearest_neighbor_search/nearest_neighbor_search.h"
#include "triangulation.h"
#include "mLibCore.h"

std::map<vertex_descriptor, std::vector<vertex_descriptor>> cluster(const SurfaceMesh & mesh, const SurfaceMesh & child_mesh)
{
	std::map<vertex_descriptor, std::vector<vertex_descriptor>> vertex_cluster_map;
	NearestNeighborSearch search(mesh);
	for (auto v : child_mesh.vertices())
	{
		auto s = search.search(child_mesh.point(v), 1);
		for (Neighbor_search::iterator it = s.begin(); it != s.end(); ++it) {
			auto nearest_v = it->first;
			vertex_cluster_map[nearest_v].push_back(v);
		}		
	}
	return vertex_cluster_map;
}


Point add_vertex(const SurfaceMesh & original_mesh, vertex_descriptor v_original_mesh, SurfaceMesh & new_mesh)
{
	auto original_mesh_normals = original_mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
	auto original_mesh_level = original_mesh.property_map<vertex_descriptor, unsigned int>("v:level").first;

	auto normals = new_mesh.add_property_map<vertex_descriptor, Vector>("v:normal", Vector(0., 0., 0.)).first;
	auto finer_level_v = new_mesh.add_property_map<vertex_descriptor, vertex_descriptor>("v:finer_level_v", vertex_descriptor()).first;
	auto level = new_mesh.add_property_map<vertex_descriptor, unsigned int>("v:level", 0).first;

	auto point = original_mesh.point(v_original_mesh);
	auto v = new_mesh.add_vertex(point);

	finer_level_v[v] = v_original_mesh;
	normals[v] = original_mesh_normals[v_original_mesh];
	level[v] = original_mesh_level[v_original_mesh] - 1;

	return point;
}


void HierarchicalMesh::refineVertex(vertex_descriptor v)
{
	auto refined = _mesh.property_map<vertex_descriptor, bool>("v:refined").first;
	if (!refined[v]) {
		refined[v] = true;
		auto finer_level_v = _mesh.property_map<vertex_descriptor, vertex_descriptor>("v:finer_level_v").first;
		auto level = _mesh.property_map<vertex_descriptor, unsigned int>("v:level").first;

		auto l = level[v];
		auto next_l = l + 1;
		auto map = _vertex_cluster_map[l];
		auto & child_mesh = _meshes[next_l];

		if (l < _meshes.size())
		{
			auto cluster = map[v];
			for (auto c_v : cluster)
			{
				auto p = child_mesh.point(c_v);
				auto new_v = _mesh.add_vertex(p);
			}
		}
	}
}

void HierarchicalMesh::refineEdge(edge_descriptor edge)
{
	auto v0 = _mesh.source(_mesh.halfedge(edge));
	auto v1 = _mesh.target(_mesh.halfedge(edge));
	refineVertex(v0);
	refineVertex(v1);
}

void HierarchicalMesh::triangulate()
{
	// removes all faces and edges
	for (auto v : _mesh.vertices()) {
		_mesh.set_halfedge(v, SurfaceMesh::null_halfedge());
	}
	for (auto e : _mesh.edges()) {		
		_mesh.remove_edge(e);
	}
	for (auto f : _mesh.faces()) {
		_mesh.remove_face(f);
	}

	surfaceMeshFrontTriangulation(_mesh);
}

HierarchicalMesh::HierarchicalMesh(const std::vector<SurfaceMesh> & meshes)
	: _meshes(meshes)
{
	assert(!_meshes.empty());
	_mesh = _meshes[0];

	for (size_t i = 0; i < _meshes.size() - 1; ++i)
	{
		_vertex_cluster_map.push_back(cluster(_meshes[i], _meshes[i + 1]));
	}
}

HierarchicalMesh::HierarchicalMesh(const HierarchicalMesh & deformation_graph)
	: _mesh(deformation_graph._mesh)
	, _meshes(deformation_graph._meshes)
	, _vertex_cluster_map(deformation_graph._vertex_cluster_map)
{
}

HierarchicalMesh & HierarchicalMesh::operator=(HierarchicalMesh other)
{
	if (&other == this)
		return *this;

	_meshes = other._meshes;
	_mesh = other._mesh;
	_vertex_cluster_map = other._vertex_cluster_map;
	return *this;
}