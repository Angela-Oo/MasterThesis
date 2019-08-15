#include "hierarchical_mesh.h"
#include "algo/nearest_neighbor_search/nearest_neighbor_search.h"
#include "triangulation.h"
#include "mLibCore.h"
#include <CGAL/Polygon_mesh_processing/repair.h>

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
			break;
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
	auto cluster = new_mesh.add_property_map<vertex_descriptor, vertex_descriptor>("v:cluster", vertex_descriptor()).first;
	auto point = original_mesh.point(v_original_mesh);
	auto v = new_mesh.add_vertex(point);

	finer_level_v[v] = v_original_mesh;
	normals[v] = original_mesh_normals[v_original_mesh];
	level[v] = original_mesh_level[v_original_mesh] - 1;
	cluster[v] = v;
	return point;
}

SurfaceMesh HierarchicalMesh::getInitMesh()
{
	return _meshes[0];
}

HierarchicalMesh::HierarchicalMesh(const std::vector<SurfaceMesh> & meshes)
	: _meshes(meshes)
{
	assert(!_meshes.empty());
	for (size_t i = 0; i < _meshes.size() - 1; ++i)
	{
		_vertex_cluster_map.push_back(cluster(_meshes[i], _meshes[i + 1]));
	}
}

HierarchicalMesh::HierarchicalMesh(const HierarchicalMesh & deformation_graph)
	: _meshes(deformation_graph._meshes)
	, _vertex_cluster_map(deformation_graph._vertex_cluster_map)
{
}

HierarchicalMesh & HierarchicalMesh::operator=(HierarchicalMesh other)
{
	if (&other == this)
		return *this;

	_meshes = other._meshes;
	_vertex_cluster_map = other._vertex_cluster_map;
	return *this;
}












std::vector<vertex_descriptor> HierarchicalMeshRefinement::refineVertex(vertex_descriptor v, SurfaceMesh & mesh)
{
	std::vector<vertex_descriptor> new_vertices;
	auto refined = mesh.property_map<vertex_descriptor, bool>("v:refined").first;
	auto color = mesh.property_map<vertex_descriptor, ml::vec4f>("v:color").first;
	auto levels = mesh.property_map<vertex_descriptor, unsigned int>("v:level").first;
	auto l = levels[v];
	if (!refined[v] && l < _hierarchical_mesh._meshes.size()) {
		refined[v] = true;
		if(l == 0)
			color[v] = ml::vec4f(1., 0., 0., 1.);
		else
			color[v] = ml::vec4f(0., 1., 0., 1.);
		auto cluster_id = mesh.property_map<vertex_descriptor, vertex_descriptor>("v:cluster").first;
		auto finer_level_v = mesh.property_map<vertex_descriptor, vertex_descriptor>("v:finer_level_v").first;

		
		auto & child_mesh = _hierarchical_mesh._meshes[l + 1];
		auto finer_level_v_test = child_mesh.property_map<vertex_descriptor, vertex_descriptor>("v:finer_level_v").first;

		const std::vector<vertex_descriptor> & cluster = _hierarchical_mesh._vertex_cluster_map[l].at(cluster_id[v]);
		for (auto c_v : cluster)
		{			
			if (finer_level_v[v] != c_v) {
				auto p = child_mesh.point(c_v);
				auto new_v = mesh.add_vertex(p);
				levels[new_v] = l + 1;
				cluster_id[new_v] = c_v;
				finer_level_v[new_v] = finer_level_v_test[c_v];
				new_vertices.push_back(new_v);
			}
		}
	}
	return new_vertices;
}


std::vector<vertex_descriptor> HierarchicalMeshRefinement::refineEdge(edge_descriptor edge, SurfaceMesh & mesh)
{
	auto v0 = mesh.source(mesh.halfedge(edge));
	auto v1 = mesh.target(mesh.halfedge(edge));

	std::vector<vertex_descriptor> new_vertices = refineVertex(v0, mesh);
	std::vector<vertex_descriptor> new_vertices_v1 = refineVertex(v1, mesh);
	new_vertices.insert(new_vertices.end(), new_vertices_v1.begin(), new_vertices_v1.end());
	return new_vertices;
}

void HierarchicalMeshRefinement::triangulate(SurfaceMesh & mesh)
{
	// removes all faces and edges
	for (auto v : mesh.vertices()) {
		mesh.set_halfedge(v, SurfaceMesh::null_halfedge());
	}
	for (auto e : mesh.edges()) {
		mesh.remove_edge(e);
	}
	for (auto f : mesh.faces()) {
		mesh.remove_face(f);
	}

	surfaceMeshFrontTriangulation(mesh);

}

std::vector<vertex_descriptor> HierarchicalMeshRefinement::refine(std::vector<edge_descriptor> edges, SurfaceMesh & mesh)
{
	std::vector<vertex_descriptor> new_vertices;
	for (auto & e : edges) {
		auto vs = refineEdge(e, mesh);
		new_vertices.insert(new_vertices.end(), vs.begin(), vs.end());
	}
	triangulate(mesh);
	CGAL::Polygon_mesh_processing::remove_isolated_vertices(mesh);

	return new_vertices;
}

HierarchicalMeshRefinement::HierarchicalMeshRefinement(const HierarchicalMesh & hierarchical_mesh)
	: _hierarchical_mesh(hierarchical_mesh)
{
}

