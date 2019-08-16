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


MeshLevel getMeshLevel(const SurfaceMesh & mesh, const vertex_descriptor & v)
{
	auto levels = mesh.property_map<vertex_descriptor, MeshLevel>("v:level");
	if (mesh.is_valid(v) && levels.second) {
		return levels.first[v];
	}
	else {
		throw std::exception("mesh does not contain expected vertex or has no property map v:level");
	}
}



SurfaceMesh HierarchicalMesh::getInitMesh()
{
	SurfaceMesh mesh = _meshes[0];
	mesh.add_property_map<edge_descriptor, ml::vec4f>("e:color", ml::vec4f(1., 1., 1., 1.));
	mesh.add_property_map<vertex_descriptor, ml::vec4f>("v:color", ml::vec4f(1., 1., 1., 1.));
	mesh.add_property_map<vertex_descriptor, bool>("v:refined", false);
	return mesh;
}

bool HierarchicalMesh::validLevel(unsigned int level) const
{
	return level < _meshes.size();
}

size_t HierarchicalMesh::size() const
{
	return _meshes.size();
}

const SurfaceMesh & HierarchicalMesh::getMesh(unsigned int level) const
{
	assert(level < _meshes.size());
	return _meshes[level];
}

const std::map<vertex_descriptor, std::vector<vertex_descriptor>> & HierarchicalMesh::getClusters(unsigned int level) const
{
	assert(level < _meshes.size());
	return _vertex_cluster_map[level];
}

const std::vector<vertex_descriptor> & HierarchicalMesh::getCluster(const MeshLevel & v) const
{
	auto clusters = getClusters(v.level);
	return clusters.at(v.cluster_v);
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
	//auto color = mesh.property_map<vertex_descriptor, ml::vec4f>("v:color").first;
	auto levels = mesh.property_map<vertex_descriptor, MeshLevel>("v:level").first;

	auto mesh_level = levels[v];
	if (!refined[v] && _hierarchical_mesh.validLevel(mesh_level.level)) {
		refined[v] = true;
		auto & child_mesh = _hierarchical_mesh.getMesh(mesh_level.level + 1);

		const std::vector<vertex_descriptor> & cluster = _hierarchical_mesh.getCluster(mesh_level);
		for (auto c_v : cluster)
		{
			if (mesh_level.cluster_v_finer_level != c_v) {
				auto p = child_mesh.point(c_v);
				auto new_v = mesh.add_vertex(p);
				levels[new_v] = getMeshLevel(child_mesh, c_v);

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

