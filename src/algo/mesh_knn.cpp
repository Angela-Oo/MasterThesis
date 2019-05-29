#include "stdafx.h"

#include "mesh_knn.h"


OpenMeshKNN::OpenMeshKNN(const ml::OpenMeshTriMesh::Mesh & mesh, unsigned int max_k)
	: _mesh(mesh)
	, _neares_neighbor_search(50, 12)
{
	std::vector<const float*> nn_points;
	for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		_vertex_indices.push_back(v_it.handle());
		nn_points.push_back(mesh.point(v_it.handle()).data());
	}
	_neares_neighbor_search.init(nn_points, 3, max_k);
	nn_points.clear();
}

ml::OpenMeshTriMesh::Mesh::VertexHandle OpenMeshKNN::nearest_index(const OpenMesh::Vec3f & p)
{
	auto index = _neares_neighbor_search.nearest(p.data());
	return _vertex_indices[index];
}

OpenMesh::Vec3f OpenMeshKNN::nearest(const OpenMesh::Vec3f & p)
{
	auto index = nearest_index(p);
	return _mesh.point(index);
}

std::vector<ml::OpenMeshTriMesh::Mesh::VertexHandle> OpenMeshKNN::k_nearest_indices(const OpenMesh::Vec3f & point, unsigned int k)
{
	std::vector<unsigned int> indices = _neares_neighbor_search.kNearest(point.data(), k, 0.000001);
	std::vector<ml::OpenMeshTriMesh::Mesh::VertexHandle> vertex_indices;
	for (auto & i : indices) {
		vertex_indices.push_back(_vertex_indices[i]);
	}
	return vertex_indices;
}

std::vector<OpenMesh::Vec3f> OpenMeshKNN::k_nearest(const OpenMesh::Vec3f & point, unsigned int k)
{
	auto indices = k_nearest_indices(point, k);
	std::vector<OpenMesh::Vec3f> graph_nodes;
	for (auto & i : indices) {
		graph_nodes.push_back(_mesh.point(i));
	}
	return graph_nodes;
}
