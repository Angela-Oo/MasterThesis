#pragma once
#include <vector>
#include "mLibInclude.h"
#include "mLibFLANN.h"
#include "ext-openmesh/triMesh.h"

class TriMeshKNN {
	const ml::TriMeshf & _mesh;
	ml::NearestNeighborSearchFLANN<float> _neares_neighbor_search;
public:
	TriMeshKNN(const ml::TriMeshf & mesh, unsigned int max_k = 1);
	unsigned int nearest_index(const ml::vec3f & point);
	ml::TriMeshf::Vertex nearest(const ml::vec3f & point);
	std::vector<unsigned int> k_nearest_indices(const ml::vec3f & point, unsigned int k);
	std::vector<ml::TriMeshf::Vertex> k_nearest(const ml::vec3f & point, unsigned int k);
};



class OpenMeshKNN {
	const ml::OpenMeshTriMesh::Mesh & _mesh;
	std::vector<ml::OpenMeshTriMesh::Mesh::VertexHandle> _vertex_indices;
	ml::NearestNeighborSearchFLANN<float> _neares_neighbor_search;
public:
	OpenMeshKNN(const ml::OpenMeshTriMesh::Mesh & mesh, unsigned int max_k = 1);
	ml::OpenMeshTriMesh::Mesh::VertexHandle nearest_index(const OpenMesh::Vec3f & point);
	OpenMesh::Vec3f nearest(const OpenMesh::Vec3f & point);
	std::vector<ml::OpenMeshTriMesh::Mesh::VertexHandle> k_nearest_indices(const OpenMesh::Vec3f & point, unsigned int k);
	std::vector<OpenMesh::Vec3f> k_nearest(const OpenMesh::Vec3f & point, unsigned int k);
};



template<typename Mesh, typename VertexHandle, typename Vertex>
class MeshKNearestNeighbor {
	const Mesh & _mesh;
	std::vector<VertexHandle> _vertex_indices;
	ml::NearestNeighborSearchFLANN<float> _neares_neighbor_search;
public:
	MeshKNearestNeighbor(const Mesh & mesh, unsigned int max_k = 1);
	VertexHandle nearest_index(const Vertex & point);
	Vertex nearest(const Vertex & point);
	std::vector<VertexHandle> k_nearest_indices(const Vertex & point, unsigned int k);
	std::vector<Vertex> k_nearest(const Vertex & point, unsigned int k);
};

template<typename Mesh, typename VertexHandle, typename Vertex>
MeshKNearestNeighbor<Mesh, VertexHandle, Vertex>::MeshKNearestNeighbor(const Mesh & mesh, unsigned int max_k)
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

template<typename Mesh, typename VertexHandle, typename Vertex>
VertexHandle MeshKNearestNeighbor<Mesh, VertexHandle, Vertex>::nearest_index(const Vertex & p)
{
	auto index = _neares_neighbor_search.nearest(p.data());
	return _vertex_indices[index];
}

template<typename Mesh, typename VertexHandle, typename Vertex>
Vertex MeshKNearestNeighbor<Mesh, VertexHandle, Vertex>::nearest(const Vertex & p)
{
	auto index = nearest_index(p);
	return _mesh.point(index);
}

template<typename Mesh, typename VertexHandle, typename Vertex>
std::vector<VertexHandle> MeshKNearestNeighbor<Mesh, VertexHandle, Vertex>::k_nearest_indices(const Vertex & point, unsigned int k)
{
	std::vector<unsigned int> indices = _neares_neighbor_search.kNearest(point.data(), k, 0.000001);
	std::vector<VertexHandle> vertex_indices;
	for (auto & i : indices) {
		vertex_indices.push_back(_vertex_indices[i]);
	}
	return vertex_indices;
}

template<typename Mesh, typename VertexHandle, typename Vertex>
std::vector<Vertex> MeshKNearestNeighbor<Mesh, VertexHandle, Vertex>::k_nearest(const Vertex & point, unsigned int k)
{
	auto indices = k_nearest_indices(point, k);
	std::vector<Vertex> graph_nodes;
	for (auto & i : indices) {
		graph_nodes.push_back(_mesh.point(i));
	}
	return graph_nodes;
}
