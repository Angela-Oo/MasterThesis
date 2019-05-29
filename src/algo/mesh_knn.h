#pragma once
#include <vector>
#include "mLibInclude.h"
#include "mLibFLANN.h"
#include "ext-openmesh/triMesh.h"


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


