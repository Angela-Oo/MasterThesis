#pragma once
#include <vector>
#include "mLibInclude.h"
#include "ext-flann/nearestNeighborSearchFLANN.h"
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

