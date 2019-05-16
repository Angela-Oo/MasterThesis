#pragma once

#include "mLibInclude.h"
#include "deformation_graph.h"
#include <vector>


typedef ml::TriMeshf Mesh;


template<typename Graph, typename Node>
class DeformedMesh
{
private:
	const DeformationGraph<Graph, Node> & _deformation_graph;
	Mesh _mesh;
	std::vector<NearestNodes> _deformable_points;
private:
	ml::vec3f deformPoint(size_t i);
public:
	Mesh deformPoints();
public:
	DeformedMesh(const Mesh & mesh, const DeformationGraph<Graph, Node> & deformation_graph);
};

template<typename Graph, typename Node>
ml::vec3f DeformedMesh<Graph, Node>::deformPoint(size_t i)
{
	return _deformation_graph.deformPoint(_mesh.m_vertices[i].position, _deformable_points[i]);
}

template<typename Graph, typename Node>
Mesh DeformedMesh<Graph, Node>::deformPoints()
{
	Mesh deformed_points = _mesh;
	for (size_t i = 0; i < _mesh.m_vertices.size(); ++i)
	{
		deformed_points.m_vertices[i].position = deformPoint(i);
	}
	return deformed_points;
}


template<typename Graph, typename Node>
DeformedMesh<Graph, Node>::DeformedMesh(const Mesh & mesh, const DeformationGraph<Graph, Node> & deformation_graph)
	: _mesh(mesh)
	, _deformation_graph(deformation_graph)
{
	_deformable_points.resize(_mesh.m_vertices.size());
	for (size_t i = 0; i < _mesh.m_vertices.size(); ++i)
	{		
		auto & point = _mesh.m_vertices[i].position;
		std::vector<vertex_index> knn_nodes_indices = _deformation_graph._deformation_graph_knn->k_nearest_indices(point, _deformation_graph._k + 1);
		std::vector<double> weights = _deformation_graph.weights(point, knn_nodes_indices);
		_deformable_points[i] = NearestNodes(point, knn_nodes_indices, weights);
	}
}

