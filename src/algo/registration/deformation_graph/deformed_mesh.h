#pragma once

#include "mLibInclude.h"
#include "deformation_graph.h"

namespace Registration
{

SurfaceMesh deformationGraphToSurfaceMesh(const DeformationGraph & deformation_graph, bool color_based_on_cost, bool smooth_cost = true, bool fit_cost = false);

class DeformedMesh
{
private:
	const DeformationGraph & _deformation_graph;
	SurfaceMesh _mesh;
	unsigned int _k; // number of interpolated deformation graph nodes per vertex
private:
	NearestNodes createNearestNodes(vertex_descriptor v) const;
public:
	CGAL::Iterator_range<SurfaceMesh::Vertex_iterator> vertices() const;
	uint32_t number_of_vertices() const;
	Point point(SurfaceMesh::Vertex_index v) const;
	Vector normal(SurfaceMesh::Vertex_index v) const;
	Point deformed_point(SurfaceMesh::Vertex_index v) const;
	Vector deformed_normal(SurfaceMesh::Vertex_index v) const;
	NearestNodes & nearestNodes(SurfaceMesh::Vertex_index v) const;
	std::vector<PositionAndDeformation> deformations(SurfaceMesh::Vertex_index v) const;
	SurfaceMesh deformPoints();
public:
	DeformedMesh(const SurfaceMesh & mesh, const DeformationGraph & deformation_graph, unsigned int number_of_interpolation_neighbors);
};


}