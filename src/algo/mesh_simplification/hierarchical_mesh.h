#pragma once

#include "algo/surface_mesh/mesh_definition.h"
#include "algo/surface_mesh/nearest_neighbor_search.h"

class GenerateHierarchicalMesh
{
private:
	const SurfaceMesh & _mesh;
	double _radius;
	double _max_radius;
private:
	std::set<vertex_descriptor> _unsupported_vertices;
	std::map<vertex_descriptor, double> _candidates;
	RadiusNearestNeighborSearch _search;
private:
	vertex_descriptor nextVertex();
	void updateUnsupportedVertices(Point point);
public:
	SurfaceMesh create();
	GenerateHierarchicalMesh(const SurfaceMesh & mesh, double radius);
};

SurfaceMesh createHierarchicalMesh(const SurfaceMesh & mesh, double radius);

