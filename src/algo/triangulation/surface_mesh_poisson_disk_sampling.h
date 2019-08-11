#pragma once

#include "mesh/mesh_definition.h"
#include "algo/nearest_neighbor_search/nearest_neighbor_search.h"

class SurfaceMeshPoissonDiskSampling
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
	void supportVertex(vertex_descriptor v);
	void addCandidate(std::pair<vertex_descriptor, double> n);
	vertex_descriptor nextVertex();
	bool finished();
	void updateUnsupportedVertices(Point point);
public:
	/// creates a surface mesh only containing points that are at least radius appart
	/// the surface mesh only contains points that are also in the mesh passed in the constructor
	SurfaceMesh create(std::function<Point(const SurfaceMesh &, vertex_descriptor, SurfaceMesh &)> add_vertex);
	SurfaceMeshPoissonDiskSampling(const SurfaceMesh & mesh, 
								   double radius);
};


