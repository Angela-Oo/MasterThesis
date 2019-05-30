#pragma once

#include "mLibInclude.h"
#include <vector>
#include "algo/surface_mesh/mesh_definition.h"
#include "algo/surface_mesh/nearest_neighbor_search.h"
#include "i_deformation.h"
#include <CGAL/squared_distance_3.h> //for 3D functions
#include "nearest_nodes.h"
#include "position_and_deformation.h"

namespace DG
{


class DeformationGraph
{
public:
	const int _k = 4;
	SurfaceMesh _mesh;
	PositionAndDeformation _global;
	std::unique_ptr<NearestNeighborSearch> _knn_search;
	std::function<std::shared_ptr<IDeformation>()> _create_node;
private:
	void initGlobalDeformation(std::shared_ptr<IDeformation> global_deformation);
public:
	std::vector<double> weights(const Point & point, std::vector<vertex_descriptor>& nearest_nodes_indices) const;
	std::vector<vertex_descriptor> nearestNodes(const Point & point) const;
	Point deformPoint(const Point & point, const NearestNodes & nearest_nodes) const;
public:
	PositionAndDeformation getNode(vertex_descriptor node_index) const;
	PositionAndDeformation deformNode(vertex_descriptor node_index) const;
	PositionAndDeformation invertNode(vertex_descriptor node_index) const;
public:
	DeformationGraph() = default;
	// all mesh vertices will be deformation nodes
	DeformationGraph(const SurfaceMesh & nodes, std::function<std::shared_ptr<IDeformation>()> create_node);
	DeformationGraph(const SurfaceMesh & mesh, const std::shared_ptr<IDeformation> & global_deformation, std::function<std::shared_ptr<IDeformation>()> create_node);
	DeformationGraph(const DeformationGraph & deformation_graph);
	DeformationGraph & operator=(DeformationGraph other);
};

DeformationGraph invertDeformationGraph(const DeformationGraph & deformation_graph);

// transform positions to current transformations and reset transformations
DeformationGraph transformDeformationGraph(const DeformationGraph & deformation_graph);

}