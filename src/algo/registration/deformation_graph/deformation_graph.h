#pragma once

#include "mLibInclude.h"
#include <vector>
#include "algo/surface_mesh/mesh_definition.h"
#include "algo/surface_mesh/nearest_neighbor_search.h"
#include "i_deformation.h"
#include <CGAL/squared_distance_3.h> //for 3D functions
#include "nearest_nodes.h"
#include "position_and_deformation.h"

namespace Registration
{


class DeformationGraph
{
public:
	SurfaceMesh _mesh;
	PositionAndDeformation _global;
	std::unique_ptr<NearestNeighborSearch> _knn_search;
	std::function<std::shared_ptr<IDeformation>()> _create_node;
public:
	std::vector<vertex_descriptor> DeformationGraph::getKNearestNodes(const Point & point, unsigned int k) const;
	Point deformPoint(const Point & point, const NearestNodes & nearest_nodes) const;
	Vector deformNormal(const Vector & normal, const NearestNodes & nearest_nodes) const;
public:
	Point getNodePosition(vertex_descriptor node_index) const;
	PositionAndDeformation getNode(vertex_descriptor node_index) const;
	PositionAndDeformation deformNode(vertex_descriptor node_index) const;
	PositionAndDeformation invertNode(vertex_descriptor node_index) const;
public:
	void setRigidDeformation(const PositionAndDeformation & rigid_deformation);
	RigidDeformation getRigidDeformation() const;
public:
	DeformationGraph() = default;
	// all mesh vertices will be deformation nodes
	DeformationGraph(const SurfaceMesh & graph, 
					 const PositionAndDeformation & global_deformation, 
					 std::function<std::shared_ptr<IDeformation>()> create_node);
	DeformationGraph(const DeformationGraph & deformation_graph);
	DeformationGraph & operator=(DeformationGraph other);
};



PositionAndDeformation createGlobalDeformation(const SurfaceMesh & mesh, std::function<std::shared_ptr<IDeformation>()> create_node);

DeformationGraph createDeformationGraphFromMesh(SurfaceMesh mesh,
												PositionAndDeformation global_deformation,
												std::function<std::shared_ptr<IDeformation>()> create_node);


DeformationGraph invertDeformationGraph(const DeformationGraph & deformation_graph);

// transform positions to current transformations and reset transformations
DeformationGraph transformDeformationGraph(const DeformationGraph & deformation_graph);

}