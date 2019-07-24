#pragma once

#include "mLibInclude.h"
#include <vector>
#include "algo/surface_mesh/mesh_definition.h"
#include "algo/surface_mesh/nearest_neighbor_search.h"
#include "i_deformation.h"
#include <CGAL/squared_distance_3.h> //for 3D functions
#include "nearest_nodes.h"
#include "position_and_deformation.h"
#include "algo/registration/hsv_to_rgb.h"

namespace Registration
{

template <typename PositionDeformation>
class DeformationGraph
{
public:
	SurfaceMesh _mesh;
	PositionAndDeformation _global;
	std::unique_ptr<NearestNeighborSearch> _knn_search;
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
	DeformationGraph invertDeformation() const;
	void setRigidDeformation(const PositionAndDeformation & rigid_deformation);
	RigidDeformation getRigidDeformation() const;
public:
	DeformationGraph() = default;
	// all mesh vertices will be deformation nodes
	DeformationGraph(const SurfaceMesh & graph, 
					 const PositionAndDeformation & global_deformation);
	DeformationGraph(const DeformationGraph<PositionDeformation> & deformation_graph);
	DeformationGraph<PositionDeformation> & operator=(DeformationGraph<PositionDeformation> other);
};


template <typename PositionDeformation>
std::vector<vertex_descriptor> DeformationGraph<PositionDeformation>::getKNearestNodes(const Point & point, unsigned int k) const
{
	Neighbor_search search = _knn_search->search(point);
	vertex_descriptor nearest_node_index = search.begin()->first;

	auto getNodeDistance = [&](vertex_descriptor vertex_index) {
		return CGAL::squared_distance(point, _mesh.point(vertex_index));
	};

	auto getNodeDistancesAroundVertex = [&](vertex_descriptor vertex_index, std::map<vertex_descriptor, double> & node_distances) {
		for (auto & v : _mesh.vertices_around_target(_mesh.halfedge(vertex_index))) {
			if (node_distances.find(v) == node_distances.end())
				node_distances[v] = getNodeDistance(v);
		}
	};

	std::map<vertex_descriptor, double> node_distance;
	node_distance[nearest_node_index] = getNodeDistance(nearest_node_index);
	getNodeDistancesAroundVertex(nearest_node_index, node_distance);
	if (node_distance.size() < k) {
		for (auto & n : node_distance) {
			getNodeDistancesAroundVertex(n.first, node_distance);
		}
	}

	std::vector<std::pair<vertex_descriptor, double>> sorted_node_distance;
	for (auto & d : node_distance) {
		sorted_node_distance.push_back(d);
	}
	std::sort(sorted_node_distance.begin(), sorted_node_distance.end(),
			  [](const std::pair<vertex_descriptor, double> & rhs, const std::pair<vertex_descriptor, double> & lhs) { return rhs.second < lhs.second; });

	assert(sorted_node_distance.size() >= k);
	std::vector<vertex_descriptor> indices;
	for (unsigned int i = 0; i < k && i < sorted_node_distance.size(); ++i)
		indices.push_back(sorted_node_distance[i].first);
	return indices;
}

template <typename PositionDeformation>
Point DeformationGraph<PositionDeformation>::deformPoint(const Point & point, const NearestNodes & nearest_nodes) const
{
	Vector deformed_point(0., 0., 0.);

	auto & property_map_nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<IPositionDeformation>>("v:node");
	assert(property_map_nodes.second);
	auto & nodes = property_map_nodes.first;

	for (auto n_w : nearest_nodes.node_weight_vector)
	{
		double w = n_w.second;
		auto node = getNode(n_w.first);
		Vector transformed_point = node.deformPosition(point) - CGAL::ORIGIN;
		transformed_point *= w;
		deformed_point += transformed_point;
	}

	Point global_deformed_point = _global.deformPosition(CGAL::ORIGIN + deformed_point);
	return global_deformed_point;
}

template <typename PositionDeformation>
Vector DeformationGraph<PositionDeformation>::deformNormal(const Vector & normal, const NearestNodes & nearest_nodes) const
{
	Vector deformed_normal(0., 0., 0.);

	auto & property_map_nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<IPositionDeformation>>("v:node");
	assert(property_map_nodes.second);
	auto & nodes = property_map_nodes.first;

	for (auto n_w : nearest_nodes.node_weight_vector)
	{
		double w = n_w.second;
		auto node = getNode(n_w.first);
		Vector transformed_normal = node.deformNormal(normal);
		transformed_normal *= w;
		deformed_normal += transformed_normal;
	}
	Vector global_deformed_normal = _global.deformNormal(deformed_normal);
	return global_deformed_normal;
}

template <typename PositionDeformation>
Point DeformationGraph<PositionDeformation>::getNodePosition(vertex_descriptor node_index) const
{
	return _mesh.point(node_index);
}

template <typename PositionDeformation>
PositionAndDeformation DeformationGraph<PositionDeformation>::getNode(vertex_descriptor node_index) const
{
	PositionAndDeformation node;
	auto deformation_nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<IPositionDeformation>>("v:node"); // todo needs to be unique ptr (deep copy not possible)
	assert(deformation_nodes.second);
	std::shared_ptr<IPositionDeformation> n = deformation_nodes.first[node_index];
	node._deformation = n;
	node._point = _mesh.point(node_index);
	return node;
}

template <typename PositionDeformation>
PositionAndDeformation DeformationGraph<PositionDeformation>::deformNode(vertex_descriptor node_index) const
{
	PositionAndDeformation node = getNode(node_index);
	auto deformed_point = _global.deformPosition(node.getDeformedPosition());

	PositionAndDeformation deformed_node;
	deformed_node._point = deformed_point;
	deformed_node._deformation = std::make_shared<PositionDeformation>(deformed_point);
	return deformed_node;
}


template <typename PositionDeformation>
PositionAndDeformation DeformationGraph<PositionDeformation>::invertNode(vertex_descriptor node_index) const
{
	PositionAndDeformation node = getNode(node_index);

	PositionAndDeformation deformed_node;
	deformed_node._point = _global.deformPosition(node.getDeformedPosition());
	deformed_node._deformation = node._deformation->invertDeformation();
	return deformed_node;
}

template <typename PositionDeformation>
void DeformationGraph<PositionDeformation>::setRigidDeformation(const PositionAndDeformation & rigid_deformation)
{
	_global = rigid_deformation;
}

template <typename PositionDeformation>
RigidDeformation DeformationGraph<PositionDeformation>::getRigidDeformation() const
{
	return RigidDeformation(_global._deformation->rotation(),
							_global._deformation->translation(),
							_global._point);
}

template <typename PositionDeformation>
DeformationGraph<PositionDeformation> DeformationGraph<PositionDeformation>::invertDeformation() const
{
	SurfaceMesh mesh = _mesh;

	auto property_deformations = mesh.property_map<vertex_descriptor, std::shared_ptr<IPositionDeformation>>("v:node");
	assert(property_deformations.second);
	auto property_normals = mesh.property_map<vertex_descriptor, Vector>("v:normal");
	assert(property_normals.second);

	auto normals = property_normals.first;
	auto deformations = property_deformations.first;
	for (auto v : mesh.vertices())
	{
		auto deformed_node = invertNode(v);
		mesh.point(v) = deformed_node._point;
		deformations[v] = deformed_node._deformation;
	}

	auto global = _global;
	global._deformation = _global._deformation->invertDeformation();
	return DeformationGraph(mesh, global);
}

template <typename PositionDeformation>
DeformationGraph<PositionDeformation>::DeformationGraph(const SurfaceMesh & graph,
														const PositionAndDeformation & global_deformation)
	: _mesh(graph)
	, _global(global_deformation)
{
	_knn_search = std::make_unique<NearestNeighborSearch>(_mesh);
}

template <typename PositionDeformation>
DeformationGraph<PositionDeformation>::DeformationGraph(const DeformationGraph<PositionDeformation> & deformation_graph)
	: _global(deformation_graph._global)
	, _mesh(deformation_graph._mesh)
{
	// deep copy of deformations
	auto nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<IPositionDeformation>>("v:node");
	assert(nodes.second);
	for (auto & v : _mesh.vertices()) {
		nodes.first[v] = nodes.first[v]->clone();
	}
	_global._deformation = _global._deformation->clone();

	_knn_search = std::make_unique<NearestNeighborSearch>(_mesh);
}



template <typename PositionDeformation>
DeformationGraph<PositionDeformation> & DeformationGraph<PositionDeformation>::operator=(DeformationGraph other)
{
	if (&other == this)
		return *this;

	_global = other._global;
	_mesh = other._mesh;

	// deep copy of deformations
	auto nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<IPositionDeformation>>("v:node");
	assert(nodes.second);
	for (auto & v : _mesh.vertices()) {
		nodes.first[v] = nodes.first[v]->clone();
	}
	_global._deformation = _global._deformation->clone();

	_knn_search = std::make_unique<NearestNeighborSearch>(_mesh);
	return *this;
}


template <typename PositionDeformation>
PositionAndDeformation createGlobalDeformation(const SurfaceMesh & mesh)
{
	Vector global_position(0., 0., 0.);
	for (auto & v : mesh.vertices()) {
		global_position += mesh.point(v) - CGAL::ORIGIN;
	}
	global_position /= mesh.number_of_vertices();

	PositionAndDeformation global;
	global._point = CGAL::ORIGIN + global_position;
	global._deformation = std::make_shared<PositionDeformation>(CGAL::ORIGIN + global_position);
	return global;	
}

template <typename PositionDeformation>
DeformationGraph<PositionDeformation> createDeformationGraphFromMesh(SurfaceMesh mesh,
																	 PositionAndDeformation global_deformation)
{
	SurfaceMesh::Property_map<vertex_descriptor, std::shared_ptr<IPositionDeformation>> nodes;
	bool created;
	boost::tie(nodes, created) = mesh.add_property_map<vertex_descriptor, std::shared_ptr<IPositionDeformation>>("v:node", std::make_shared<PositionDeformation>(CGAL::ORIGIN));
	assert(created);
	//mesh.add_property_map<vertex_descriptor, double>("v:fit_cost", 0.);
	mesh.add_property_map<edge_descriptor, double>("e:smooth_cost", 0.);
	//mesh.add_property_map<vertex_descriptor, double>("v:conf_cost", 0.);
	//mesh.add_property_map<vertex_descriptor, bool>("v:vertex_used", true);

	auto vertex_color = errorToRGB(0.);
	auto colors = mesh.add_property_map<vertex_descriptor, ml::vec4f>("v:color", vertex_color).first;

	for (auto & v : mesh.vertices()) {
		nodes[v] = std::make_shared<PositionDeformation>(mesh.point(v)); // todo
		colors[v] = vertex_color;
	}

	return DeformationGraph<PositionDeformation>(mesh, global_deformation);
}



}