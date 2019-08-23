#pragma once

#include "nearest_nodes.h"
#include "i_deformation.h"
#include "algo/registration/util/hsv_to_rgb.h"
#include "mesh/mesh_definition.h"
#include "algo/nearest_neighbor_search/nearest_neighbor_search.h"
#include <CGAL/squared_distance_3.h> //for 3D functions
#include <vector>
#include "algo/registration/util/math.h"
#include "algo/registration/util/dual_quaternion.h"

namespace Registration
{

template <typename PositionDeformation>
class DeformationGraph
{
public:
	SurfaceMesh _mesh;
	PositionDeformation _global;
private:
	unsigned int _k; // number of interpolation neighbors
public:
	std::unique_ptr<NearestNeighborSearch> _knn_search;
public:
	unsigned int getNumberOfInterpolationNeighbors() const { return _k; }
public:
	std::vector<vertex_descriptor> DeformationGraph::getKNearestNodes(const Point & point, unsigned int k) const;
	Point deformPoint(const Point & point, const NearestNodes & nearest_nodes) const;
	Point deformDLBPoint(const Point & point, const NearestNodes & nearest_nodes) const;
	Vector deformNormal(const Vector & normal, const NearestNodes & nearest_nodes) const;
public:
	PositionDeformation & getDeformation(vertex_descriptor node_index) const;
	PositionDeformation deformNode(vertex_descriptor node_index) const;
public:
	DeformationGraph invertDeformation() const;
	void setRigidDeformation(const RigidDeformation & rigid_deformation);
	RigidDeformation getRigidDeformation() const;
public:
	DeformationGraph() = default;
	// all mesh vertices will be deformation nodes
	DeformationGraph(const SurfaceMesh & graph, 
					 const PositionDeformation & global_deformation,
					 unsigned int k);
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

//
//template <typename PositionDeformation>
//std::vector<vertex_descriptor> DeformationGraph<PositionDeformation>::getKNearestNodesTest(const Point & point, unsigned int k) const
//{
//	Neighbor_search search = _knn_search->search(point, k);
//	std::vector<vertex_descriptor> indices;
//	for (Neighbor_search::iterator it = search.begin(); it != search.end(); ++it) {
//		indices.push_back(it->first);
//	}
//	return indices;
//}

template <typename PositionDeformation>
Point DeformationGraph<PositionDeformation>::deformPoint(const Point & point, const NearestNodes & nearest_nodes) const
{
	Vector deformed_point(0., 0., 0.);

	// vi' = sum_j  wj(vi) * [ Rj(vi - gj) + gj + tj ]
	for (auto n_w : nearest_nodes.node_weight_vector)
	{
		double w = n_w.second;
		auto deformation = getDeformation(n_w.first);
		//  [ Rj(vi - gj) + gj + tj ]
		Vector transformed_point = deformation.deformPosition(point) - CGAL::ORIGIN;
		//  wj(vi) * [ Rj(vi - gj) + gj + tj ]
		transformed_point *= w;
		deformed_point += transformed_point;
	}

	Point global_deformed_point = _global.deformPosition(CGAL::ORIGIN + deformed_point);
	return global_deformed_point;
}

template <typename PositionDeformation>
Point DeformationGraph<PositionDeformation>::deformDLBPoint(const Point & point, const NearestNodes & nearest_nodes) const
{
	// vi' = sum_j  wj(vi) * [ Rj(vi - gj) + gj + tj ]
	DualQuaternion dq(ml::mat4d::identity());
	for (auto n_w : nearest_nodes.node_weight_vector)
	{
		double w = n_w.second;
		auto deformation = getDeformation(n_w.first);
		auto dual_transformation = deformation.deformDLBPosition();
		DualNumber weight(w, w);// 0.);// w);
		dual_transformation = dual_transformation * weight;
		dq += dual_transformation;
	}

	dq.normalize();
	ml::mat4d transformation = dq.operator ml::mat4d();
	ml::vec3d deformed_point = transformation * convertVector(point - CGAL::ORIGIN);

	Point global_deformed_point = _global.deformPosition(CGAL::ORIGIN + convertVector(deformed_point));
	return global_deformed_point;
}

template <typename PositionDeformation>
Vector DeformationGraph<PositionDeformation>::deformNormal(const Vector & normal, const NearestNodes & nearest_nodes) const
{
	Vector deformed_normal(0., 0., 0.);

	auto & property_map_nodes = _mesh.property_map<vertex_descriptor, PositionDeformation>("v:node_deformation");
	assert(property_map_nodes.second);
	auto & nodes = property_map_nodes.first;

	for (auto n_w : nearest_nodes.node_weight_vector)
	{
		double w = n_w.second;
		auto node = getDeformation(n_w.first);
		Vector transformed_normal = node.deformNormal(normal);
		transformed_normal *= w;
		deformed_normal += transformed_normal;
	}
	Vector global_deformed_normal = _global.deformNormal(deformed_normal);
	return global_deformed_normal;
}

template <typename PositionDeformation>
PositionDeformation & DeformationGraph<PositionDeformation>::getDeformation(vertex_descriptor node_index) const
{
	auto deformation_nodes = _mesh.property_map<vertex_descriptor, PositionDeformation>("v:node_deformation");
	assert(deformation_nodes.second);
	return deformation_nodes.first[node_index];
}

template <typename PositionDeformation>
PositionDeformation DeformationGraph<PositionDeformation>::deformNode(vertex_descriptor node_index) const
{
	auto node = getDeformation(node_index);
	Point deformed_point = _global.deformPosition(node.getDeformedPosition());
	return PositionDeformation(deformed_point);
}

template <typename PositionDeformation>
void DeformationGraph<PositionDeformation>::setRigidDeformation(const RigidDeformation & rigid_deformation)
{
	_global = PositionDeformation(rigid_deformation);
}

template <typename PositionDeformation>
RigidDeformation DeformationGraph<PositionDeformation>::getRigidDeformation() const
{
	return RigidDeformation(_global.rotation(),
							_global.translation(),
							_global.position());
}

template <typename PositionDeformation>
DeformationGraph<PositionDeformation> DeformationGraph<PositionDeformation>::invertDeformation() const
{
	SurfaceMesh mesh = _mesh;
	auto property_deformations = mesh.property_map<vertex_descriptor, PositionDeformation>("v:node_deformation");
	assert(property_deformations.second);
	auto property_normals = mesh.property_map<vertex_descriptor, Vector>("v:normal");
	assert(property_normals.second);

	auto normals = property_normals.first;
	auto deformations = property_deformations.first;
	for (auto v : mesh.vertices())
	{
		auto & node = getDeformation(v);
		deformations[v] = node.invertDeformation();
		mesh.point(v) = deformations[v].position();
	}

	auto global = _global.invertDeformation();
	return DeformationGraph(mesh, global, _k);
}

template <typename PositionDeformation>
DeformationGraph<PositionDeformation>::DeformationGraph(const SurfaceMesh & graph,
														const PositionDeformation & global_deformation,
														unsigned int k)
	: _mesh(graph)
	, _global(global_deformation)
	, _k(k)
{
	_knn_search = std::make_unique<NearestNeighborSearch>(_mesh);
}



template <typename PositionDeformation>
DeformationGraph<PositionDeformation>::DeformationGraph(const DeformationGraph<PositionDeformation> & deformation_graph)
	: _global(deformation_graph._global)
	, _mesh(deformation_graph._mesh)
	, _k(deformation_graph._k)
{
	_knn_search = std::make_unique<NearestNeighborSearch>(_mesh);
}

template <typename PositionDeformation>
DeformationGraph<PositionDeformation> & DeformationGraph<PositionDeformation>::operator=(DeformationGraph other)
{
	if (&other == this)
		return *this;

	_global = other._global;
	_mesh = other._mesh;
	_k = other._k;

	_knn_search = std::make_unique<NearestNeighborSearch>(_mesh);
	return *this;
}

//-----------------------------------------------------------------------------

template <typename PositionDeformation>
PositionDeformation createGlobalDeformation(const SurfaceMesh & mesh)
{
	Vector global_position(0., 0., 0.);
	for (auto & v : mesh.vertices()) {
		global_position += mesh.point(v) - CGAL::ORIGIN;
	}
	global_position /= mesh.number_of_vertices();
	return PositionDeformation(CGAL::ORIGIN + global_position);
}

template <typename PositionDeformation>
DeformationGraph<PositionDeformation> createDeformationGraphFromMesh(SurfaceMesh mesh,
																	 PositionDeformation global_deformation,
																	 unsigned int number_of_interpolation_neighbors)
{
	SurfaceMesh::Property_map<vertex_descriptor, PositionDeformation> nodes;
	bool created;
	boost::tie(nodes, created) = mesh.add_property_map<vertex_descriptor, PositionDeformation>("v:node_deformation", PositionDeformation(CGAL::ORIGIN));
	assert(created);
	mesh.add_property_map<edge_descriptor, double>("e:smooth_cost", 0.);

	auto vertex_color = errorToRGB(0.);
	auto colors = mesh.add_property_map<vertex_descriptor, ml::vec4f>("v:color", vertex_color).first;
	for (auto & v : mesh.vertices()) {
		nodes[v] = PositionDeformation(mesh.point(v));
		colors[v] = vertex_color;
	}
	return DeformationGraph<PositionDeformation>(mesh, global_deformation, number_of_interpolation_neighbors);
}





}