#include "deformation_graph.h"
#include "algo/registration/hsv_to_rgb.h"
#include <cassert>



namespace DG {


std::vector<double> nodeDistanceWeighting(const ml::vec3f & point, const std::vector<ml::vec3f>& node_positions)
{
	auto last_node = node_positions[node_positions.size() - 1];
	double d_max = ml::dist(point, last_node);

	std::vector<double> weights;
	for (size_t i = 0; i < node_positions.size() - 1; ++i)
	{
		double normed_distance = ml::dist(point, node_positions[i]);
		double weight = std::pow(1. - (normed_distance / d_max), 2);
		weights.push_back(weight);
	}

	double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
	std::for_each(weights.begin(), weights.end(), [sum](double & w) { w = w / sum; });
	return weights;
}


std::vector<vertex_descriptor> DeformationGraph::getKNearestNodes(const Point & point, unsigned int k) const
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
	if (sorted_node_distance.size() < k)
		std::cout << "help not enouth nodes found" << std::endl;
	std::vector<vertex_descriptor> indices;
	for (int i = 0; i < k && i < sorted_node_distance.size(); ++i)
		indices.push_back(sorted_node_distance[i].first);
	return indices;
}


Point DeformationGraph::deformPoint(const Point & point, const NearestNodes & nearest_nodes) const
{
	Vector deformed_point(0.,0.,0.);

	auto & property_map_nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node");
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

Vector DeformationGraph::deformNormal(const Vector & normal, const NearestNodes & nearest_nodes) const
{
	Vector deformed_normal(0., 0., 0.);

	auto & property_map_nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node");
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

Point DeformationGraph::getNodePosition(vertex_descriptor node_index) const
{
	return _mesh.point(node_index);
}

PositionAndDeformation DeformationGraph::getNode(vertex_descriptor node_index) const
{
	PositionAndDeformation node;
	auto deformation_nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node"); // todo needs to be unique ptr (deep copy not possible)
	assert(deformation_nodes.second);
	std::shared_ptr<IDeformation> n = deformation_nodes.first[node_index];
	node._deformation = n;
	node._point = _mesh.point(node_index);
	node._normal = _mesh.property_map<vertex_descriptor, Vector>("v:normal").first[node_index];
	return node;
}

PositionAndDeformation DeformationGraph::deformNode(vertex_descriptor node_index) const
{
	PositionAndDeformation node = getNode(node_index);

	PositionAndDeformation deformed_node;
	deformed_node._point = _global.deformPosition(node.getDeformedPosition());
	deformed_node._normal = _global.deformNormal(node.getDeformedNormal());
	deformed_node._deformation = _create_node();
	return deformed_node;
}


PositionAndDeformation DeformationGraph::invertNode(vertex_descriptor node_index) const
{
	PositionAndDeformation node = getNode(node_index);

	PositionAndDeformation deformed_node;
	deformed_node._point = _global.deformPosition(node.getDeformedPosition());
	deformed_node._normal = _global.deformNormal(node.getDeformedNormal());
	deformed_node._deformation = node._deformation->invertDeformation();
	return deformed_node;
}

void DeformationGraph::initGlobalDeformation(std::shared_ptr<IDeformation> global_deformation)
{
	Vector global_position(0., 0., 0.);
	for (auto & v : _mesh.vertices()) {
		global_position += _mesh.point(v) - CGAL::ORIGIN;
	}
	global_position /= _mesh.number_of_vertices();

	_global._point = CGAL::ORIGIN + global_position;
	_global._normal = Vector(0., 0., 1.);
	_global._deformation = global_deformation;
}

DeformationGraph::DeformationGraph(const SurfaceMesh & mesh,
								   std::function<std::shared_ptr<IDeformation>()> create_node)
	: _mesh(mesh)
	, _create_node(create_node)
{

	SurfaceMesh::Property_map<vertex_descriptor, std::shared_ptr<IDeformation>> nodes;
	bool created;
	boost::tie(nodes, created) = _mesh.add_property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node", create_node());
	assert(created);
	_mesh.add_property_map<vertex_descriptor, double>("v:fit_cost", 0.);
	_mesh.add_property_map<edge_descriptor, double>("e:smooth_cost", 0.);
	_mesh.add_property_map<vertex_descriptor, double>("v:conf_cost", 0.);
	_mesh.add_property_map<vertex_descriptor, bool>("v:vertex_used", true);

	auto vertex_color = errorToRGB(0.);
	auto colors = _mesh.add_property_map<vertex_descriptor, ml::vec4f>("v:color", vertex_color).first;

	auto normals = _mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
	for (auto & v : _mesh.vertices()) {
		nodes[v] = _create_node();
		colors[v] = vertex_color;
	}
	initGlobalDeformation(_create_node());

	_knn_search = std::make_unique<NearestNeighborSearch>(_mesh);
}


DeformationGraph::DeformationGraph(const SurfaceMesh & graph, 
								   const std::shared_ptr<IDeformation> & global_deformation, 
								   std::function<std::shared_ptr<IDeformation>()> create_node)
	: _mesh(graph)
	, _create_node(create_node)
{
	initGlobalDeformation(global_deformation);
	_knn_search = std::make_unique<NearestNeighborSearch>(_mesh);
}


DeformationGraph::DeformationGraph(const DeformationGraph & deformation_graph)
	: _global(deformation_graph._global)
	, _mesh(deformation_graph._mesh)
	, _create_node(deformation_graph._create_node)
{
	_knn_search = std::make_unique<NearestNeighborSearch>(_mesh);
}

DeformationGraph & DeformationGraph::operator=(DeformationGraph other)
{
	if (&other == this)
		return *this;

	_global = other._global;
	_mesh = other._mesh;
	_create_node = other._create_node;
	_knn_search = std::make_unique<NearestNeighborSearch>(_mesh);
	return *this;
}




DeformationGraph invertDeformationGraph(const DeformationGraph & deformation_graph)
{
	SurfaceMesh mesh = deformation_graph._mesh;

	auto property_deformations = mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node");
	assert(property_deformations.second);
	auto property_normals = mesh.property_map<vertex_descriptor, Vector>("v:normal");
	assert(property_normals.second);

	auto normals = property_normals.first;
	auto deformations = property_deformations.first;
	for (auto v : mesh.vertices())
	{
		auto deformed_node = deformation_graph.invertNode(v);
		mesh.point(v) = deformed_node._point;
		normals[v] = deformed_node._normal;
		deformations[v] = deformed_node._deformation;
	}

	auto global_deformation = deformation_graph._global._deformation->invertDeformation();
	return DeformationGraph(mesh, global_deformation, deformation_graph._create_node);
}

DeformationGraph transformDeformationGraph(const DeformationGraph & deformation_graph)
{
	SurfaceMesh mesh = deformation_graph._mesh;
	
	//auto property_fit_cost = mesh.property_map<vertex_descriptor, double>("v:fit_cost");
	//auto property_smooth_cost = mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	//auto property_conf_cost = mesh.property_map<vertex_descriptor, double>("v:conf_cost");
	//auto property_vertex_used = mesh.property_map<vertex_descriptor, bool>("v:vertex_used");

	auto property_deformations = mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node");
	assert(property_deformations.second);
	auto property_normals = mesh.property_map<vertex_descriptor, Vector>("v:normal");
	assert(property_normals.second);

	auto normals = property_normals.first;
	auto deformations = property_deformations.first;
	for (auto v : mesh.vertices())
	{
		auto deformed_node = deformation_graph.deformNode(v);
		mesh.point(v) = deformed_node._point;
		normals[v] = deformed_node._normal;
		deformations[v] = deformed_node._deformation;
	}

	return DeformationGraph(mesh, deformation_graph._create_node);
}

}