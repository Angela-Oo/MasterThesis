#include "deformation_graph_cgal_mesh.h"
#include "algo/registration/node_weighting.h"

std::vector<double> DeformationGraphCgalMesh::weights(const ml::vec3f & point, std::vector<vertex_descriptor>& nearest_nodes_indices) const
{
	auto property_map_nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<INode>>("node");
	if (property_map_nodes.second) {
		auto & nodes = property_map_nodes.first;
		std::vector<ml::vec3f> knn_nodes;
		for (auto & i : nearest_nodes_indices) {
			knn_nodes.push_back(nodes[i]->position());
		}
		return nodeDistanceWeighting(point, knn_nodes);
	}
	else {
		std::cout << "something went wrong" << std::endl;
	}
	return std::vector<double>();
}


std::vector<vertex_descriptor> DeformationGraphCgalMesh::nearestNodes(const ml::vec3f & point) const
{
	// tODO
	//vertex_descriptor nearest_node_index = _deformation_graph_knn->nearest_index(point);
	vertex_descriptor nearest_node_index;

	auto property_map_nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<INode>>("node");
	if (!property_map_nodes.second)
	{
		return std::vector<vertex_descriptor>();
	}
	auto & nodes = property_map_nodes.first;

	auto getNodeDistances = [&](vertex_descriptor vertex_index) {
		std::map<vertex_descriptor, double> node_distance;
		for (auto & v : _mesh.vertices_around_target(_mesh.halfedge(vertex_index)))
		{
			auto neighbor_point = _mesh.point(v);
			auto point = _mesh.point(vertex_index);
			node_distance[v] = CGAL::squared_distance(point, neighbor_point);
		}
		return node_distance;
	};

	std::map<vertex_descriptor, double> node_distance = getNodeDistances(nearest_node_index);
	if (node_distance.size() < _k) {
		for (auto & n : node_distance) {
			auto more_distance = getNodeDistances(n.first);
			node_distance.insert(more_distance.begin(), more_distance.end());
		}
	}

	std::vector<std::pair<vertex_descriptor, double>> sorted_node_distance;
	for (auto & d : node_distance) {
		sorted_node_distance.push_back(d);
	}

	std::sort(sorted_node_distance.begin(), sorted_node_distance.end(),
			  [](const std::pair<vertex_descriptor, double> & rhs, const std::pair<vertex_descriptor, double> & lhs)
	{
		return rhs.second < lhs.second;
	});

	std::vector<vertex_descriptor> indices;
	indices.push_back(nearest_node_index);
	for (int i = 0; i < _k && i < sorted_node_distance.size(); ++i)
		indices.push_back(sorted_node_distance[i].first);
	return indices;
}


ml::vec3f DeformationGraphCgalMesh::deformPoint(const ml::vec3f & point, const NearestNodes & nearest_nodes) const
{
	ml::vec3f deformed_point = ml::vec3f::origin;

	auto & property_map_nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<INode>>("node");
	if (!property_map_nodes.second) {
		std::cout << "something went wrong" << std::endl;
		return deformed_point;
	}

	auto & nodes = property_map_nodes.first;
	DeformationGraphMesh::Property_map<vertex_descriptor, std::shared_ptr<INode>> PropertyMapNodes;
	for (size_t i = 0; i < nearest_nodes.nodes.size() - 1; ++i)
	{
		auto vertex_index = nearest_nodes.nodes[i];
		double w = nearest_nodes.weights[i];

		ml::vec3f transformed_point = nodes[vertex_index]->deformPosition(point) * w;
		deformed_point += transformed_point;
	}

	ml::vec3f global_deformed_point = _global_rigid_deformation->deformPosition(deformed_point);
	return global_deformed_point;
}

Mesh::Vertex DeformationGraphCgalMesh::deformNode(vertex_descriptor node_index)
{
	auto & nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<INode>>("node").first;
	ml::vec3f pos = nodes[node_index]->deformedPosition();
	ml::vec3f normal = nodes[node_index]->deformedNormal();
	ml::vec3f global_pos = _global_rigid_deformation->deformPosition(pos);
	ml::vec3f global_normal = _global_rigid_deformation->deformNormal(normal);

	Mesh::Vertex v;
	v.position = global_pos;
	v.normal = global_normal;
	return v;
}


DeformationGraphCgalMesh::DeformationGraphCgalMesh(const DeformationGraphMesh & mesh)
	: _mesh(mesh)
{
	/*auto & vertices = mesh.getVertices();

	std::map<int, vertex_index> index_to_vertex_index;
	for (int index = 0; index < vertices.size(); ++index)
	{
		vertex_index v = boost::add_vertex(_graph);
		Node n(index, vertices[index].position, vertices[index].normal.getNormalized());
		boost::put(boost::get(node_t(), _graph), v, n);
		index_to_vertex_index[index] = v;
	}

	auto & indices = mesh.getIndices();
	for (int index = 0; index < indices.size(); ++index)
	{
		auto n1 = index_to_vertex_index[indices[index][0]];
		auto n2 = index_to_vertex_index[indices[index][1]];
		auto n3 = index_to_vertex_index[indices[index][2]];




		auto addEdge = [&](vertex_index v1, vertex_index v2, vertex_index v3)
		{
			if (boost::edge(v1, v2, _graph).second == false)
			{
				boost::add_edge(n2, n3, _graph);
			}
		};
		addEdge(n1, n2, n3);
		addEdge(n2, n3, n1);
		addEdge(n3, n1, n2);
	}
	_deformation_graph_knn = std::make_unique<GraphKNN<Graph, Node>>(_graph, _k + 1);

	auto& nodes = boost::get(node_t(), _graph);
	int count = 0;
	ml::vec3f global_position = ml::vec3f::origin;
	for (auto vp = boost::vertices(_graph); vp.first != vp.second; ++vp.first) {
		Node& node = nodes[*vp.first];
		global_position += node.position();
		count++;
	}
	global_position /= static_cast<float>(count);
	_global_rigid_deformation = Node(-1, global_position, ml::vec3d::eZ);*/
}



DeformationGraphCgalMesh::DeformationGraphCgalMesh(const DeformationGraphMesh & graph, const std::shared_ptr<INode> & global_rigid_deformation)
	: _mesh(graph)
	, _global_rigid_deformation(global_rigid_deformation)
{

	//_nodes = _mesh.property_map<vertex_descriptor, std::shared_ptr<INode>>("node");
	//_deformation_graph_knn = std::make_unique<GraphKNN<Graph, Node>>(_graph, _k + 1);
}


DeformationGraphCgalMesh::DeformationGraphCgalMesh(const DeformationGraphCgalMesh & deformation_graph)
	: _global_rigid_deformation(deformation_graph._global_rigid_deformation)
	, _mesh(deformation_graph._mesh)
{
	//_deformation_graph_knn = std::make_unique<GraphKNN<Graph, Node>>(_graph, _k + 1);
}

DeformationGraphCgalMesh & DeformationGraphCgalMesh::operator=(DeformationGraphCgalMesh other)
{
	if (&other == this)
		return *this;

	_global_rigid_deformation = other._global_rigid_deformation;
	_mesh = other._mesh;
	//_deformation_graph_knn = std::make_unique<GraphKNN<Graph, Node>>(_graph, _k + 1);
	return *this;
}
