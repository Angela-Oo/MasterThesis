#pragma once

#include "../mLibInclude.h"
#include "boost/graph/adjacency_list.hpp"

struct node_t {
	typedef boost::vertex_property_tag kind;
};

class Node
{
public:
	ml::vec3f _g; // node position
	ml::mat3d _r; // rotation matrix
	ml::vec3d _t; // translation vector
public:
	ml::vec3d deformedPosition();
public:
	Node(ml::vec3f g);
	Node();
};

class DeformationGraph
{
	typedef boost::property<node_t, Node> VertexProperty;
	typedef boost::adjacency_list<>::vertex_descriptor vertex_index;
	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperty> Graph;
private:
	const std::vector<ml::vec3f> & _points;
	const int k = 4;
	std::vector<std::vector<vertex_index>> _k_nearest_nodes_of_points;	
public:
	Node _global_rigid_deformation;
	Graph _graph;	
public:
	double weight(const ml::vec3f & point, Node & node, double dmax);
	std::vector<double> weights(const ml::vec3f & point, std::vector<Node>& nodes);
	ml::vec3f deformPoint(const ml::vec3f & point, std::vector<vertex_index> & k_nearest_node_ids);
	std::vector<ml::vec3f> getDeformedPoints();
public:
	DeformationGraph(const std::vector<ml::vec3f> & points, size_t number_of_nodes);
};
