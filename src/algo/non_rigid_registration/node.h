#pragma once
#include "mLibInclude.h"
#include "boost/graph/adjacency_list.hpp"

struct node_t {
	typedef boost::vertex_property_tag kind;
};

typedef boost::adjacency_list<>::vertex_descriptor vertex_index;

class Node
{
public:
	ml::vec3f _g; // node position
	ml::mat3d _r; // rotation matrix
	ml::vec3d _t; // translation vector
	ml::vec3d _n; // normal vector
	double _w; // weight
	//std::vector<vertex_index> _nearestNeighbors;
public:
	ml::vec3d deformedPosition() const;
	ml::vec3d deformedNormal() const;
	ml::vec3d deformPosition(ml::vec3f pos) const;
public:
	Node(ml::vec3f g, ml::vec3f n);
	Node();
	Node(const Node& node) = default;
};

typedef boost::property<node_t, Node> VertexProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperty> Graph;