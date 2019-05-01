#pragma once
#include "mLibInclude.h"
#include "boost/graph/adjacency_list.hpp"

struct node_t {
	typedef boost::vertex_property_tag kind;
};

typedef boost::adjacency_list<>::vertex_descriptor vertex_index;

class Node
{
private:
	ml::vec3f _g; // node position
	ml::vec3d _n; // normal vector
	ml::mat3d _r; // rotation matrix
	//ml::vec3d _r; // rotation matrix
	ml::vec3d _t; // translation vector	
	double _w; // weight
	//std::vector<vertex_index> _nearestNeighbors;
public:
	ml::vec3f & g();
	ml::vec3d & n();
	double * r();
	double * t();	
	double * w();
public:
	ml::vec3f position() const;
	ml::vec3d normal() const;
	const ml::mat3d & rotation() const;
	const ml::vec3d & translation() const;
	double weight() const;
public:
	ml::vec3d deformedPosition() const;
	ml::vec3d deformedNormal() const;
	ml::vec3d deformPosition(const ml::vec3f & pos) const;
public:
	Node(const ml::vec3f & g, const ml::vec3d & n);
	//Node(const ml::vec3f & g, const ml::vec3d & n, const ml::vec3d & r, const ml::vec3d & t);
	Node(const ml::vec3f & g, const ml::vec3d & n, const ml::mat3d & r, const ml::vec3d & t);
	Node();
	Node(const Node& node) = default;
};

typedef boost::property<node_t, Node> VertexProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperty> Graph;