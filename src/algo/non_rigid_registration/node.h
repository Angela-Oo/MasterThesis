#pragma once
#include "mLibInclude.h"
#include "graph_node_type.h"

namespace ED {
class Node
{
private:
	ml::vec3f _g; // node position
	ml::vec3d _n; // normal vector
	ml::mat3d _r; // rotation matrix
	ml::vec3d _t; // translation vector	
	double _w; // weight
	int _index;
public:
	ml::vec3f & g() { return _g; };
	ml::vec3d & n() { return _n; };
	double * r() { return (&_r)->getData(); };
	double * t() { return (&_t)->getData(); };
	double * w() { return &_w; }
public:
	ml::vec3f position() const { return _g; }
	ml::vec3d normal() const { return _n; }
	const ml::mat3d & rotation() const { return _r; }
	const ml::vec3d & translation() const { return _t; }
	double weight() const { return _w; }
	int index() const { return _index; }
public:
	ml::vec3d deformedPosition() const;
	ml::vec3d deformedNormal() const;
	ml::vec3d deformPosition(const ml::vec3f & pos) const;
public:
	Node(int index, const ml::vec3f & g, const ml::vec3d & n);
	Node(int index, const ml::vec3f & g, const ml::vec3d & n, const ml::mat3d & r, const ml::vec3d & t);
	Node();
	Node(const Node& node) = default;
};

typedef boost::property<node_t, Node> VertexProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperty> Graph;

}