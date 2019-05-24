#pragma once
#include "mLibInclude.h"
#include "algo/deformation_graph/i_node.h"

namespace ARAP {

class Node : public INode
{
private:
	ml::vec3d _r; // rotation matrix
	ml::vec3d _t; // translation vector	
	double _w; // weight
public:
	double * r() override { return (&_r)->getData(); }
	double * t() override { return (&_t)->getData(); }
	double * w() override { return &_w; }
public:
	Matrix rotation() const override;
	Vector translation() const override;
	double weight() const override { return _w; };
public:
	Node(const ml::vec3d & r, const ml::vec3d & t);
	Node();
	Node(const Node& node, bool inverse);
};


class Edge {
public:
	double _smooth_cost;
};

}
