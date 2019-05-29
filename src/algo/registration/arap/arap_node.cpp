#include "stdafx.h"

#include "arap_node.h"
#include <ceres/rotation.h>


namespace ARAP {

Matrix Node::rotation() const
{
	ml::mat3d r;
	ceres::AngleAxisToRotationMatrix(_r.array, r.getData());
	auto r_t =  r.getTranspose(); // why??
	Matrix m(r_t(0, 0), r_t(0, 1), r_t(0, 2), r_t(1, 0), r_t(1, 1), r_t(1, 2), r_t(2, 0), r_t(2, 1), r_t(2, 2));
	return m;
}

Vector Node::translation() const
{
	return Vector(_t[0], _t[1], _t[2]);
}

Node::Node(const ml::vec3d & r, const ml::vec3d & t)
	: _r(r)
	, _t(t)
	, _w(1.)
{}

Node::Node()
	: Node(ml::vec3f::origin, ml::vec3f::origin)
{}

Node::Node(const Node & node, bool inverse)
	: _r(node._r)
	, _t(node._t)
	, _w(node._w)
{
	if (inverse) {
		_r = -node._r;
		_t = -node._t;
	}
}

}