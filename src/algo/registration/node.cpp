#include "stdafx.h"

#include "node.h"
#include <ceres/rotation.h>

namespace ED
{

ml::vec3d Node::deformedPosition() const
{
	return _g + _t;
}

ml::vec3d Node::deformedNormal() const
{
	auto r_t = rotation().getTranspose();
	auto normal = r_t * _n;
	return normal.getNormalized();
}

ml::vec3d Node::deformPosition(const ml::vec3f & pos) const
{
	return (rotation()*(pos - _g)) + _g + _t;
}

ml::vec3d Node::deformNormal(const ml::vec3f & normal) const
{
	auto r_t = rotation();
	auto n = r_t * normal;
	return n.getNormalized();
}


Node::Node(int index, const ml::vec3f & g, const ml::vec3d & n)
	: Node(index, g, n, ml::mat3d::identity(), ml::vec3d::origin)
{}

Node::Node(int index, const ml::vec3f & g, const ml::vec3d & n, const ml::mat3d & r, const ml::vec3d & t)
	: _index(index)
	, _g(g)
	, _n(n)
	, _r(r)
	, _t(t)
	, _w(1.)
	, _fit_cost(0.)
	, _rotation_cost(0.)
	, _conf_cost(0.)
{}

Node::Node()
	: Node(-1, ml::vec3f::origin, ml::vec3f::eZ)
{}

Node::Node(const Node & node, bool inverse)
	: _index(node._index)
	, _g(node._g)
	, _n(node._n)
	, _r(node._r)
	, _t(node._t)
	, _w(node._w)
	, _fit_cost(node._fit_cost)
	, _rotation_cost(node._rotation_cost)
	, _conf_cost(node._conf_cost)
{
	if (inverse) {
		_g = node.deformedPosition();
		_n = node.deformedNormal();
		_r = node._r.getInverse();
		_t = -node._t;
	}
}


}