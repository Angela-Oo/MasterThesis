#include "node.h"

ml::vec3d Node::deformedPosition()
{
	return _g + _t;
}

ml::vec3d Node::deformPosition(ml::vec3f pos)
{
	return (_r*(pos - _g)) + _g + _t;
}

Node::Node(ml::vec3f g, ml::vec3f n)
	: _g(g)
	, _n(n)
	, _r(ml::mat3d::identity())
	, _t(ml::vec3d::origin)
	, _w(1.)
{}

Node::Node()
	: _g(ml::vec3f::origin)
	, _n(ml::vec3f::eZ)
	, _r(ml::mat3d::identity())
	, _t(ml::vec3d::origin)
	, _w(1.)
{}