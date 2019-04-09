#include "node.h"

ml::vec3d Node::deformedPosition()
{
	return _g + _t;
}

ml::vec3d Node::deformPosition(ml::vec3f pos)
{
	return (_r*(pos - _g)) + _g + _t;
}

Node::Node(ml::vec3f g)
	: _g(g)
	, _r(ml::mat3d::identity())
	, _t(ml::vec3d::origin)
	, _w(1.)
{}

Node::Node()
	: _g(ml::vec3f::origin)
	, _r(ml::mat3d::identity())
	, _t(ml::vec3d::origin)
	, _w(1.)
{}